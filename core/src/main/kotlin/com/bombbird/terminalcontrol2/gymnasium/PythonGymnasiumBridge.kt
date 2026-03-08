package com.bombbird.terminalcontrol2.gymnasium

import com.badlogic.ashley.core.Entity
import com.badlogic.ashley.utils.ImmutableArray
import com.badlogic.gdx.math.MathUtils
import com.bombbird.terminalcontrol2.ai.reward.RewardHandler
import com.bombbird.terminalcontrol2.components.AircraftInfo
import com.bombbird.terminalcontrol2.components.Altitude
import com.bombbird.terminalcontrol2.components.ApproachInfo
import com.bombbird.terminalcontrol2.components.CustomPosition
import com.bombbird.terminalcontrol2.components.GlideSlopeCaptured
import com.bombbird.terminalcontrol2.components.GroundTrack
import com.bombbird.terminalcontrol2.components.IndicatedAirSpeed
import com.bombbird.terminalcontrol2.components.LandingRoll
import com.bombbird.terminalcontrol2.components.LocalizerCaptured
import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.components.Speed
import com.bombbird.terminalcontrol2.components.VisualCaptured
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.global.AIRCRAFT_TO_SPAWN
import com.bombbird.terminalcontrol2.global.CHECK_AIRCRAFT_CONFLICT
import com.bombbird.terminalcontrol2.global.CHECK_MVA_CONFLICT
import com.bombbird.terminalcontrol2.global.ENABLE_TRAJECTORY_ALTITUDE_MASKING
import com.bombbird.terminalcontrol2.global.CHECK_WAKE_CONFLICT
import com.bombbird.terminalcontrol2.global.MAX_RL_AIRCRAFT
import com.bombbird.terminalcontrol2.global.SIMPLIFIED_LOC_CAP
import com.bombbird.terminalcontrol2.global.TRAJECTORY_CHECK_MAX_TIME_S
import com.bombbird.terminalcontrol2.gymnasium.ipc.SharedMemoryIPC
import com.bombbird.terminalcontrol2.gymnasium.ipc.SharedMemoryIPCFactory
import com.bombbird.terminalcontrol2.gymnasium.staterestore.RLStateRestoreManager
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.systems.TrajectorySystemInterval
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.traffic.despawnAircraft
import com.bombbird.terminalcontrol2.utilities.FileLog
import com.bombbird.terminalcontrol2.utilities.addNewClearanceToPendingClearances
import com.bombbird.terminalcontrol2.utilities.byte
import com.bombbird.terminalcontrol2.utilities.calculateDistanceBetweenPoints
import com.bombbird.terminalcontrol2.utilities.convertWorldAndRenderDeg
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import com.bombbird.terminalcontrol2.utilities.modulateHeading
import com.bombbird.terminalcontrol2.utilities.pxToNm
import ktx.ashley.get
import ktx.ashley.has
import ktx.collections.GdxArray
import ktx.collections.GdxArrayMap
import ktx.collections.GdxSet
import ktx.collections.toGdxArray
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.roundToInt

class PythonGymnasiumBridge(
    envId: String, private val conflictManager: ConflictManager, private val trajectorySystemInterval: TrajectorySystemInterval, evalMode: Boolean,
    goalReward: Float, mvaConflictPenalty: Float, aircraftConflictPenalty: Float, wakeConflictPenalty: Float,
): GymnasiumBridge {
    companion object {
        const val CONSTANT_SIZE = 20
        const val SIZE_PER_AIRCRAFT = 56
        const val SIZE_PER_INSTRUCTION = 6
        const val ADDITIONAL_PADDING = (8 - (CONSTANT_SIZE + MAX_RL_AIRCRAFT * SIZE_PER_INSTRUCTION) % 8) % 8
        const val SHM_FILE_SIZE = CONSTANT_SIZE + MAX_RL_AIRCRAFT * SIZE_PER_INSTRUCTION + ADDITIONAL_PADDING + MAX_RL_AIRCRAFT * SIZE_PER_AIRCRAFT

        const val FRAMES_PER_ACTION = 10 * 30

        const val HDG_ACTION_MULTIPLIER = 5
        const val ALT_ACTION_MULTIPLIER = 1000
        const val ALT_ACTION_ADDER = 2000
        const val SPD_ACTION_MULTIPLIER = 10
        const val SPD_ACTION_ADDER = 160

        const val LOOP_EXIT_MS = 60000
    }

    private var framesToAction = FRAMES_PER_ACTION
    private var lastActionTime = 0L
    private var trainerInitialized = false
    private var loopExited = false
    private var resetNeeded = false
    private var terminating = false
    private val agentIdToAircraft = Array<Entity?>(MAX_RL_AIRCRAFT) { null }
    private val assignedCallsigns = GdxSet<String>()
    private var spawnedInCurrentSession = 0
    private var landedInCurrentSession = 0

    private val rewardHandler = RewardHandler(conflictManager, evalMode, goalReward, mvaConflictPenalty, aircraftConflictPenalty, wakeConflictPenalty)
    private val rlStateRestoreManager = RLStateRestoreManager(5)

    private val sharedMemoryIPC: SharedMemoryIPC = SharedMemoryIPCFactory.getSharedMemory(envId, SHM_FILE_SIZE)
    private val envName = "[env$envId]"

    override fun getEpisodeSpawnCount(): Int {
        return spawnedInCurrentSession
    }

    override fun incrementSpawnCount() {
        spawnedInCurrentSession++
    }

    override fun update(
        aircraft: GdxArrayMap<String, Aircraft>, stopServer: () -> Unit,
        gs: GameServer, resetAircraft: () -> GdxArrayMap<String, Aircraft>
    ) {
        if (loopExited) return

        if (!trainerInitialized) {
            sharedMemoryIPC.waitForTrainerInitialized()
            trainerInitialized = true
            FileLog.info("$envName PythonGymnasiumBridge", "Trainer initialized")
        }

        // Check for reset sim event
        if (sharedMemoryIPC.needsResetSim()) {
//            FileLog.info("$envName PythonGymnasiumBridge", "Resetting state")
            resetNeeded = false

            // Reset the agent ID to aircraft mapping
            for (i in 0 until agentIdToAircraft.size) agentIdToAircraft[i] = null
            landedInCurrentSession = 0

            assignedCallsigns.clear()
            rlStateRestoreManager.clearSnapshots()
            resetAircraft()
            spawnedInCurrentSession = aircraft.size
            rewardHandler.rewardReset()
            writeState(aircraft)
            val baseSnapshot = rlStateRestoreManager.getSnapshot(gs)
            rlStateRestoreManager.addSnapshot(
                baseSnapshot.copy(
                    bridgeSpawnedInSession = spawnedInCurrentSession,
                    bridgeLandedInSession = landedInCurrentSession,
                    rewardHandlerState = rewardHandler.getStateForSnapshot()
                )
            )

            terminating = false
            sharedMemoryIPC.signalActionReady()
//            println("${System.currentTimeMillis()} Reset action ready")

            // Wait for action done event before continuing simulation
//            println("${System.currentTimeMillis()} Waiting action done (reset)")
            if (!sharedMemoryIPC.waitForActionDone(LOOP_EXIT_MS)) {
                // Assume RL program has exited, stop bridge loop
                FileLog.warn("$envName PythonGymnasiumBridge", "Update loop exited")
                loopExited = true
                return
            }

            // Check for exit flag
            if (sharedMemoryIPC.readBytes(1, 1)[0] == 1.byte) {
                FileLog.warn("$envName PythonGymnasiumBridge", "Training finished, exiting simulator")
                loopExited = true
                stopServer()
                return
            }

//            println("${System.currentTimeMillis()} (Reset) Performing action")
            performAction(aircraft)

            framesToAction = FRAMES_PER_ACTION
            lastActionTime = System.currentTimeMillis()
            return
        }

        framesToAction--
        if (framesToAction <= 0 && !resetNeeded) {
            terminating = writeState(aircraft)

            if (rlStateRestoreManager.snapshotCount() == 5) {
                // TODO Change rollback criteria to conflict detection and perform forced action selection
                val restoredSnapshot = rlStateRestoreManager.restoreSnapshot(2, gs)
                restoredSnapshot.bridgeSpawnedInSession?.let { spawnedInCurrentSession = it }
                restoredSnapshot.bridgeLandedInSession?.let { landedInCurrentSession = it }
                restoredSnapshot.rewardHandlerState?.let { rewardHandler.applyState(it) }

                // Mappings will be assigned again in writeState
                assignedCallsigns.clear()
                for (i in 0 until agentIdToAircraft.size) agentIdToAircraft[i] = null

                rlStateRestoreManager.removeFirstSnapshot()
                rlStateRestoreManager.removeFirstSnapshot()
                rlStateRestoreManager.removeFirstSnapshot()
                terminating = writeState(aircraft)
            } else {
                val baseSnapshot = rlStateRestoreManager.getSnapshot(gs)
                rlStateRestoreManager.addSnapshot(
                    baseSnapshot.copy(
                        bridgeSpawnedInSession = spawnedInCurrentSession,
                        bridgeLandedInSession = landedInCurrentSession,
                        rewardHandlerState = rewardHandler.getStateForSnapshot()
                    )
                )
            }

            // Send action ready event after writing state and action masks to shared memory
            sharedMemoryIPC.signalActionReady()
//            println("${System.currentTimeMillis()} Set action ready")

            if (sharedMemoryIPC.needsResetAfterStep() || terminating) {
//                println("$envName Reset requested after step: terminating is $terminating")
                // Reset requested, exit update so won't get blocked
                resetNeeded = true
                terminating = false
                return
            }

            // Wait for action done event before continuing simulation
            if (!sharedMemoryIPC.waitForActionDone(LOOP_EXIT_MS)) {
                // Assume RL program has exited, stop bridge loop
                FileLog.warn("$envName PythonGymnasiumBridge", "Update loop exited")
                loopExited = true
                return
            }

            // Check for exit flag
            if (sharedMemoryIPC.readBytes(1, 1)[0] == 1.byte) {
                FileLog.warn("$envName PythonGymnasiumBridge", "Training finished, exiting simulator")
                loopExited = true
                stopServer()
                return
            }

//            println("${System.currentTimeMillis()} Performing action")
            performAction(aircraft)

            framesToAction = FRAMES_PER_ACTION
            lastActionTime = System.currentTimeMillis()
        }

        // Max 5 minutes of waiting before considering as deadlocked
        if ((System.currentTimeMillis() - lastActionTime) > 5 * 60 * 1000) {
            FileLog.warn(
                "$envName PythonGymnasiumBridge",
                "Reset deadlock; terminating=$terminating, shouldTerminate=${sharedMemoryIPC.readBytes(1, 1)[0]}"
            )
            throw IllegalStateException("$envName: Deadlocked")
        }
    }

    private fun writeState(aircraft: GdxArrayMap<String, Aircraft>): Boolean {
        if (aircraft.size > MAX_RL_AIRCRAFT) {
            throw IllegalArgumentException("$envName Aircraft must have <= $MAX_RL_AIRCRAFT items, got ${aircraft.size} instead")
        }

//        val shouldTerminate = (if (conflicts.size > 0) 1 else 0).byte
        val shouldTerminate = 0.byte
        var nonTerminateCount = 0

        // Assign agent IDs to newly spawned aircraft, if any
        for (i in 0 until aircraft.size) {
            val ac = aircraft.getValueAt(i).entity
            val callsign = ac[AircraftInfo.mapper]!!.icaoCallsign
            if (assignedCallsigns.contains(callsign)) continue

            if (agentIdToAircraft[assignedCallsigns.size] != null) throw IllegalStateException("$envName: Expected agent ${assignedCallsigns.size} to be null before assignment")
            agentIdToAircraft[assignedCallsigns.size] = ac
            assignedCallsigns.add(callsign)
        }

        val conflicts = if (CHECK_AIRCRAFT_CONFLICT || CHECK_MVA_CONFLICT || CHECK_WAKE_CONFLICT) {
            // Conflict check
            conflictManager.getConflictsRL(ImmutableArray(agentIdToAircraft.filterNotNull().toGdxArray()))
        } else GdxArray()
        val predictedConflicts = if (ENABLE_TRAJECTORY_ALTITUDE_MASKING) {
            trajectorySystemInterval.trajectoryManager.checkTrajectoryConflictsRL(trajectorySystemInterval.trajectoryTimeStates)
        } else GdxArray()

        val acRewards = rewardHandler.rewardStep(agentIdToAircraft, aircraft, conflicts)

        val stateArray = ByteBuffer.allocate(MAX_RL_AIRCRAFT * SIZE_PER_AIRCRAFT).order(ByteOrder.nativeOrder())
        val acToRemove = GdxArray<Int>()
        for (currAgentID in 0 until agentIdToAircraft.size) {
            val currAircraft = agentIdToAircraft[currAgentID]

            if (currAircraft == null) {
                stateArray.position(stateArray.position() + SIZE_PER_AIRCRAFT - 7)
                stateArray.put(0)  // Aircraft does not exist
                stateArray.put(0)  // Termination flag (NA)
                stateArray.put(0)  // Action mask (NA)
            } else {
                val currAcInfo = currAircraft[AircraftInfo.mapper]!!
                val currPos = currAircraft[Position.mapper]!!
                val currAlt = currAircraft[Altitude.mapper]!!
                val currIas = currAircraft[IndicatedAirSpeed.mapper]!!
                val currSpd = currAircraft[Speed.mapper]!!
                val currGroundTrack = currAircraft[GroundTrack.mapper]!!
                val currHdg = modulateHeading(convertWorldAndRenderDeg(currGroundTrack.trackVectorPxps.angleDeg()))
                val currPrevClearance = getLatestClearanceState(currAircraft)!!
                val currLocCap = if (currAircraft.has(LocalizerCaptured.mapper)) 1.byte else 0.byte

                var currShouldTerminate = shouldTerminate

                if (currLocCap == 1.byte && SIMPLIFIED_LOC_CAP) {
                    currShouldTerminate = 1
                    acToRemove.add(currAgentID)
                } else if (!aircraft.containsKey(currAcInfo.icaoCallsign)) {
                    currShouldTerminate = 1
                    agentIdToAircraft[currAgentID] = null
                    landedInCurrentSession++
                }

                // Reward, ICAO type, x, y, alt, ias, track, track rate, vertical speed, cleared alt, cleared hdg, cleared IAS, LOC cap, mask
                stateArray.putFloat(acRewards[currAgentID]!!)
                for (c in currAcInfo.icaoType) {
                    stateArray.put(c.code.toByte())
                }
                stateArray.putFloat(currPos.x)
                stateArray.putFloat(currPos.y)
                stateArray.putFloat(currAlt.altitudeFt)
                stateArray.putFloat(currIas.iasKt)
                stateArray.putFloat(currHdg)
                stateArray.putFloat(currSpd.angularSpdDps)
                stateArray.putFloat(currSpd.vertSpdFpm)
                stateArray.putInt(currPrevClearance.clearedAlt)
                stateArray.putInt(currPrevClearance.vectorHdg?.toInt() ?: currHdg.toInt())
                stateArray.putInt(currPrevClearance.clearedIas.toInt())
                stateArray.put(currLocCap)
                stateArray.put(1)  // Aircraft exists
                stateArray.put(currShouldTerminate)
                nonTerminateCount += 1 - currShouldTerminate
                var altMask = 15
                val ongoingConflict = conflicts.find { it.entity1 == currAircraft || it.entity2 == currAircraft }
                if (ongoingConflict == null) for (predConflict in predictedConflicts) {
                    if (predConflict.aircraft1 != currAircraft && predConflict.aircraft2 != currAircraft) continue
                    if (predConflict.advanceTimeS > TRAJECTORY_CHECK_MAX_TIME_S) continue

                    // Masking rules:
                    // If aircraft is not currently in conflict, and a conflict is predicted on current trajectory that:
                    // 1. Occurs at about the same altitude as cleared altitude, mask the "no change" action
                    // 2. Occurs at lower altitude than cleared altitude (which is above current aircraft altitude), mask the "+1000" and "no change" actions
                    // 3. Occurs at higher altitude than cleared altitude (which is below current aircraft altitude), mask the "-3000", "-1000" and "no change action"
                    if (abs(predConflict.altFt - currPrevClearance.clearedAlt) <= 25) altMask -= 2
                    else if (predConflict.altFt < currPrevClearance.clearedAlt) altMask -= 3
                    else if (predConflict.altFt > currPrevClearance.clearedAlt) altMask -= 14

                    break
                }
                stateArray.put(altMask.byte)
            }

            stateArray.put(currAgentID.byte)
            stateArray.position(stateArray.position() + 3)  // 3 bytes padding
        }

        // Write miscellaneous metrics
        sharedMemoryIPC.setFloat(4, landedInCurrentSession.toFloat() / AIRCRAFT_TO_SPAWN)
        sharedMemoryIPC.setFloat(8, rewardHandler.aircraftConflictCount.toFloat() / AIRCRAFT_TO_SPAWN)
        sharedMemoryIPC.setFloat(12, rewardHandler.mvaConflictCount.toFloat() / AIRCRAFT_TO_SPAWN)
        sharedMemoryIPC.setFloat(16, rewardHandler.wakeConflictCount.toFloat() / AIRCRAFT_TO_SPAWN)

        // Copy all aircraft states
        sharedMemoryIPC.copyByteArray(CONSTANT_SIZE + MAX_RL_AIRCRAFT * SIZE_PER_INSTRUCTION + ADDITIONAL_PADDING, stateArray)

        // Action waiting flag
        sharedMemoryIPC.setByte(0, 1)

        for (agentId in acToRemove) {
            despawnAircraft(agentIdToAircraft[agentId]!!)
            agentIdToAircraft[agentId] = null
        }
        acToRemove.clear()

        return shouldTerminate == 1.byte || nonTerminateCount == 0
    }

    private fun performAction(aircraft: GdxArrayMap<String, Aircraft>) {
        if (aircraft.size > MAX_RL_AIRCRAFT) {
            throw IllegalArgumentException("$envName Aircraft must have <= $MAX_RL_AIRCRAFT items, got ${aircraft.size} instead")
        }

        val bytes = sharedMemoryIPC.readBytes(0, SHM_FILE_SIZE)
        val proceedFlag = bytes[0]
        if (proceedFlag.toInt() != 1) throw IllegalStateException("$envName ProceedFlag must be 1")
        // Reset action waiting flag
        sharedMemoryIPC.setByte(0, 0)
//        println("${System.currentTimeMillis()} Proceed unset")

//        println("Selected aircraft: $acIndex; aircraft count: ${aircraft.size}")

        for (currAgentID in 0 until agentIdToAircraft.size) {
            val instructionStartOffset = CONSTANT_SIZE + currAgentID * SIZE_PER_INSTRUCTION

            val targetAircraft = agentIdToAircraft[currAgentID] ?: continue

            if (bytes[instructionStartOffset + 4] != 1.byte) continue  // No clearance required
            if (targetAircraft.has(LandingRoll.mapper)) continue

            val isLocCap = targetAircraft.has(LocalizerCaptured.mapper)
            val pos = targetAircraft.get(Position.mapper)!!
            val appEntity = targetAircraft[GlideSlopeCaptured.mapper]?.gsApp ?: targetAircraft[LocalizerCaptured.mapper]?.locApp ?: targetAircraft[VisualCaptured.mapper]?.visApp
            val rwyThrPos = appEntity?.get(ApproachInfo.mapper)?.rwyObj?.entity?.get(CustomPosition.mapper)
            val distNm = rwyThrPos?.let {
                pxToNm(calculateDistanceBetweenPoints(pos.x, pos.y, rwyThrPos.x, rwyThrPos.y))
            } ?: 999f

            val prevClearance = getLatestClearanceState(targetAircraft)!!
            val prevHdg = prevClearance.vectorHdg
            val prevAlt = prevClearance.clearedAlt
            val prevIas = prevClearance.clearedIas

            // Reduced action space v1
            val deltaHdg = if (isLocCap) 0f
            else when (val opt = sharedMemoryIPC.readShort(instructionStartOffset).toInt()) {
                0 -> -45f
                1 -> -10f
                2 -> 0f
                3 -> 10f
                4 -> 45f
                else -> throw IllegalArgumentException("Unexpected hdg action $opt")
            }
            val deltaAlt = if (isLocCap) 0
            else when (val opt = bytes[instructionStartOffset + 2].toInt()) {
                0 -> -3000
                1 -> -1000
                2 -> 0
                3 -> 1000
                4 -> 3000
                else -> throw IllegalArgumentException("Unexpected alt action $opt")
            }
            val deltaIas = if (isLocCap && distNm < 9.5f) 0 else when (val opt = bytes[instructionStartOffset + 3].toInt()) {
                0 -> -30
                1 -> -10
                2 -> 0
                3 -> 10
                4 -> 30
                else -> throw IllegalArgumentException("Unexpected ias action $opt")
            }

            // Reduced action space v3
//            val deltaHdg = when (val opt = sharedMemoryIPC.readShort(instructionStartOffset).toInt()) {
//                0 -> -45f
//                1 -> -20f
//                2 -> -10f
//                3 -> 0f
//                4 -> 10f
//                5 -> 20f
//                6 -> 45f
//                else -> throw IllegalArgumentException("Unexpected hdg action $opt")
//            }
//            val deltaAlt = when (val opt = bytes[instructionStartOffset + 2].toInt()) {
//                0 -> -3000
//                1 -> -1000
//                2 -> 0
//                3 -> 1000
//                else -> throw IllegalArgumentException("Unexpected alt action $opt")
//            }
//            val deltaIas = when (val opt = bytes[instructionStartOffset + 3].toInt()) {
//                0 -> -30
//                1 -> -10
//                2 -> 0
//                3 -> 10
//                else -> throw IllegalArgumentException("Unexpected ias action $opt")
//            }

            val minSpd = when {
                distNm < 4 -> max(targetAircraft[AircraftInfo.mapper]?.aircraftPerf?.appSpd!!.toInt(), 150)
                distNm < 8 -> 150
                else -> 160
            }
            val clearedHdg = if (prevHdg == null) prevHdg else modulateHeading(prevHdg + deltaHdg).roundToInt().toShort()
            val clearedAlt = MathUtils.clamp(prevAlt + deltaAlt, 2000, 15000)
            val clearedIas = MathUtils.clamp(prevIas + deltaIas, minSpd, 250).toShort()
            val changed = prevClearance.clearedAlt != clearedAlt || prevClearance.vectorHdg != clearedHdg || prevClearance.clearedIas != clearedIas

            if (changed) {
                val clearanceState = prevClearance.copy(vectorHdg = clearedHdg, clearedAlt = clearedAlt, clearedIas = clearedIas)
                addNewClearanceToPendingClearances(targetAircraft, clearanceState, 0)
            }
        }
    }
}

fun Boolean.toInt() = if (this) 1 else 0