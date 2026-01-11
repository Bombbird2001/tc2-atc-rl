package com.bombbird.terminalcontrol2.gymnasium

import com.badlogic.ashley.core.Entity
import com.badlogic.ashley.utils.ImmutableArray
import com.bombbird.terminalcontrol2.components.AircraftInfo
import com.bombbird.terminalcontrol2.components.Altitude
import com.bombbird.terminalcontrol2.components.ApproachChildren
import com.bombbird.terminalcontrol2.components.GroundTrack
import com.bombbird.terminalcontrol2.components.IndicatedAirSpeed
import com.bombbird.terminalcontrol2.components.LocalizerCaptured
import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.components.Speed
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.global.CHECK_CONFLICT
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.LOC_CAP_REWARD
import com.bombbird.terminalcontrol2.global.MAX_RL_AIRCRAFT
import com.bombbird.terminalcontrol2.global.PER_STEP_PENALTY
import com.bombbird.terminalcontrol2.global.SIMPLIFIED_LOC_CAP
import com.bombbird.terminalcontrol2.gymnasium.ipc.SharedMemoryIPC
import com.bombbird.terminalcontrol2.gymnasium.ipc.SharedMemoryIPCFactory
import com.bombbird.terminalcontrol2.navigation.distPxFromLoc
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.traffic.despawnAircraft
import com.bombbird.terminalcontrol2.utilities.FileLog
import com.bombbird.terminalcontrol2.utilities.addNewClearanceToPendingClearances
import com.bombbird.terminalcontrol2.utilities.byte
import com.bombbird.terminalcontrol2.utilities.convertWorldAndRenderDeg
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import com.bombbird.terminalcontrol2.utilities.modulateHeading
import ktx.ashley.get
import ktx.ashley.has
import ktx.collections.GdxArray
import ktx.collections.GdxArrayMap
import ktx.collections.GdxSet
import ktx.collections.set
import ktx.collections.toGdxArray
import java.nio.ByteBuffer
import java.nio.ByteOrder

class PythonGymnasiumBridge(envId: String): GymnasiumBridge {
    companion object {
        const val CONSTANT_SIZE = 4
        const val SIZE_PER_AIRCRAFT = 52
        const val SIZE_PER_INSTRUCTION = 6
        const val ADDITIONAL_PADDING = 2
        const val SHM_FILE_SIZE = CONSTANT_SIZE + MAX_RL_AIRCRAFT * SIZE_PER_INSTRUCTION + ADDITIONAL_PADDING + MAX_RL_AIRCRAFT * SIZE_PER_AIRCRAFT

        const val FRAMES_PER_ACTION = 10 * 30

        const val HDG_ACTION_MULTIPLIER = 5
        const val ALT_ACTION_MULTIPLIER = 1000
        const val ALT_ACTION_ADDER = 2000
        const val SPD_ACTION_MULTIPLIER = 10
        const val SPD_ACTION_ADDER = 160

        const val LOOP_EXIT_MS = 15000
    }

    var framesToAction = FRAMES_PER_ACTION
    var trainerInitialized = false
    var loopExited = false
    var resetNeeded = false
    var terminating = false
    val acPrevLocDistPx: GdxArrayMap<String, Float> = GdxArrayMap()
    val acPrevAlt: GdxArrayMap<String, Float> = GdxArrayMap()
    val acOnLoc: HashSet<String> = HashSet()
    val agentIdToAircraft = Array<Entity?>(MAX_RL_AIRCRAFT) { null }
    var assignedCallsigns = GdxSet<String>()
    var targetApproach = lazy {
        GAME.gameServer?.airports?.get(0)?.entity?.get(ApproachChildren.mapper)?.approachMap?.get("ILS 02L")!!
    }
//    var clearancesChangePenalty = 0f

    val conflictManager = ConflictManager()

    private val sharedMemoryIPC: SharedMemoryIPC = SharedMemoryIPCFactory.getSharedMemory(envId, SHM_FILE_SIZE)
    private val envName = "[env$envId]"

    override fun update(aircraft: GdxArrayMap<String, Aircraft>, resetAircraft: () -> GdxArrayMap<String, Aircraft>) {
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

            assignedCallsigns.clear()
            resetAircraft()
            writeState(aircraft)

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
//            println("${System.currentTimeMillis()} (Reset) Performing action")
            performAction(aircraft)

            framesToAction = FRAMES_PER_ACTION
            return
        }

        framesToAction--
        if (framesToAction <= 0 && !resetNeeded) {
            terminating = writeState(aircraft)

            // Send action ready event after writing state to shared memory
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
//            println("${System.currentTimeMillis()} Performing action")
            performAction(aircraft)

            framesToAction = FRAMES_PER_ACTION
        }

        if (framesToAction < -500000) {
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

        // Proceed flag
        sharedMemoryIPC.setByte(0, 1)

        val conflicts = if (CHECK_CONFLICT) {
            // Conflict check
            conflictManager.getConflictsRL(ImmutableArray(aircraft.values().map { it.entity }.toGdxArray()))
        } else GdxArray()
        val shouldTerminate = (if (conflicts.size > 0) 1 else 0).byte
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

        val stateArray = ByteBuffer.allocate(MAX_RL_AIRCRAFT * SIZE_PER_AIRCRAFT).order(ByteOrder.nativeOrder())
        val acToRemove = GdxArray<Entity>()
        for (currAgentID in 0 until agentIdToAircraft.size) {
            val currAircraft = agentIdToAircraft[currAgentID]

            if (currAircraft == null) {
                stateArray.position(stateArray.position() + SIZE_PER_AIRCRAFT - 3)
                stateArray.put(0)  // Aircraft does not exist
                stateArray.put(0)  // Termination flag (NA)
            } else {
                // Aircraft x, y, alt, gs, track
                val currAcInfo = currAircraft[AircraftInfo.mapper]!!
                val currPos = currAircraft[Position.mapper]!!
                val currAlt = currAircraft[Altitude.mapper]!!
                val currIas = currAircraft[IndicatedAirSpeed.mapper]!!
                val currSpd = currAircraft[Speed.mapper]!!
                val currGroundTrack = currAircraft[GroundTrack.mapper]!!
                val currHdg = modulateHeading(convertWorldAndRenderDeg(currGroundTrack.trackVectorPxps.angleDeg()))
                val currPrevClearance = getLatestClearanceState(currAircraft)!!
                val currLocCap = if (currAircraft.has(LocalizerCaptured.mapper)) 1.byte else 0.byte
                var acReward = 0f
                var currShouldTerminate = shouldTerminate

                if (currLocCap == 1.byte) {
                    // If aircraft has previously captured LOC, ignore its rewards
                    if (acOnLoc.contains(currAcInfo.icaoCallsign)) {
                        continue
                    }

                    // Lump sum reward on LOC capture TODO depending on intercept angle (lower angle = higher reward)?
                    acReward += LOC_CAP_REWARD
                    acOnLoc.add(currAcInfo.icaoCallsign)
                    acPrevLocDistPx.removeKey(currAcInfo.icaoCallsign)
                    acPrevAlt.removeKey(currAcInfo.icaoCallsign)
                    FileLog.info("$envName PythonGymnasiumBridge", "${currAcInfo.icaoCallsign} captured LOC")

                    if (SIMPLIFIED_LOC_CAP) {
                        acToRemove.add(currAircraft)
                        currShouldTerminate = 1
                    }
                } else {
                    acOnLoc.remove(currAcInfo.icaoCallsign)
                }

                // Reward from previous action
                // Constant -0.03 per time step + decrease in distance towards LOC line segment
                // + decrease in altitude
                val newLocDistPx = distPxFromLoc(currPos, targetApproach.value.entity, 6)
                if (acPrevLocDistPx.containsKey(currAcInfo.icaoCallsign)) {
                    val distReward = (acPrevLocDistPx[currAcInfo.icaoCallsign] - newLocDistPx) / 1600
                    val altReward = (acPrevAlt[currAcInfo.icaoCallsign] - currAlt.altitudeFt) / 12000
                    acReward += distReward + altReward - PER_STEP_PENALTY
                }

                // TODO Assign negative reward for conflict involving this aircraft
                // Smaller negative reward for other aircraft

                // Clearance change penalty from previous clearance
//            reward -= clearancesChangePenalty

                // Discourage aircraft from loitering too long close to LOC
//            if (newLocDistPx < nmToPx(4) && currAlt.altitudeFt <= 6010) totalAcReward -= 0.06f

                // Reward, ICAO type, x, y, alt, ias, track, track rate, vertical speed, cleared alt, cleared hdg, cleared IAS, LOC cap, mask
                stateArray.putFloat(acReward)
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

                acPrevLocDistPx[currAcInfo.icaoCallsign] = newLocDistPx
                acPrevAlt[currAcInfo.icaoCallsign] = currAlt.altitudeFt
            }

            stateArray.put(currAgentID.byte)
        }
        sharedMemoryIPC.copyByteArray(CONSTANT_SIZE + MAX_RL_AIRCRAFT * SIZE_PER_INSTRUCTION + ADDITIONAL_PADDING, stateArray)

        for (ac in acToRemove) despawnAircraft(ac)
        acToRemove.clear()

        return shouldTerminate == 1.byte || nonTerminateCount == 0
    }

    private fun performAction(aircraft: GdxArrayMap<String, Aircraft>) {
        if (aircraft.size > MAX_RL_AIRCRAFT) {
            throw IllegalArgumentException("$envName Aircraft must have <= $MAX_RL_AIRCRAFT items, got ${aircraft.size} instead")
        }

//        val targetAircraft = aircraft.getValueAt(0).entity

        val bytes = sharedMemoryIPC.readBytes(0, SHM_FILE_SIZE)
        val proceedFlag = bytes[0]
        if (proceedFlag.toInt() != 1) throw IllegalStateException("$envName ProceedFlag must be 1")
        sharedMemoryIPC.setByte(0, 0) // Reset proceed flag
//        println("${System.currentTimeMillis()} Proceed unset")

//        println("Selected aircraft: $acIndex; aircraft count: ${aircraft.size}")
//        println("Issue instruction: $issueInstruction, clearedHdg: $clearedHdg, clearedAlt: $clearedAlt, clearedIas: $clearedIas")

//        clearancesChangePenalty = 0f

        for (i in 0 until aircraft.size) {
            val instructionStartOffset = CONSTANT_SIZE + i * SIZE_PER_INSTRUCTION

            val targetAircraft = aircraft.getValueAt(i).entity

            if (bytes[instructionStartOffset + 4] != 1.byte || targetAircraft.has(LocalizerCaptured.mapper)) continue  // No clearance required
            val clearedHdg = (sharedMemoryIPC.readShort(instructionStartOffset) * HDG_ACTION_MULTIPLIER).toShort()
            val clearedAlt = bytes[instructionStartOffset + 2] * ALT_ACTION_MULTIPLIER + ALT_ACTION_ADDER
            val clearedIas = (bytes[instructionStartOffset + 3] * SPD_ACTION_MULTIPLIER + SPD_ACTION_ADDER).toShort()

            val prevClearance = getLatestClearanceState(targetAircraft)!!
            val changed = prevClearance.clearedAlt != clearedAlt || prevClearance.vectorHdg != clearedHdg || prevClearance.clearedIas != clearedIas

            if (changed) {
                val clearanceState = prevClearance.copy(vectorHdg = clearedHdg, clearedAlt = clearedAlt, clearedIas = clearedIas)
                addNewClearanceToPendingClearances(targetAircraft, clearanceState, 0)
            }
        }
    }
}

fun Boolean.toInt() = if (this) 1 else 0