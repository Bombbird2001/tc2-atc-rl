package com.bombbird.terminalcontrol2.gymnasium

import com.badlogic.ashley.core.Entity
import com.badlogic.ashley.utils.ImmutableArray
import com.badlogic.gdx.math.MathUtils
import com.bombbird.terminalcontrol2.ai.reward.RewardHandler
import com.bombbird.terminalcontrol2.components.AircraftInfo
import com.bombbird.terminalcontrol2.components.Altitude
import com.bombbird.terminalcontrol2.components.ApproachInfo
import com.bombbird.terminalcontrol2.components.CustomPosition
import com.bombbird.terminalcontrol2.components.ClearanceAct
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
import com.bombbird.terminalcontrol2.gymnasium.staterestore.restoreSnapshot
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.systems.TrajectorySystemInterval
import com.bombbird.terminalcontrol2.traffic.conflict.Conflict
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.traffic.despawnAircraft
import com.bombbird.terminalcontrol2.components.FlightType
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
import ktx.collections.getOrPut
import ktx.collections.set
import ktx.collections.toGdxArray
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.roundToInt

class PythonGymnasiumBridge(
    envId: String, private val conflictManager: ConflictManager, private val trajectorySystemInterval: TrajectorySystemInterval, private val evalMode: Boolean,
    goalReward: Float, mvaConflictPenalty: Float, aircraftConflictPenalty: Float, wakeConflictPenalty: Float,
): GymnasiumBridge {
    companion object {
        const val CONSTANT_SIZE = 28
        const val SIZE_PER_AIRCRAFT = 60
        const val SIZE_PER_INSTRUCTION = 6
        const val ADDITIONAL_PADDING = (8 - (CONSTANT_SIZE + MAX_RL_AIRCRAFT * SIZE_PER_INSTRUCTION) % 8) % 8
        const val SHM_FILE_SIZE = CONSTANT_SIZE + MAX_RL_AIRCRAFT * SIZE_PER_INSTRUCTION + ADDITIONAL_PADDING + MAX_RL_AIRCRAFT * SIZE_PER_AIRCRAFT

        const val FRAMES_PER_ACTION = 10 * 30
        const val CONFLICT_RESOLUTION_LOOKBACK_STEPS = 10

//        const val HDG_ACTION_MULTIPLIER = 5
//        const val ALT_ACTION_MULTIPLIER = 1000
//        const val ALT_ACTION_ADDER = 2000
//        const val SPD_ACTION_MULTIPLIER = 10
//        const val SPD_ACTION_ADDER = 160

        const val LOOP_EXIT_MS = 60000

        val TEST_OPTIONS_RESET_RECEIVED = intArrayOf()
        val CONFLICT_RESOLUTION_RESET_RECEIVED = mutableMapOf<String, IntArray>()

        fun getHeadingOptions(origAction: Int): IntArray {
            val allOptions = intArrayOf(0, 1, 2, 3, 4)
            return allOptions.filter { it != origAction }
                .sortedBy { abs(it - origAction) }
                .toIntArray()
        }

        fun getIasOptions(origAction: Int): IntArray {
            val allOptions = intArrayOf(0, 1, 2, 3, 4)
            return allOptions.filter { it < origAction }
                .sortedBy { origAction - it }
                .toIntArray()
        }
    }

    private var framesToAction = FRAMES_PER_ACTION
    private var lastActionTime = 0L
    private var trainerInitialized = false
    private var loopExited = false
    private var resetNeeded = false
    private var terminating = false
    private val agentIdToAircraft = Array<Entity?>(MAX_RL_AIRCRAFT) { null }
    private var aircraftAdded = 0
    private var spawnedInCurrentSession = 0
    private var landedInCurrentSession = 0

//    private val noLandAircraftTypeCount = GdxArrayMap<String, Int>()
//    private val noLandRecatCount = GdxArrayMap<Char, Int>()

    private val rewardHandler = RewardHandler(conflictManager, evalMode, goalReward, mvaConflictPenalty, aircraftConflictPenalty, wakeConflictPenalty)
    private val rlStateRestoreManager = RLStateRestoreManager(CONFLICT_RESOLUTION_LOOKBACK_STEPS)

    private val sharedMemoryIPC: SharedMemoryIPC = SharedMemoryIPCFactory.getSharedMemory(envId, SHM_FILE_SIZE)
    private val envName = "[env$envId]"

    private fun makeGhostAircraftEntity(callsign: String): Entity {
        // Create an entity that is safe for writeState serialization, but do NOT add it back into gs.aircraft.
        val ac = Aircraft(callsign, 0f, 0f, 0f, "B738", FlightType.ARRIVAL, false)
        if (ac.entity[Position.mapper] == null) ac.entity.add(Position(0f, 0f))
        if (ac.entity[Altitude.mapper] == null) ac.entity.add(Altitude(0f))
        if (ac.entity[IndicatedAirSpeed.mapper] == null) ac.entity.add(IndicatedAirSpeed(0f))
        if (ac.entity[Speed.mapper] == null) ac.entity.add(Speed())
        if (ac.entity[GroundTrack.mapper] == null) ac.entity.add(GroundTrack())
        if (ac.entity[ClearanceAct.mapper] == null) ac.entity.add(ClearanceAct())
        // Ensure track vector is non-zero to avoid NaNs in angle computations
        ac.entity[GroundTrack.mapper]?.trackVectorPxps?.let { v ->
            if (v.isZero) v.set(1f, 0f)
        }
        // Ensure latest clearance exists and has non-null fields that writeState serializes
        ac.entity[ClearanceAct.mapper]?.actingClearance?.clearanceState?.apply {
            vectorHdg = 0
            clearedAlt = 0
            clearedIas = 0
        }
        return ac.entity
    }

    private fun restoreAgentIdToAircraftFromCallsigns(callsigns: List<String?>?, gs: GameServer) {
        for (i in 0 until agentIdToAircraft.size) {
            val callsign = callsigns?.getOrNull(i)
            val restored = if (callsign != null) gs.aircraft.get(callsign) else null
            agentIdToAircraft[i] = when {
                callsign == null -> null
                restored != null -> restored.entity
                else -> makeGhostAircraftEntity(callsign)
            }
        }
    }

    private fun applyBridgeStateFromSnapshot(snapshot: com.bombbird.terminalcontrol2.gymnasium.staterestore.Snapshot, gs: GameServer) {
        snapshot.bridgeSpawnedInSession?.let { spawnedInCurrentSession = it }
        snapshot.bridgeLandedInSession?.let { landedInCurrentSession = it }
        snapshot.rewardHandlerState?.let { rewardHandler.applyState(it) }
        snapshot.bridgeAddedInSession?.let { aircraftAdded = it }
        restoreAgentIdToAircraftFromCallsigns(snapshot.bridgeAgentCallsigns, gs)
    }

    private fun applyBridgeStateFromBackup(
        spawned: Int,
        landed: Int,
        reward: com.bombbird.terminalcontrol2.gymnasium.staterestore.RewardHandlerSnapshotData,
        added: Int,
        agentCallsigns: List<String?>,
        gs: GameServer
    ) {
        spawnedInCurrentSession = spawned
        landedInCurrentSession = landed
        rewardHandler.applyState(reward)
        aircraftAdded = added
        restoreAgentIdToAircraftFromCallsigns(agentCallsigns, gs)
    }

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

//            if (landedInCurrentSession != AIRCRAFT_TO_SPAWN && landedInCurrentSession > 0) {
//                FileLog.info("$envName PythonGymnasiumBridge", "$spawnedInCurrentSession spawned in previous episode, of which $aircraftAdded added, $landedInCurrentSession landed, score: ${gs.score}")
//                for (ac in aircraft) {
//                    val icaoType = ac.value.entity[AircraftInfo.mapper]?.icaoType!!
//                    val recatType = ac.value.entity[AircraftInfo.mapper]?.aircraftPerf?.recat!!
//                    val currCount = noLandAircraftTypeCount.getOrPut(icaoType) { 0 }
//                    val currRecatCount = noLandRecatCount.getOrPut(recatType) { 0 }
//                    noLandAircraftTypeCount[icaoType] = currCount + 1
//                    noLandRecatCount[recatType] = currRecatCount + 1
//                    FileLog.info("$envName PythonGymnasiumBridge", "${ac.value.entity[AircraftInfo.mapper]?.icaoCallsign!!} did not land!")
//                }
//                FileLog.info("$envName PythonGymnasiumBridge", "AC array state: ${agentIdToAircraft.joinToString(" -- ") {
//                    it?.get(AircraftInfo.mapper)?.icaoCallsign ?: "NA"
//                }}")
//            }

            // Reset the agent ID to aircraft mapping
            for (i in 0 until agentIdToAircraft.size) agentIdToAircraft[i] = null
            landedInCurrentSession = 0
            gs.score = 0

            aircraftAdded = 0
            rlStateRestoreManager.clearSnapshots()
            resetAircraft()
            spawnedInCurrentSession = aircraft.size
            rewardHandler.rewardReset()

            val baseSnapshot = rlStateRestoreManager.getSnapshot(gs)
            val preWriteSpawned = spawnedInCurrentSession
            val preWriteLanded = landedInCurrentSession
            val preWriteAdded = aircraftAdded
            val preWriteAgentCallsigns = Array(agentIdToAircraft.size) { agentIdToAircraft[it]?.get(AircraftInfo.mapper)?.icaoCallsign }.toList()
            val preWriteRewardState = rewardHandler.getStateForSnapshot()

            writeState(aircraft)
            
            rlStateRestoreManager.addSnapshot(
                baseSnapshot.copy(
                    bridgeSpawnedInSession = preWriteSpawned,
                    bridgeLandedInSession = preWriteLanded,
                    bridgeAddedInSession = preWriteAdded,
                    bridgeAgentCallsigns = preWriteAgentCallsigns,
                    rewardHandlerState = preWriteRewardState
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
                // FileLog.info("$envName PythonGymnasiumBridge", "$noLandAircraftTypeCount\n$noLandRecatCount")
                loopExited = true
                stopServer()
                return
            }

//            println("${System.currentTimeMillis()} (Reset) Performing action")
            val recordedActions = performAction(aircraft)
            rlStateRestoreManager.updateLatestSnapshotActions(recordedActions)

            framesToAction = FRAMES_PER_ACTION
            lastActionTime = System.currentTimeMillis()
            return
        }

        framesToAction--
        if (framesToAction <= 0 && !resetNeeded) {
            // Capture state BEFORE writeState so that if we rollback to this snapshot,
            // we can safely execute writeState again without double-processing the frame
            val baseSnapshot = rlStateRestoreManager.getSnapshot(gs)
            val preWriteSpawned = spawnedInCurrentSession
            val preWriteLanded = landedInCurrentSession
            val preWriteAdded = aircraftAdded
            val preWriteAgentCallsigns = Array(agentIdToAircraft.size) { agentIdToAircraft[it]?.get(AircraftInfo.mapper)?.icaoCallsign }.toList()
            val preWriteRewardState = rewardHandler.getStateForSnapshot()

            val (isTerminating, conflicts) = writeState(aircraft)
            terminating = isTerminating

            var actionOverrides: MutableMap<String, IntArray>? = null
            var shouldAddSnapshot = true

            if (evalMode && conflicts.notEmpty() && rlStateRestoreManager.snapshotCount() >= CONFLICT_RESOLUTION_LOOKBACK_STEPS) {
                actionOverrides = resolveConflicts(conflicts, gs, aircraft, CONFLICT_RESOLUTION_LOOKBACK_STEPS, stopServer)

                // Reset if needsResetAfterStep received during conflict resolution (due to exceeding step limit)
                if (actionOverrides == CONFLICT_RESOLUTION_RESET_RECEIVED) {
                    FileLog.warn("$envName PythonGymnasiumBridge", "Received needsResetAfterStep during conflict resolution - resetting next step")
                    resetNeeded = true
                    terminating = false
                    return
                }

                // Regardless of whether the conflict resolution is successful, the resolveConflicts function will always
                // restore the snapshot to the appropriate one
                // - If unsuccessful, it will restore to the same snapshot as before resolution
                // - If successful, it will restore to the snapshot CONFLICT_RESOLUTION_LOOKBACK_STEPS steps ago, and
                // actionOverrides will be populated with the correct action overrides to avoid conflict
                if (actionOverrides != null) {
                    // Do not add the snapshot when actionOverrides is not null (successful) - it will
                    // have restored the appropriate snapshot whose state is already in the restoreManager
                    shouldAddSnapshot = false
                    
                    // Write the state again to ensure the state is consistent with the snapshot.
                    // Because we restored to T-n (which was captured pre-writeState), this safely calculates 
                    // T-n's rewards and state exactly once without double-counting.
                    // Step count modifier is -CONFLICT_RESOLUTION_LOOKBACK_STEPS since we want to "replace" the lookback steps
                    // and do not include them in the step count
                    val (isTerminating2, _) = writeState(aircraft, stepCountModifier = (-CONFLICT_RESOLUTION_LOOKBACK_STEPS).byte)
                    terminating = isTerminating2
                }
                // If actionOverrides == null, it restored to the backup taken right after the first writeState.
                // We do not call writeState again to avoid double-processing T.
            }

            if (shouldAddSnapshot) {
                rlStateRestoreManager.addSnapshot(
                    baseSnapshot.copy(
                        bridgeSpawnedInSession = preWriteSpawned,
                        bridgeLandedInSession = preWriteLanded,
                        bridgeAddedInSession = preWriteAdded,
                        bridgeAgentCallsigns = preWriteAgentCallsigns,
                        rewardHandlerState = preWriteRewardState
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
                // FileLog.info("$envName PythonGymnasiumBridge", "$noLandAircraftTypeCount\n$noLandRecatCount")
                loopExited = true
                stopServer()
                return
            }

//            println("${System.currentTimeMillis()} Performing action")
            val recordedActions = performAction(aircraft, actionOverrides ?: emptyMap())
            rlStateRestoreManager.updateLatestSnapshotActions(recordedActions)

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

    private fun resolveConflicts(conflicts: GdxArray<Conflict>, gs: GameServer, aircraft: GdxArrayMap<String, Aircraft>, stepsBack: Int, stopServer: () -> Unit): MutableMap<String, IntArray>? {
        // Save backup of current state T
        val backupSnapshot = rlStateRestoreManager.getSnapshot(gs)
        val backupSpawned = spawnedInCurrentSession
        val backupLanded = landedInCurrentSession
        val backupReward = rewardHandler.getStateForSnapshot()
        val backupAdded = aircraftAdded
        val backupAgentCallsigns = Array(agentIdToAircraft.size) { agentIdToAircraft[it]?.get(AircraftInfo.mapper)?.icaoCallsign }
        val backupShm = sharedMemoryIPC.readBytes(0, SHM_FILE_SIZE)

        val sortedConflicts = GdxArray<Conflict>().apply { addAll(conflicts) }
        sortedConflicts.sort(Comparator { c1, c2 ->
            val rank1 = when (c1.reason) {
                Conflict.MVA, Conflict.SID_STAR_MVA, Conflict.RESTRICTED -> 1
                Conflict.WAKE_INFRINGE -> 2
                else -> 3
            }
            val rank2 = when (c2.reason) {
                Conflict.MVA, Conflict.SID_STAR_MVA, Conflict.RESTRICTED -> 1
                Conflict.WAKE_INFRINGE -> 2
                else -> 3
            }
            rank1.compareTo(rank2)
        })

//        FileLog.info(
//            "$envName PythonGymnasiumBridge",
//            "CR start: latestSnapshotT=${rlStateRestoreManager.getSnapshotAt(1)?.timestep}, lookbackSteps=$stepsBack, conflictCount=${sortedConflicts.size}"
//        )

        val currentOverrides = mutableMapOf<String, IntArray>()

        for (conflict in sortedConflicts) {
            val resolved = resolveSingleConflict(conflict, gs, aircraft, stepsBack, currentOverrides, stopServer)
                ?: return CONFLICT_RESOLUTION_RESET_RECEIVED  // Return special indicator object if we need to reset
            if (!resolved) {
//                FileLog.warn("$envName PythonGymnasiumBridge", "Failed to resolve conflict (" +
//                        "${conflict.entity1[AircraftInfo.mapper]?.icaoCallsign} locCap=${conflict.entity1.has(LocalizerCaptured.mapper)}, " +
//                        "${conflict.entity2?.get(AircraftInfo.mapper)?.icaoCallsign} locCap=${conflict.entity2?.has(LocalizerCaptured.mapper)}, " +
//                        "${conflict.reason}), aborting resolution")
                // Restore backup and return false
                restoreSnapshot(backupSnapshot, gs)
                applyBridgeStateFromBackup(
                    spawned = backupSpawned,
                    landed = backupLanded,
                    reward = backupReward,
                    added = backupAdded,
                    agentCallsigns = backupAgentCallsigns.asList(),
                    gs = gs
                )
                
                sharedMemoryIPC.copyByteArray(0, ByteBuffer.wrap(backupShm))
//                FileLog.info(
//                    "$envName PythonGymnasiumBridge",
//                    "CR abort: restored backup state; latestSnapshotT=${rlStateRestoreManager.getSnapshotAt(1)?.timestep}"
//                )
                return null
            }
        }

        // Successfully resolved all conflicts!
        val snapshotTn = rlStateRestoreManager.restoreSnapshot(stepsBack, gs)
        applyBridgeStateFromSnapshot(snapshotTn, gs)
        // FileLog.info(
        //     "$envName PythonGymnasiumBridge",
        //     "CR success: restored to snapshotT=${snapshotTn.timestep} (nextT=${snapshotTn.timestep + 1}); landed=$landedInCurrentSession overrides=${currentOverrides.size} aircraft=${aircraft.size} agents=${agentIdToAircraft.filterNotNull().size}"
        // )
//        FileLog.info("$envName PythonGymnasiumBridge", "Resolved conflict by performing actions:\n${currentOverrides.toList().joinToString {
//            "${it.first}: ${it.second.joinToString(" ")}"
//        }}")

        return currentOverrides
    }

    private fun resolveSingleConflict(
        conflict: Conflict,
        gs: GameServer,
        aircraft: GdxArrayMap<String, Aircraft>,
        stepsBack: Int,
        currentOverrides: MutableMap<String, IntArray>,
        stopServer: () -> Unit
    ): Boolean? {
        val snapshotTn = rlStateRestoreManager.getSnapshotAt(stepsBack) ?: return false
//        val baseTimestep = snapshotTn.timestep
        
        val ac1 = conflict.entity1
        val callsign1 = ac1[AircraftInfo.mapper]?.icaoCallsign ?: run {
//            FileLog.info(
//                "$envName PythonGymnasiumBridge",
//                "CR resolveSingleConflict end: baseSnapshotT=$baseTimestep, success=true (missing callsign1)"
//            )
            return true
        }
        val ac2 = conflict.entity2
        val callsign2 = ac2?.get(AircraftInfo.mapper)?.icaoCallsign

        val ac1Loc = ac1.has(LocalizerCaptured.mapper)
        val ac2Loc = ac2?.has(LocalizerCaptured.mapper)
        val isMva = conflict.reason == Conflict.MVA || conflict.reason == Conflict.SID_STAR_MVA || conflict.reason == Conflict.RESTRICTED

        if (isMva) {
            val orig = currentOverrides[callsign1]?.get(0) ?: snapshotTn.actions[callsign1]?.get(0) ?: 2
            val options = getHeadingOptions(orig)
            val succ = testOptions(callsign1, options, "hdg", gs, aircraft, stepsBack, currentOverrides, stopServer)
            if (succ === TEST_OPTIONS_RESET_RECEIVED) return null
            if (succ != null) {
                currentOverrides[callsign1] = succ
//                FileLog.info(
//                    "$envName PythonGymnasiumBridge",
//                    "CR resolveSingleConflict end: baseSnapshotT=$baseTimestep, success=true (MVA) ac=$callsign1"
//                )
                return true
            }
        } else if (ac1Loc == ac2Loc) {
            // Either both are on LOC, or both are not on LOC - can only be normal conflict (not wake)
            // Choose IAS option if both on LOC, or heading option if both not on LOC
            val optionFunction = if (ac1Loc) PythonGymnasiumBridge::getIasOptions else PythonGymnasiumBridge::getHeadingOptions
            val optionName = if (ac1Loc) "ias" else "hdg"

            val orig1 = currentOverrides[callsign1]?.get(2) ?: snapshotTn.actions[callsign1]?.get(2) ?: 2
            val options1 = optionFunction(orig1)
            val succ1 = testOptions(callsign1, options1, optionName, gs, aircraft, stepsBack, currentOverrides, stopServer)
            if (succ1 === TEST_OPTIONS_RESET_RECEIVED) return null
            if (succ1 != null) {
                currentOverrides[callsign1] = succ1
//                FileLog.info(
//                    "$envName PythonGymnasiumBridge",
//                    "CR resolveSingleConflict end: baseSnapshotT=$baseTimestep, success=true, ac=$callsign1, onLoc=$ac1Loc"
//                )
                return true
            }
            if (callsign2 != null) {
                val orig2 = currentOverrides[callsign2]?.get(2) ?: snapshotTn.actions[callsign2]?.get(2) ?: 2
                val options2 = optionFunction(orig2)
                val succ2 = testOptions(callsign2, options2, optionName, gs, aircraft, stepsBack, currentOverrides, stopServer)
                if (succ2 === TEST_OPTIONS_RESET_RECEIVED) return null
                if (succ2 != null) {
                    currentOverrides[callsign2] = succ2
//                    FileLog.info(
//                        "$envName PythonGymnasiumBridge",
//                        "CR resolveSingleConflict end: baseSnapshotT=$baseTimestep, success=true, ac=$callsign2, onLoc=$ac2Loc"
//                    )
                    return true
                }
            }
        } else {
            // One of aircraft is on LOC, the other is not
            // Can be aircraft-aircraft conflict, or wake conflict (ac2 will be null if this is the case)
            val acToResolve = if (ac2 == null) ac1  // Wake conflict, resolve only ac1
            else if (ac1Loc) ac2 else ac1  // Aircraft-aircraft conflict, resolve aircraft not on LOC
            val acOnLoc = if (acToResolve == ac1) ac1Loc else ac2Loc!!
            val callsignToResolve = if (acToResolve == ac1) callsign1 else callsign2!!
            val optionFunction = if (acOnLoc) PythonGymnasiumBridge::getIasOptions else PythonGymnasiumBridge::getHeadingOptions
            val optionName = if (acOnLoc) "ias" else "hdg"

            val orig = currentOverrides[callsignToResolve]?.get(0) ?: snapshotTn.actions[callsignToResolve]?.get(0) ?: 2
            val options = optionFunction(orig)
            val succ = testOptions(callsignToResolve, options, optionName, gs, aircraft, stepsBack, currentOverrides, stopServer)
            if (succ === TEST_OPTIONS_RESET_RECEIVED) return null
            if (succ != null) {
                currentOverrides[callsignToResolve] = succ
//                FileLog.info(
//                    "$envName PythonGymnasiumBridge",
//                    "CR resolveSingleConflict end: baseSnapshotT=$baseTimestep, success=true " +
//                            "changed=$firstCallsign locCap=$ac1Loc kept=$secondCallsign locCap=$ac2Loc"
//                )
                return true
            }
        }

//        FileLog.info(
//            "$envName PythonGymnasiumBridge",
//            "CR resolveSingleConflict end: baseSnapshotT=$baseTimestep, success=false (reason=${conflict.reason}) ac1=$callsign1 ac2=$callsign2"
//        )
        return false
    }

    private fun testOptions(
        testAc: String,
        options: IntArray,
        optionType: String,
        gs: GameServer,
        aircraft: GdxArrayMap<String, Aircraft>,
        stepsBack: Int,
        currentOverrides: Map<String, IntArray>,
        stopServer: () -> Unit
    ): IntArray? {
        val snapshotTn = rlStateRestoreManager.getSnapshotAt(stepsBack) ?: return null
        val originalActions = snapshotTn.actions

        for (opt in options) {
            // val prevLanded = landedInCurrentSession
            // val prevAircraft = aircraft.size
            // val prevAgent = agentIdToAircraft.filterNotNull().size
            restoreSnapshot(snapshotTn, gs)
            applyBridgeStateFromSnapshot(snapshotTn, gs)

            // FileLog.info(
            //     "$envName PythonGymnasiumBridge",
            //     "CR testOptions: restored snapshotT=${snapshotTn.timestep} landed=$landedInCurrentSession, previous landed=$prevLanded; aircraft=${aircraft.size}, previous aircraft=$prevAircraft; agents=${agentIdToAircraft.filterNotNull().size}, previous agents=$prevAgent"
            // )

            var success = true

            for (step in 0 until CONFLICT_RESOLUTION_LOOKBACK_STEPS) {
                // Do not count this step since it is just for planning
                val (_, conflicts) = writeState(aircraft, stepCountModifier = -1)

                if (step > 0) {
                    val involvesTestAc = conflicts.any {
                        it.entity1[AircraftInfo.mapper]?.icaoCallsign == testAc ||
                        it.entity2?.get(AircraftInfo.mapper)?.icaoCallsign == testAc
                    }
                    if (involvesTestAc) {
                        success = false
                        break
                    }
                }

                sharedMemoryIPC.signalActionReady()

                // Return a special indicator object if we need to reset
                if (sharedMemoryIPC.needsResetAfterStep()) {
                    return TEST_OPTIONS_RESET_RECEIVED
                }

                if (!sharedMemoryIPC.waitForActionDone(LOOP_EXIT_MS)) {
                    FileLog.warn("$envName PythonGymnasiumBridge", "Update loop exited during resolution")
                    loopExited = true
                    return null
                }
                if (sharedMemoryIPC.readBytes(1, 1)[0] == 1.byte) {
                    loopExited = true
                    stopServer()
                    return null
                }

                val overrides = mutableMapOf<String, IntArray>()
                if (step == 0) {
                    overrides.putAll(currentOverrides)
                    val orig = originalActions[testAc] ?: intArrayOf(2, 2, 2)
                    val newAction = overrides[testAc]?.clone() ?: orig.clone()
                    when (optionType) {
                        "hdg" -> newAction[0] = opt
                        "ias" -> newAction[2] = opt
                    }
                    overrides[testAc] = newAction
                }

                performAction(aircraft, overrides)

                repeat(FRAMES_PER_ACTION) {
                    gs.engine.update(1 / 30f)
                }
            }

            if (success) {
                // Conflict check for final step - we do not even send this information to the agent so steps are not counted to begin with
                val (_, finalConflicts) = writeState(aircraft, stepCountModifier = -1)
                val involvesTestAc = finalConflicts.any {
                    it.entity1[AircraftInfo.mapper]?.icaoCallsign == testAc ||
                    it.entity2?.get(AircraftInfo.mapper)?.icaoCallsign == testAc
                }
                if (!involvesTestAc) {
                    val orig = originalActions[testAc] ?: intArrayOf(2, 2, 2)
                    val newAction = currentOverrides[testAc]?.clone() ?: orig.clone()
                    when (optionType) {
                        "hdg" -> newAction[0] = opt
                        "ias" -> newAction[2] = opt
                    }
                    return newAction
                }
            }
        }
        return null
    }

    private fun writeState(aircraft: GdxArrayMap<String, Aircraft>, stepCountModifier: Byte = 0): Pair<Boolean, GdxArray<Conflict>> {
        if (aircraft.size > MAX_RL_AIRCRAFT) {
            throw IllegalArgumentException("$envName Aircraft must have <= $MAX_RL_AIRCRAFT items, got ${aircraft.size} instead")
        }

//        val shouldTerminate = (if (conflicts.size > 0) 1 else 0).byte
        val shouldTerminate = 0.byte
        var nonTerminateCount = 0

        // Assign agent IDs to newly spawned aircraft, if any
        for (i in 0 until aircraft.size) {
            val ac = aircraft.getValueAt(i).entity
            if (agentIdToAircraft.contains(ac)) continue

            if (agentIdToAircraft[aircraftAdded] != null) throw IllegalStateException("$envName: Expected agent $aircraftAdded to be null before assignment")
            agentIdToAircraft[aircraftAdded] = ac
            aircraftAdded++
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
                stateArray.putInt(currAcInfo.aircraftPerf.appSpd.toInt())
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
        sharedMemoryIPC.setFloat(8, rewardHandler.aircraftConflictCountNoLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        sharedMemoryIPC.setFloat(12, rewardHandler.mvaConflictCount.toFloat() / AIRCRAFT_TO_SPAWN)
        sharedMemoryIPC.setFloat(16, rewardHandler.wakeConflictCountNoLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        sharedMemoryIPC.setFloat(20, rewardHandler.aircraftConflictCountLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        sharedMemoryIPC.setFloat(24, rewardHandler.wakeConflictCountLoc.toFloat() / AIRCRAFT_TO_SPAWN)

        // Copy all aircraft states
        sharedMemoryIPC.copyByteArray(CONSTANT_SIZE + MAX_RL_AIRCRAFT * SIZE_PER_INSTRUCTION + ADDITIONAL_PADDING, stateArray)

        // Action waiting flag
        sharedMemoryIPC.setByte(0, 1)

        // Step count modifier
        sharedMemoryIPC.setByte(2, stepCountModifier)

        for (agentId in acToRemove) {
            despawnAircraft(agentIdToAircraft[agentId]!!)
            agentIdToAircraft[agentId] = null
        }
        acToRemove.clear()

        val filteredConflicts = GdxArray<Conflict>()
        for (c in conflicts) {
            if (c.reason != Conflict.RL_WAKE_CONFLICT_INCREASED_MARGIN && c.reason != Conflict.RL_AIRCRAFT_CONFLICT_INCREASED_MARGIN) {
                filteredConflicts.add(c)
            }
        }

        return Pair(shouldTerminate == 1.byte || nonTerminateCount == 0, filteredConflicts)
    }

    private fun performAction(aircraft: GdxArrayMap<String, Aircraft>, overrides: Map<String, IntArray> = emptyMap()): Map<String, IntArray> {
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

        val actionsRecorded = mutableMapOf<String, IntArray>()

        for (currAgentID in 0 until agentIdToAircraft.size) {
            val instructionStartOffset = CONSTANT_SIZE + currAgentID * SIZE_PER_INSTRUCTION

            val targetAircraft = agentIdToAircraft[currAgentID] ?: continue
            val callsign = targetAircraft[AircraftInfo.mapper]?.icaoCallsign ?: continue

            var hdgOpt = sharedMemoryIPC.readShort(instructionStartOffset).toInt()
            var altOpt = bytes[instructionStartOffset + 2].toInt()
            var iasOpt = bytes[instructionStartOffset + 3].toInt()
            
            overrides[callsign]?.let {
                if (it[0] != -1) hdgOpt = it[0]
                if (it[1] != -1) altOpt = it[1]
                if (it[2] != -1) iasOpt = it[2]
            }

            actionsRecorded[callsign] = intArrayOf(hdgOpt, altOpt, iasOpt)

            if (bytes[instructionStartOffset + 4] != 1.byte) continue  // No clearance required
            if (targetAircraft.has(LandingRoll.mapper)) continue

            val isLocCap = targetAircraft.has(LocalizerCaptured.mapper)
            val pos = targetAircraft[Position.mapper]!!
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
            else when (hdgOpt) {
                0 -> -45f
                1 -> -10f
                2 -> 0f
                3 -> 10f
                4 -> 45f
                else -> throw IllegalArgumentException("Unexpected hdg action $hdgOpt")
            }
            val deltaAlt = if (isLocCap) 0
            else when (altOpt) {
                0 -> -3000
                1 -> -1000
                2 -> 0
                3 -> 1000
                4 -> 3000
                else -> throw IllegalArgumentException("Unexpected alt action $altOpt")
            }
            val deltaIas = if (isLocCap && distNm < 9.5f) 0 else when (iasOpt) {
                0 -> -30
                1 -> -10
                2 -> 0
                3 -> 10
                4 -> 30
                else -> throw IllegalArgumentException("Unexpected ias action $iasOpt")
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

        return actionsRecorded
    }
}

fun Boolean.toInt() = if (this) 1 else 0