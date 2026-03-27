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
import com.bombbird.terminalcontrol2.components.SpawnGroup
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
        const val FRAMES_PER_ACTION = 10 * 30
        const val CONFLICT_RESOLUTION_NO_LOC_LOOKBACK_STEPS = 10
        const val CONFLICT_RESOLUTION_LOC_LOOKBACK_STEPS = 40

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
    private var allMvaConflictCount = 0
    private var allAircraftConflictCountNoLoc = 0
    private var allAircraftConflictCountLoc = 0
    private var allWakeConflictCountNoLoc = 0
    private var allWakeConflictCountLoc = 0

//    private val noLandAircraftTypeCount = GdxArrayMap<String, Int>()
//    private val noLandRecatCount = GdxArrayMap<Char, Int>()

    private val baseSize = 4
    private val metricsHandler = MetricsHandler(baseSize)
    private val metricsPadding = (8 - MAX_RL_AIRCRAFT % 8) % 8
    private val constantSize = baseSize + metricsHandler.size + metricsPadding
    private val sizePerAircraft = 60
    private val sizePerInstruction = 6
    private val additionalPadding = (8 - (constantSize + MAX_RL_AIRCRAFT * sizePerInstruction) % 8) % 8
    private val shmFileSize = constantSize + MAX_RL_AIRCRAFT * sizePerInstruction + additionalPadding + MAX_RL_AIRCRAFT * sizePerAircraft

    private val rewardHandler = RewardHandler(conflictManager, evalMode, goalReward, mvaConflictPenalty, aircraftConflictPenalty, wakeConflictPenalty)
    private val rlStateRestoreManager = RLStateRestoreManager(max(CONFLICT_RESOLUTION_NO_LOC_LOOKBACK_STEPS, CONFLICT_RESOLUTION_LOC_LOOKBACK_STEPS))
    private val sharedMemoryIPC: SharedMemoryIPC = SharedMemoryIPCFactory.getSharedMemory(envId, shmFileSize).apply {
        metricsHandler.init(this)
    }

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
            allMvaConflictCount = 0
            allAircraftConflictCountNoLoc = 0
            allAircraftConflictCountLoc = 0
            allWakeConflictCountNoLoc = 0
            allWakeConflictCountLoc = 0
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

            // writeState removes increased margin conflicts by default
            for (conflict in conflicts) {
                val ac1Loc = conflict.entity1.has(LocalizerCaptured.mapper)
                val ac2Loc = conflict.entity2?.has(LocalizerCaptured.mapper) ?: false

                if (conflict.entity2 != null) {
                    if (ac1Loc && ac2Loc) allAircraftConflictCountLoc++ else allAircraftConflictCountNoLoc++
                } else if (conflict.reason == Conflict.WAKE_INFRINGE) {
                    if (ac1Loc) allWakeConflictCountLoc++ else allWakeConflictCountNoLoc++
                } else if (conflict.reason in arrayOf(Conflict.MVA, Conflict.SID_STAR_MVA, Conflict.RESTRICTED)) {
                    allMvaConflictCount++
                }
            }

            terminating = isTerminating

            var actionOverrides: MutableMap<String, IntArray>? = null
            var shouldAddSnapshot = true

            if (evalMode && conflicts.notEmpty()) {
                // Only resolve conflicts that require the largest rollback stepsBack (10 = no-LOC, 30 = LOC)
                val (selectedConflicts, stepsBack) = getConflictsToResolve(conflicts)
                if (rlStateRestoreManager.snapshotCount() >= stepsBack) {
                    actionOverrides = resolveConflicts(selectedConflicts, gs, aircraft, stepsBack, stopServer)

                    // Reset if needsResetAfterStep received during conflict resolution (due to exceeding step limit)
                    if (actionOverrides == CONFLICT_RESOLUTION_RESET_RECEIVED) {
                        FileLog.warn("$envName PythonGymnasiumBridge", "Received needsResetAfterStep during conflict resolution - resetting next step")
                        resetNeeded = true
                        terminating = false
                        return
                    }

                    // resolveConflicts leaves the game in one of two states:
                    // - Failure: restored to the pre-resolution backup (state T right after the first writeState above).
                    // - Success: restoreSnapshot(stepsBack) applied; world is at T-stepsBack with overrides ready for performAction.
                    if (actionOverrides != null) {
                        // Success: do not push another snapshot — history already ends at the restored T-stepsBack frame.
                        shouldAddSnapshot = false

                        // Re-run writeState at T-stepsBack so rewards/metrics match that frame once (no double-count).
                        // stepCountModifier = -stepsBack offsets the training step counter for the rolled-back frames.
                        val (isTerminating2, _) = writeState(aircraft, stepCountModifier = (-stepsBack).byte)
                        terminating = isTerminating2
                    }
                    // Failure (actionOverrides == null): backup already restored inside resolveConflicts; skip this writeState
                    // to avoid applying writeState twice for the same logical time T.
                }
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

    /** Per-conflict plan: rollback depth and which modality to try (RESOLVE_BOTH = heading first, then IAS). */
    private data class ConflictMetadata(
        val conflict: Conflict, val stepsBack: Int, val resolutionAc1: String, val resolutionAc2: String?,
        val resolutionOption: Int
    ) {
        companion object {
            const val RESOLVE_HDG = 0
            const val RESOLVE_IAS = 1
            const val RESOLVE_BOTH = 2
        }
    }

    private fun getMetadataForConflict(conflict: Conflict): ConflictMetadata {
        val ac1 = conflict.entity1
        val callsign1 = ac1[AircraftInfo.mapper]?.icaoCallsign!!
        val ac2 = conflict.entity2
        val callsign2 = ac2?.get(AircraftInfo.mapper)?.icaoCallsign

        val ac1Loc = ac1.has(LocalizerCaptured.mapper)
        val ac2Loc = ac2?.has(LocalizerCaptured.mapper)
        val isMva = conflict.reason == Conflict.MVA || conflict.reason == Conflict.SID_STAR_MVA || conflict.reason == Conflict.RESTRICTED

        if (isMva) {
            return ConflictMetadata(
                conflict, CONFLICT_RESOLUTION_NO_LOC_LOOKBACK_STEPS, callsign1,
                null, ConflictMetadata.RESOLVE_HDG
            )
        } else if (ac1Loc == ac2Loc) {
            // Either both on LOC, or both are not — aircraft–aircraft only (not wake).
            // Deeper rollback when both on LOC; try HDG first then IAS if RESOLVE_BOTH.
            val steps = if (!ac1Loc) CONFLICT_RESOLUTION_NO_LOC_LOOKBACK_STEPS else CONFLICT_RESOLUTION_LOC_LOOKBACK_STEPS
            val resolveOption = if (!ac1Loc) ConflictMetadata.RESOLVE_HDG else ConflictMetadata.RESOLVE_BOTH
            return ConflictMetadata(
                conflict, steps, callsign1, callsign2, resolveOption
            )
        } else {
            // One of aircraft is on LOC, the other is not
            // Can be aircraft-aircraft conflict, or wake conflict (ac2 will be null if this is the case)
            val acToResolve = if (ac2 == null) ac1  // Wake conflict, resolve only ac1
            else if (ac1Loc) ac2 else ac1  // Aircraft-aircraft conflict, resolve aircraft not on LOC
            val acOnLoc = if (acToResolve == ac1) ac1Loc else ac2Loc!!
            val callsignToResolve = if (acToResolve == ac1) callsign1 else callsign2!!
            val steps = if (acOnLoc) CONFLICT_RESOLUTION_LOC_LOOKBACK_STEPS else CONFLICT_RESOLUTION_NO_LOC_LOOKBACK_STEPS
            val resolveOption = if (acOnLoc) ConflictMetadata.RESOLVE_BOTH else ConflictMetadata.RESOLVE_HDG
            return ConflictMetadata(
                conflict, steps, callsignToResolve, null, resolveOption
            )
        }
    }

    /** Keep only conflicts whose required [ConflictMetadata.stepsBack] equals the maximum among this step's conflicts. */
    private fun getConflictsToResolve(conflicts: GdxArray<Conflict>): Pair<GdxArray<ConflictMetadata>, Int> {
        var currMaxSteps = 0
        val filteredConflicts = GdxArray<ConflictMetadata>(conflicts.size)
        for (conflict in conflicts) {
            val metaData = getMetadataForConflict(conflict)
            if (metaData.stepsBack > currMaxSteps) {
                filteredConflicts.clear()
                currMaxSteps = metaData.stepsBack
            }
            if (metaData.stepsBack == currMaxSteps) {
                filteredConflicts.add(metaData)
            }
        }

        return Pair(filteredConflicts, currMaxSteps)
    }

    private fun resolveConflicts(selectedConflicts: GdxArray<ConflictMetadata>, gs: GameServer, aircraft: GdxArrayMap<String, Aircraft>, stepsBack: Int, stopServer: () -> Unit): MutableMap<String, IntArray>? {
        // Save backup of current state T
        val backupSnapshot = rlStateRestoreManager.getSnapshot(gs)
        val backupSpawned = spawnedInCurrentSession
        val backupLanded = landedInCurrentSession
        val backupReward = rewardHandler.getStateForSnapshot()
        val backupAdded = aircraftAdded
        val backupAgentCallsigns = Array(agentIdToAircraft.size) { agentIdToAircraft[it]?.get(AircraftInfo.mapper)?.icaoCallsign }
        val backupShm = sharedMemoryIPC.readBytes(0, shmFileSize)

//        FileLog.info(
//            "$envName PythonGymnasiumBridge",
//            "CR start: latestSnapshotT=${rlStateRestoreManager.getSnapshotAt(1)?.timestep}, lookbackSteps=$stepsBack, conflictCount=${sortedConflicts.size}"
//        )

        val currentOverrides = mutableMapOf<String, IntArray>()

        for (conflict in selectedConflicts) {
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
        conflict: ConflictMetadata,
        gs: GameServer,
        aircraft: GdxArrayMap<String, Aircraft>,
        stepsBack: Int,
        currentOverrides: MutableMap<String, IntArray>,
        stopServer: () -> Unit
    ): Boolean? {
        val snapshotTn = rlStateRestoreManager.getSnapshotAt(stepsBack) ?: return false
//        val baseTimestep = snapshotTn.timestep

        val resolutionOption = conflict.resolutionOption
        val resolutionSteps = when (resolutionOption) {
            ConflictMetadata.RESOLVE_HDG -> arrayOf(ConflictMetadata.RESOLVE_HDG)
            ConflictMetadata.RESOLVE_IAS -> arrayOf(ConflictMetadata.RESOLVE_IAS)
            ConflictMetadata.RESOLVE_BOTH -> arrayOf(ConflictMetadata.RESOLVE_HDG, ConflictMetadata.RESOLVE_IAS)
            else -> throw IllegalArgumentException("Unknown resolution option: $resolutionOption")
        }

        for (resolveAc in arrayOf(conflict.resolutionAc1, conflict.resolutionAc2)) {
            if (resolveAc == null) continue

            for (optionType in resolutionSteps) {
                val defaultClearance = when (optionType) {
                    ConflictMetadata.RESOLVE_HDG -> 2
                    ConflictMetadata.RESOLVE_IAS -> 2
                    else -> throw IllegalArgumentException("Unknown individual resolution option $optionType")
                }
                val actionIndex = when (optionType) {
                    ConflictMetadata.RESOLVE_HDG -> 0
                    ConflictMetadata.RESOLVE_IAS -> 2
                    else -> throw IllegalArgumentException("Unknown individual resolution option $optionType")
                }
                val origClearance = currentOverrides[resolveAc]?.get(actionIndex) ?: snapshotTn.actions[resolveAc]?.get(actionIndex) ?: defaultClearance
                val options = when (optionType) {
                    ConflictMetadata.RESOLVE_HDG -> getHeadingOptions(origClearance)
                    ConflictMetadata.RESOLVE_IAS -> getIasOptions(origClearance)
                    else -> throw IllegalArgumentException("Unknown individual resolution option $optionType")
                }

                val successAction = testOptions(resolveAc, options, optionType, gs, aircraft, stepsBack, currentOverrides, stopServer)
                if (successAction === TEST_OPTIONS_RESET_RECEIVED) return null
                if (successAction != null) {
//                FileLog.info(
//                    "$envName PythonGymnasiumBridge",
//                    "CR resolveSingleConflict end: baseSnapshotT=$baseTimestep, success=true, conflict=$conflict"
//                )
                    currentOverrides[resolveAc] = successAction
                    return true
                }
            }
        }

//        FileLog.info(
//            "$envName PythonGymnasiumBridge",
//            "CR resolveSingleConflict end: baseSnapshotT=$baseTimestep, success=false, conflict=$conflict"
//        )
        return false
    }

    private fun testOptions(
        testAc: String,
        options: IntArray,
        optionType: Int,
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

            for (step in 0 until stepsBack) {
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
                        ConflictMetadata.RESOLVE_HDG -> newAction[0] = opt
                        ConflictMetadata.RESOLVE_IAS -> newAction[2] = opt
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
                        ConflictMetadata.RESOLVE_HDG -> newAction[0] = opt
                        ConflictMetadata.RESOLVE_IAS -> newAction[2] = opt
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
            metricsHandler.logToSharedMemory(  // Spawn group for each agent
                MetricsHandler.AIRCRAFT_SPAWN_GROUP, aircraftAdded,
                ac[SpawnGroup.mapper]?.spawnGroup!!
            )
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

        val stateArray = ByteBuffer.allocate(MAX_RL_AIRCRAFT * sizePerAircraft).order(ByteOrder.nativeOrder())
        val acToRemove = GdxArray<Int>()
        for (currAgentID in 0 until agentIdToAircraft.size) {
            val currAircraft = agentIdToAircraft[currAgentID]

            if (currAircraft == null) {
                stateArray.position(stateArray.position() + sizePerAircraft - 7)
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
                var altMask = 31
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
        metricsHandler.logToSharedMemory(MetricsHandler.LANDING_RATE, landedInCurrentSession.toFloat() / AIRCRAFT_TO_SPAWN)
        metricsHandler.logToSharedMemory(MetricsHandler.CONFLICT_RATE_NO_LOC, rewardHandler.aircraftConflictCountNoLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        metricsHandler.logToSharedMemory(MetricsHandler.MVA_CONFLICT_RATE, rewardHandler.mvaConflictCount.toFloat() / AIRCRAFT_TO_SPAWN)
        metricsHandler.logToSharedMemory(MetricsHandler.WAKE_CONFLICT_RATE_NO_LOC, rewardHandler.wakeConflictCountNoLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        metricsHandler.logToSharedMemory(MetricsHandler.CONFLICT_RATE_LOC, rewardHandler.aircraftConflictCountLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        metricsHandler.logToSharedMemory(MetricsHandler.WAKE_CONFLICT_RATE_LOC, rewardHandler.wakeConflictCountLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        // Conflict rates before resolution
        metricsHandler.logToSharedMemory(MetricsHandler.CONFLICT_RATE_NO_LOC_BEFORE_RES, allAircraftConflictCountNoLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        metricsHandler.logToSharedMemory(MetricsHandler.CONFLICT_RATE_LOC_BEFORE_RES, allAircraftConflictCountLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        metricsHandler.logToSharedMemory(MetricsHandler.MVA_CONFLICT_RATE_BEFORE_RES, allMvaConflictCount.toFloat() / AIRCRAFT_TO_SPAWN)
        metricsHandler.logToSharedMemory(MetricsHandler.WAKE_CONFLICT_RATE_NO_LOC_BEFORE_RES, allWakeConflictCountNoLoc.toFloat() / AIRCRAFT_TO_SPAWN)
        metricsHandler.logToSharedMemory(MetricsHandler.WAKE_CONFLICT_RATE_LOC_BEFORE_RES, allWakeConflictCountLoc.toFloat() / AIRCRAFT_TO_SPAWN)

        // Copy all aircraft states
        sharedMemoryIPC.copyByteArray(constantSize + MAX_RL_AIRCRAFT * sizePerInstruction + additionalPadding, stateArray)

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

        val bytes = sharedMemoryIPC.readBytes(0, shmFileSize)
        val proceedFlag = bytes[0]
        if (proceedFlag.toInt() != 1) throw IllegalStateException("$envName ProceedFlag must be 1")
        // Reset action waiting flag
        sharedMemoryIPC.setByte(0, 0)
//        println("${System.currentTimeMillis()} Proceed unset")

//        println("Selected aircraft: $acIndex; aircraft count: ${aircraft.size}")

        val actionsRecorded = mutableMapOf<String, IntArray>()

        for (currAgentID in 0 until agentIdToAircraft.size) {
            val instructionStartOffset = constantSize + currAgentID * sizePerInstruction

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