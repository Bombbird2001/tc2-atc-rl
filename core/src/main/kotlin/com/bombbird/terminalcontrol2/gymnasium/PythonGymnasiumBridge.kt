package com.bombbird.terminalcontrol2.gymnasium

import com.badlogic.ashley.utils.ImmutableArray
import com.badlogic.gdx.math.Vector2
import com.bombbird.terminalcontrol2.components.AircraftInfo
import com.bombbird.terminalcontrol2.components.Altitude
import com.bombbird.terminalcontrol2.components.ApproachChildren
import com.bombbird.terminalcontrol2.components.CommandTarget
import com.bombbird.terminalcontrol2.components.Direction
import com.bombbird.terminalcontrol2.components.GlideSlope
import com.bombbird.terminalcontrol2.components.GroundTrack
import com.bombbird.terminalcontrol2.components.Localizer
import com.bombbird.terminalcontrol2.components.LocalizerCaptured
import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.components.Speed
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.global.CONFLICT_PENALTY
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.LOC_CAP_REWARD
import com.bombbird.terminalcontrol2.global.MAG_HDG_DEV
import com.bombbird.terminalcontrol2.global.MAX_AIRCRAFT
import com.bombbird.terminalcontrol2.global.PER_STEP_PENALTY
import com.bombbird.terminalcontrol2.gymnasium.ipc.SharedMemoryIPC
import com.bombbird.terminalcontrol2.gymnasium.ipc.SharedMemoryIPCFactory
import com.bombbird.terminalcontrol2.navigation.Approach
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.utilities.FileLog
import com.bombbird.terminalcontrol2.utilities.addNewClearanceToPendingClearances
import com.bombbird.terminalcontrol2.utilities.byte
import com.bombbird.terminalcontrol2.utilities.convertWorldAndRenderDeg
import com.bombbird.terminalcontrol2.utilities.distPxFromPolygon
import com.bombbird.terminalcontrol2.utilities.findDeltaHeading
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import com.bombbird.terminalcontrol2.utilities.modulateHeading
import com.bombbird.terminalcontrol2.utilities.nmToPx
import com.bombbird.terminalcontrol2.utilities.pxpsToKt
import ktx.ashley.get
import ktx.ashley.has
import ktx.collections.GdxArrayMap
import ktx.collections.set
import ktx.collections.toGdxArray
import ktx.math.plusAssign
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs

class PythonGymnasiumBridge(envId: String): GymnasiumBridge {
    companion object {
        const val SIZE_PER_AIRCRAFT = 44
        const val SHM_FILE_SIZE = 12 + MAX_AIRCRAFT * SIZE_PER_AIRCRAFT

        const val FRAMES_PER_ACTION = 10 * 30

        const val HDG_ACTION_MULTIPLIER = 1
        const val ALT_ACTION_MULTIPLIER = 1000
        const val ALT_ACTION_ADDER = 2000
        const val SPD_ACTION_MULTIPLIER = 10
        const val SPD_ACTION_ADDER = 160

        const val LOOP_EXIT_MS = 15000

        fun distPxFromLoc(acPos: Position, approach: Approach): Float {
            val pos = approach.entity[Position.mapper]!!
            val loc = approach.entity[Localizer.mapper]!!
            val dir = approach.entity[Direction.mapper]!!.trackUnitVector
            val gs = approach.entity[GlideSlope.mapper]

            val offsetNm = gs?.offsetNm ?: 0f
            dir.setLength((nmToPx(1) - nmToPx(offsetNm)))
            val startPos = Vector2(pos.x, pos.y)
            startPos.plusAssign(dir)
            val endPos = Vector2(startPos)
            dir.setLength(nmToPx(loc.maxDistNm.toInt()))
            endPos.plusAssign(dir)

            return distPxFromPolygon(floatArrayOf(startPos.x, startPos.y, endPos.x, endPos.y), acPos.x, acPos.y)
        }
    }

    var framesToAction = FRAMES_PER_ACTION
    var trainerInitialized = false
    var loopExited = false
    var resetNeeded = false
    var terminating = false
    val acPrevLocDistPx: GdxArrayMap<String, Float> = GdxArrayMap()
    val acPrevAlt: GdxArrayMap<String, Float> = GdxArrayMap()
    val acOnLoc: HashSet<String> = HashSet()
    var targetApproach = lazy {
        GAME.gameServer?.airports?.get(0)?.entity?.get(ApproachChildren.mapper)?.approachMap?.get("ILS 02L")!!
    }
    var clearancesChangePenalty = 0f

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

            resetAircraft()
            writeState(aircraft)

            terminating = false
            sharedMemoryIPC.signalActionReady()
//            println("${System.currentTimeMillis()} Reset action ready")

            // Wait for action done event before continuing simulation
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
        if (aircraft.size > MAX_AIRCRAFT) {
            throw IllegalArgumentException("$envName Aircraft must have <= $MAX_AIRCRAFT items, got ${aircraft.size} instead")
        }

        // Proceed flag
        sharedMemoryIPC.setByte(0, 1)

        var reward = 0f
        var shouldTerminate = 0.byte

        // Conflict check
//        val conflicts = conflictManager.getConflictCountRL(ImmutableArray(aircraft.values().map { it.entity }.toGdxArray()))
//        if (conflicts > 0) {
//            reward -= CONFLICT_PENALTY
//            shouldTerminate = 1
//        }

        // Clearance change penalty from previous clearance
        reward -= clearancesChangePenalty
        clearancesChangePenalty = 0f

        // Simulation terminated (conflict encountered)
        sharedMemoryIPC.setByte(1, shouldTerminate)

        // Aircraft x, y, alt, gs, track
        val stateArray = ByteBuffer.allocate(MAX_AIRCRAFT * SIZE_PER_AIRCRAFT).order(ByteOrder.nativeOrder())
        var totalAcReward = 0f
        var relevantAircraftCount = 0
        for (i in 0 until aircraft.size) {
            // Only loop to aircraft size; unused slots default to 0 already during allocate which sets "present" byte to 0
            val currAircraft = aircraft.getValueAt(i).entity
            val currAcInfo = currAircraft[AircraftInfo.mapper]!!
            val currPos = currAircraft[Position.mapper]!!
            val currAlt = currAircraft[Altitude.mapper]!!
            val currSpd = currAircraft[Speed.mapper]!!
            val currGroundTrack = currAircraft[GroundTrack.mapper]!!
            val currHdg = modulateHeading(convertWorldAndRenderDeg(currGroundTrack.trackVectorPxps.angleDeg()))
            val currPrevClearance = getLatestClearanceState(currAircraft)!!
            val currLocCap = if (currAircraft.has(LocalizerCaptured.mapper)) 1.byte else 0.byte

            stateArray.putFloat(currPos.x)
            stateArray.putFloat(currPos.y)
            stateArray.putFloat(currAlt.altitudeFt)
            stateArray.putFloat(currGroundTrack.trackVectorPxps.len().let { pxpsToKt(it) })
            stateArray.putFloat(currHdg)
            stateArray.putFloat(currSpd.angularSpdDps)
            stateArray.putFloat(currSpd.vertSpdFpm)
            stateArray.putInt(currPrevClearance.clearedAlt)
            stateArray.putInt(currPrevClearance.vectorHdg?.toInt() ?: currHdg.toInt())
            stateArray.putInt(currPrevClearance.clearedIas.toInt())
            stateArray.put(currLocCap)
            stateArray.put(1) // "Aircraft exists" flag - for token masking in model
            stateArray.position(stateArray.position() + 2) // 2 padding bytes

            if (currLocCap == 1.byte) {
                // If aircraft has previously captured LOC, ignore its rewards
                if (acOnLoc.contains(currAcInfo.icaoCallsign)) {
                    continue
                }

                // Lump sum reward on LOC capture TODO depending on intercept angle (lower angle = higher reward)?
                totalAcReward += LOC_CAP_REWARD
                acOnLoc.add(currAcInfo.icaoCallsign)
                acPrevLocDistPx.removeKey(currAcInfo.icaoCallsign)
                acPrevAlt.removeKey(currAcInfo.icaoCallsign)
            } else {
                acOnLoc.remove(currAcInfo.icaoCallsign)
            }

            relevantAircraftCount++

            // Reward from previous action
            // Constant -0.03 per time step + decrease in distance towards LOC line segment
            // + decrease in altitude
            val newLocDistPx = distPxFromLoc(currPos, targetApproach.value)
            if (acPrevLocDistPx.containsKey(currAcInfo.icaoCallsign)) {
                val distReward = (acPrevLocDistPx[currAcInfo.icaoCallsign] - newLocDistPx) / 1600
                val altReward = (acPrevAlt[currAcInfo.icaoCallsign] - currAlt.altitudeFt) / 12000
                totalAcReward += distReward + altReward - PER_STEP_PENALTY
            }

            acPrevLocDistPx[currAcInfo.icaoCallsign] = newLocDistPx
            acPrevAlt[currAcInfo.icaoCallsign] = currAlt.altitudeFt

            // Discourage aircraft from loitering too long close to LOC
//            if (newLocDistPx < nmToPx(4) && currAlt.altitudeFt <= 6010) totalAcReward -= 0.06f
        }
        sharedMemoryIPC.copyByteArray(12, stateArray)

        reward += (if (relevantAircraftCount > 0) totalAcReward / relevantAircraftCount else 0f)

        sharedMemoryIPC.setFloat(8, reward)

        return shouldTerminate == 1.byte
    }

    private fun performAction(aircraft: GdxArrayMap<String, Aircraft>) {
        if (aircraft.size > MAX_AIRCRAFT) {
            throw IllegalArgumentException("$envName Aircraft must have <= $MAX_AIRCRAFT items, got ${aircraft.size} instead")
        }

//        val targetAircraft = aircraft.getValueAt(0).entity

        val bytes = sharedMemoryIPC.readBytes(0, 8)
        val proceedFlag = bytes[0]
        if (proceedFlag.toInt() != 1) throw IllegalStateException("$envName ProceedFlag must be 1")
        sharedMemoryIPC.setByte(0, 0) // Reset proceed flag
//        println("${System.currentTimeMillis()} Proceed unset")

        val issueInstruction = bytes[3] == 1.byte

        if (issueInstruction) {
            val acIndex = bytes[2].toInt()
            if (acIndex < aircraft.size) {
                val targetAircraft = aircraft.getValueAt(acIndex).entity
                val clearedHdg = (sharedMemoryIPC.readShort(4) * HDG_ACTION_MULTIPLIER).toShort()
                val clearedAlt = bytes[6] * ALT_ACTION_MULTIPLIER + ALT_ACTION_ADDER
                val clearedIas = (bytes[7] * SPD_ACTION_MULTIPLIER + SPD_ACTION_ADDER).toShort()

                val currHdg = convertWorldAndRenderDeg(targetAircraft[Direction.mapper]!!.trackUnitVector.angleDeg()) + MAG_HDG_DEV
//        val currAlt = targetAircraft[Altitude.mapper]!!.altitudeFt

                val prevClearance = getLatestClearanceState(targetAircraft)!!
                clearancesChangePenalty = (
                        0.025f +
//                    ((prevClearance.vectorHdg?.let {
//                        (findDeltaHeading(it.toFloat(), clearedHdg.toFloat(), CommandTarget.TURN_DEFAULT) > 2).toInt() * 0.15f
//                    }) ?: 0f) +
                                (clearedHdg != prevClearance.vectorHdg && abs(findDeltaHeading(currHdg, clearedHdg.toFloat(), CommandTarget.TURN_DEFAULT)) > 2).toInt() * 0.025f +
                                (clearedAlt != prevClearance.clearedAlt).toInt() * 0.025f +
                                (clearedIas != prevClearance.clearedIas).toInt() * 0.025f
                        )
                val clearanceState = prevClearance.copy(vectorHdg = clearedHdg, clearedAlt = clearedAlt, clearedIas = clearedIas)
                addNewClearanceToPendingClearances(targetAircraft, clearanceState, 0)
            } else {
                clearancesChangePenalty = 0.05f
            }
        } else {
            clearancesChangePenalty = 0f
        }
    }
}

fun Boolean.toInt() = if (this) 1 else 0