package com.bombbird.terminalcontrol2.gymnasium

import com.badlogic.ashley.core.Entity
import com.badlogic.ashley.utils.ImmutableArray
import com.badlogic.gdx.math.Vector2
import com.bombbird.terminalcontrol2.components.Altitude
import com.bombbird.terminalcontrol2.components.ApproachChildren
import com.bombbird.terminalcontrol2.components.Direction
import com.bombbird.terminalcontrol2.components.GlideSlope
import com.bombbird.terminalcontrol2.components.GroundTrack
import com.bombbird.terminalcontrol2.components.Localizer
import com.bombbird.terminalcontrol2.components.LocalizerCaptured
import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.components.Speed
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.LOC_CAP_CHECK
import com.bombbird.terminalcontrol2.gymnasium.ipc.SharedMemoryIPC
import com.bombbird.terminalcontrol2.gymnasium.ipc.SharedMemoryIPCFactory
import com.bombbird.terminalcontrol2.navigation.Approach
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.utilities.FileLog
import com.bombbird.terminalcontrol2.utilities.addNewClearanceToPendingClearances
import com.bombbird.terminalcontrol2.utilities.byte
import com.bombbird.terminalcontrol2.utilities.convertWorldAndRenderDeg
import com.bombbird.terminalcontrol2.utilities.distPxFromPolygon
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import com.bombbird.terminalcontrol2.utilities.modulateHeading
import com.bombbird.terminalcontrol2.utilities.nmToPx
import com.bombbird.terminalcontrol2.utilities.pxpsToKt
import ktx.ashley.get
import ktx.ashley.has
import ktx.collections.GdxArray
import ktx.collections.GdxArrayMap
import ktx.math.plusAssign

class PythonGymnasiumBridge(envId: Int): GymnasiumBridge {
    companion object {
        const val SHM_FILE_SIZE = 52

        const val FRAMES_PER_ACTION = 10 * 30

        const val HDG_ACTION_MULTIPLIER = 5
        const val ALT_ACTION_MULTIPLIER = 1000
        const val ALT_ACTION_ADDER = 2000
        const val SPD_ACTION_MULTIPLIER = 10
        const val SPD_ACTION_ADDER = 160

        const val LOOP_EXIT_MS = 15000
    }

    var framesToAction = FRAMES_PER_ACTION
    var loopExited = false
    var resetNeeded = false
    var terminating = false
    var prevLocDistPx = Float.MIN_VALUE
    var prevAltFt = 20000f
    var targetApproach = lazy {
        GAME.gameServer?.airports?.get(0)?.entity?.get(ApproachChildren.mapper)?.approachMap?.get("ILS 02L")!!
    }
    var clearancesChanged = 0

    val conflictManager = ConflictManager()

    private val sharedMemoryIPC: SharedMemoryIPC = SharedMemoryIPCFactory.getSharedMemory(envId, SHM_FILE_SIZE)
    private val envName = "[env$envId]"

    override fun update(aircraft: GdxArrayMap<String, Aircraft>, resetAircraft: () -> GdxArrayMap<String, Aircraft>) {
        if (loopExited) return

        // Check for reset sim event
        if (sharedMemoryIPC.needsResetSim()) {
//            FileLog.info("$envName PythonGymnasiumBridge", "Resetting state")
            resetNeeded = false
            terminating = false
            val newAircraft = resetAircraft()
            writeState(newAircraft)
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
            writeState(aircraft)

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
    }

    private fun distPxFromLoc(aircraft: Entity, approach: Approach): Float {
        val acPos = aircraft[Position.mapper]!!
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

    private fun writeState(aircraft: GdxArrayMap<String, Aircraft>) {
        if (aircraft.size != 1) {
            throw IllegalArgumentException("$envName Aircraft must have exactly 1 item, got ${aircraft.size} instead")
        }

        val targetAircraft = aircraft.getValueAt(0).entity

        // Proceed flag
        sharedMemoryIPC.setByte(0, 1)

        // Reward from previous action
        // Constant -0.01 per time step + decrease in distance towards LOC line segment +
        // lump sum reward on LOC capture TODO depending on intercept angle (lower angle = higher reward)?
        val newLocDistPx = distPxFromLoc(targetAircraft, targetApproach.value)
        val pos = targetAircraft[Position.mapper]!!
        val alt = targetAircraft[Altitude.mapper]!!
        val spd = targetAircraft[Speed.mapper]!!
        val prevClearance = getLatestClearanceState(targetAircraft)!!
        var reward = (prevLocDistPx - newLocDistPx) / 400 + (prevAltFt - alt.altitudeFt) / 12000 - 0.01f + (clearancesChanged * -0.05f)
        // Discourage aircraft from loitering too long close to LOC
        if (newLocDistPx < nmToPx(4) && alt.altitudeFt <= 6010) reward -= 0.02f
        clearancesChanged = 0
        prevLocDistPx = newLocDistPx
        prevAltFt = alt.altitudeFt
        val isLocCap: Byte = when {
            LOC_CAP_CHECK -> if (targetAircraft.has(LocalizerCaptured.mapper)) 1 else 0
            else -> if (newLocDistPx < 15) 1 else 0
        }
        var shouldTerminate: Byte = 0
        if (isLocCap == 1.byte) {
            reward += 4
            terminating = true
            shouldTerminate = 1
        }
        val conflicts = conflictManager.getConflictCountRL(ImmutableArray(GdxArray(arrayOf(targetAircraft))))
        if (conflicts > 0) {
            reward -= 5
            terminating = true
            shouldTerminate = 1
        }
        sharedMemoryIPC.setFloat(4, reward)

        // Simulation terminated (LOC captured / conflict encountered)
        sharedMemoryIPC.setByte(8, shouldTerminate)

        // Aircraft x, y, alt, gs, track
        val groundTrack = targetAircraft[GroundTrack.mapper]!!
        val currHdg = modulateHeading(convertWorldAndRenderDeg(groundTrack.trackVectorPxps.angleDeg()))
        sharedMemoryIPC.setFloat(12, pos.x)
        sharedMemoryIPC.setFloat(16, pos.y)
        sharedMemoryIPC.setFloat(20, alt.altitudeFt)
        sharedMemoryIPC.setFloat(24, groundTrack.trackVectorPxps.len().let { pxpsToKt(it) })
        sharedMemoryIPC.setFloat(28, currHdg)
        sharedMemoryIPC.setFloat(32, spd.angularSpdDps)
        sharedMemoryIPC.setFloat(36, spd.vertSpdFpm)
        sharedMemoryIPC.setInt(40, prevClearance.clearedAlt)
        sharedMemoryIPC.setInt(44, prevClearance.vectorHdg?.toInt() ?: currHdg.toInt())
        sharedMemoryIPC.setInt(48, prevClearance.clearedIas.toInt())
    }

    private fun performAction(aircraft: GdxArrayMap<String, Aircraft>) {
        if (aircraft.size != 1) {
            throw IllegalArgumentException("$envName Aircraft must have exactly 1 item, got ${aircraft.size} instead")
        }

        val targetAircraft = aircraft.getValueAt(0).entity

        val bytes = sharedMemoryIPC.readBytes(0, 4)
        val proceedFlag = bytes[0]
        if (proceedFlag.toInt() != 1) throw IllegalStateException("$envName ProceedFlag must be 1")
        sharedMemoryIPC.setByte(0, 0) // Reset proceed flag
//        println("${System.currentTimeMillis()} Proceed unset")

        val clearedHdg = (bytes[1] * HDG_ACTION_MULTIPLIER).toShort()
        val clearedAlt = bytes[2] * ALT_ACTION_MULTIPLIER + ALT_ACTION_ADDER
        val clearedIas = (bytes[3] * SPD_ACTION_MULTIPLIER + SPD_ACTION_ADDER).toShort()

        val prevClearance = getLatestClearanceState(targetAircraft)!!
        clearancesChanged = (clearedHdg != prevClearance.vectorHdg).toInt() + (clearedAlt != prevClearance.clearedAlt).toInt() + (clearedIas != prevClearance.clearedIas).toInt()
        val clearanceState = prevClearance.copy(vectorHdg = clearedHdg, clearedAlt = clearedAlt, clearedIas = clearedIas)
        addNewClearanceToPendingClearances(targetAircraft, clearanceState, 0)
    }
}

fun Boolean.toInt() = if (this) 1 else 0