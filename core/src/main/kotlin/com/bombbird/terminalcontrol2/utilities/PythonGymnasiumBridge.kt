package com.bombbird.terminalcontrol2.utilities

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
import com.bombbird.terminalcontrol2.navigation.Approach
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.sun.jna.Pointer
import com.sun.jna.platform.win32.Kernel32
import com.sun.jna.platform.win32.WinNT.HANDLE
import ktx.ashley.get
import ktx.ashley.has
import ktx.collections.GdxArray
import ktx.collections.GdxArrayMap
import ktx.math.plusAssign

class PythonGymnasiumBridge(envId: Int) {
    companion object {
        const val FILE_SIZE = 52
        const val FILE_MAP_WRITE = 0x0002
        const val EVENT_ALL_ACCESS = 0x1F0003
        const val WAIT_TIMEOUT = 0x00000102

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

    private val mmHandle = Kernel32.INSTANCE.OpenFileMapping(
        FILE_MAP_WRITE,
        false,
        "Local\\ATCRLSharedMem$envId"
    )
    private val buffer: Pointer

    // Open named events
    val resetSim: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\ATCRLResetEvent$envId")
    val actionReady: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\ATCRLActionReadyEvent$envId")
    val actionDone: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\ATCRLActionDoneEvent$envId")
    val resetAfterStep: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\ATCRLResetAfterEvent$envId")

    init {
        if (mmHandle == null) {
            throw NullPointerException("Got WinError ${Kernel32.INSTANCE.GetLastError()} when opening shared memory")
        }
        buffer = Kernel32.INSTANCE.MapViewOfFile(
            mmHandle,
            FILE_MAP_WRITE,
            0, 0, FILE_SIZE
        )
    }

    fun update(aircraft: GdxArrayMap<String, Aircraft>, resetAircraft: () -> GdxArrayMap<String, Aircraft>) {
        if (loopExited) return

        // Check for reset sim event
        val returnCode = Kernel32.INSTANCE.WaitForSingleObject(resetSim, 0)
        if (returnCode != WAIT_TIMEOUT) {
//            println("Resetting state")
            resetNeeded = false
            terminating = false
            val newAircraft = resetAircraft()
            writeState(newAircraft)
            Kernel32.INSTANCE.SetEvent(actionReady)
//            println("${System.currentTimeMillis()} Reset action ready")

            // Wait for action done event before continuing simulation
            val returnCode = Kernel32.INSTANCE.WaitForSingleObject(actionDone,LOOP_EXIT_MS)
            if (returnCode == WAIT_TIMEOUT) {
                // Assume RL program has exited, stop bridge loop
                FileLog.warn("PythonGymnasiumBridge", "Update loop exited")
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
            Kernel32.INSTANCE.SetEvent(actionReady)
//            println("${System.currentTimeMillis()} Set action ready")

            val resetAfterStepCode = Kernel32.INSTANCE.WaitForSingleObject(resetAfterStep, 0)
            if (resetAfterStepCode != WAIT_TIMEOUT || terminating) {
                // Reset requested, exit update so won't get blocked
                resetNeeded = true
                terminating = false
                return
            }

            // Wait for action done event before continuing simulation
            val returnCode = Kernel32.INSTANCE.WaitForSingleObject(actionDone,LOOP_EXIT_MS)
            if (returnCode == WAIT_TIMEOUT) {
                // Assume RL program has exited, stop bridge loop
                FileLog.warn("PythonGymnasiumBridge", "Update loop exited")
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
            throw IllegalArgumentException("Aircraft must have exactly 1 item, got ${aircraft.size} instead")
        }

        val targetAircraft = aircraft.getValueAt(0).entity

        // Proceed flag
        buffer.setByte(0, 1)

        // Reward from previous action
        // Constant -0.5 per time step + decrease in distance towards LOC line segment +
        // lump sum reward on LOC capture TODO depending on intercept angle (lower angle = higher reward)?
        val newLocDistPx = distPxFromLoc(targetAircraft, targetApproach.value)
        val pos = targetAircraft[Position.mapper]!!
        val alt = targetAircraft[Altitude.mapper]!!
        val spd = targetAircraft[Speed.mapper]!!
        val prevClearance = getLatestClearanceState(targetAircraft)!!
        var reward = (prevLocDistPx - newLocDistPx) / 400 + (prevAltFt - alt.altitudeFt) / 12000 - 0.01f + (clearancesChanged * -0.3f)
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
        buffer.setFloat(4, reward)

        // Simulation terminated (LOC captured / conflict encountered)
        buffer.setByte(8, shouldTerminate)

        // Aircraft x, y, alt, gs, track
        val groundTrack = targetAircraft[GroundTrack.mapper]!!
        val currHdg = modulateHeading(convertWorldAndRenderDeg(groundTrack.trackVectorPxps.angleDeg()))
        buffer.setFloat(12, pos.x)
        buffer.setFloat(16, pos.y)
        buffer.setFloat(20, alt.altitudeFt)
        buffer.setFloat(24, groundTrack.trackVectorPxps.len().let { pxpsToKt(it) })
        buffer.setFloat(28, currHdg)
        buffer.setFloat(32, spd.angularSpdDps)
        buffer.setFloat(36, spd.vertSpdFpm)
        buffer.setInt(40, prevClearance.clearedAlt)
        buffer.setInt(44, prevClearance.vectorHdg?.toInt() ?: currHdg.toInt())
        buffer.setInt(48, prevClearance.clearedIas.toInt())
    }

    private fun performAction(aircraft: GdxArrayMap<String, Aircraft>) {
        if (aircraft.size != 1) {
            throw IllegalArgumentException("Aircraft must have exactly 1 item, got ${aircraft.size} instead")
        }

        val targetAircraft = aircraft.getValueAt(0).entity

        val bytes = buffer.getByteArray(0, 4)
        val proceedFlag = bytes[0]
        if (proceedFlag.toInt() != 1) throw IllegalStateException("ProceedFlag must be 1")
        buffer.setByte(0, 0) // Reset proceed flag
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