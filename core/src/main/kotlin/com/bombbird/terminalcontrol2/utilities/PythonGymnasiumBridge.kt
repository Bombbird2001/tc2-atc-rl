package com.bombbird.terminalcontrol2.utilities

import com.bombbird.terminalcontrol2.components.RadarData
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.sun.jna.Pointer
import com.sun.jna.platform.win32.Kernel32
import com.sun.jna.platform.win32.WinNT.HANDLE
import ktx.ashley.get
import ktx.collections.GdxArrayMap

object PythonGymnasiumBridge {
    const val FILE_SIZE = 28
    const val FILE_MAP_WRITE = 0x0002
    const val EVENT_ALL_ACCESS = 0x1F0003
    const val WAIT_TIMEOUT = 0x00000102

    const val FRAMES_PER_ACTION = 10 * 30

    const val HDG_ACTION_MULTIPLIER = 5
    const val ALT_ACTION_MULTIPLIER = 1000
    const val ALT_ACTION_ADDER = 2000
    const val SPD_ACTION_MULTIPLIER = 10
    const val SPD_ACTION_ADDER = 160

    var framesToAction = FRAMES_PER_ACTION

    private val mmHandle = Kernel32.INSTANCE.OpenFileMapping(
        FILE_MAP_WRITE,
        false,
        "Local\\ATCRLSharedMem"
    )
    private val buffer: Pointer

    // Open named events
    val resetSim: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\ATCRLResetEvent")
    val actionReady: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\ATCRLActionReadyEvent")
    val actionDone: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\ATCRLActionDoneEvent")

    init {
        if (mmHandle == null) {
            FileLog.warn("SharedMemory", "Got WinError ${Kernel32.INSTANCE.GetLastError()} when opening shared memory")
        }
        buffer = Kernel32.INSTANCE.MapViewOfFile(
            mmHandle,
            FILE_MAP_WRITE,
            0, 0, FILE_SIZE
        )
    }

    fun update(aircraft: GdxArrayMap<String, Aircraft>, resetAircraft: () -> GdxArrayMap<String, Aircraft>) {
        // Check for reset sim event
        val returnCode = Kernel32.INSTANCE.WaitForSingleObject(resetSim, 0)
        if (returnCode != WAIT_TIMEOUT) {
            val newAircraft = resetAircraft()
            writeState(newAircraft)
            Kernel32.INSTANCE.SetEvent(actionReady)
            framesToAction = FRAMES_PER_ACTION
            return
        }

        framesToAction--
        if (framesToAction <= 0) {
            writeState(aircraft)

            // Send action ready event after writing state to shared memory
            Kernel32.INSTANCE.SetEvent(actionReady)

            // Wait for action done event before continuing simulation
            Kernel32.INSTANCE.WaitForSingleObject(actionDone, Kernel32.INFINITE)
            performAction(aircraft)

            framesToAction = FRAMES_PER_ACTION
        }
    }

    private fun writeState(aircraft: GdxArrayMap<String, Aircraft>) {
        if (aircraft.size != 1) {
            throw IllegalArgumentException("Aircraft must have exactly 1 item, got ${aircraft.size} instead")
        }

        val targetAircraft = aircraft.getValueAt(0).entity

        // Proceed flag
        buffer.setByte(0, 1)

        // TODO Reward from previous action
        // Constant -1 per time step + decrease in distance towards LOC line segment +
        // lump sum reward on LOC capture depending on intercept angle (lower angle = higher reward)
        buffer.setByte(4, -1)

        // TODO Simulation terminated (LOC captured)
        buffer.setByte(5, 0)

        // Aircraft x, y, alt, gs, track
        val radarData = targetAircraft[RadarData.mapper]!!
        buffer.setFloat(8, radarData.position.x)
        buffer.setFloat(12, radarData.position.y)
        buffer.setFloat(16, radarData.altitude.altitudeFt)
        buffer.setFloat(20, radarData.speed.speedKts)
        buffer.setFloat(24, convertWorldAndRenderDeg(radarData.direction.trackUnitVector.angleDeg()))
    }

    private fun performAction(aircraft: GdxArrayMap<String, Aircraft>) {
        if (aircraft.size != 1) {
            throw IllegalArgumentException("Aircraft must have exactly 1 item, got ${aircraft.size} instead")
        }

        val targetAircraft = aircraft.getValueAt(0).entity

        val proceedFlag = buffer.getByte(0)
        if (proceedFlag.toInt() != 1) throw IllegalStateException("ProceedFlag must be 1")
        buffer.setByte(0, 0) // Reset proceed flag

        val clearedHdg = (buffer.getByte(1) * HDG_ACTION_MULTIPLIER).toShort()
        val clearedAlt = buffer.getByte(2) * ALT_ACTION_MULTIPLIER + ALT_ACTION_ADDER
        val clearedIas = (buffer.getByte(3) * SPD_ACTION_MULTIPLIER + SPD_ACTION_ADDER).toShort()

        val clearanceState = getLatestClearanceState(targetAircraft)!!
        clearanceState.vectorHdg = clearedHdg
        clearanceState.clearedAlt = clearedAlt
        clearanceState.clearedIas = clearedIas
        addNewClearanceToPendingClearances(targetAircraft, clearanceState, 0)
    }
}