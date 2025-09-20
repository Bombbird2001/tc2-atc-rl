package com.bombbird.terminalcontrol2.gymnasium.ipc

import com.sun.jna.Pointer
import com.sun.jna.platform.win32.Kernel32
import com.sun.jna.platform.win32.WinNT.HANDLE

class WindowsSharedMemory(envId: Int, fileSizeBytes: Int): SharedMemoryIPC {
    companion object {
        const val FILE_MAP_WRITE = 0x0002
        const val EVENT_ALL_ACCESS = 0x1F0003
        const val WAIT_TIMEOUT = 0x00000102
    }

    private val mmHandle = Kernel32.INSTANCE.OpenFileMapping(
        FILE_MAP_WRITE,
        false,
        "Local\\${SharedMemoryIPC.SHM_FILE_PREFIX}$envId"
    )
    private val buffer: Pointer

    // Open named events
    val resetSim: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\${SharedMemoryIPC.RESET_PREFIX}$envId")
    val actionReady: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\${SharedMemoryIPC.ACTION_READY_PREFIX}$envId")
    val actionDone: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\${SharedMemoryIPC.ACTION_DONE_PREFIX}$envId")
    val resetAfterStep: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\${SharedMemoryIPC.RESET_AFTER_STEP_PREFIX}$envId")

    private val envName = "[env$envId]"

    init {
        if (mmHandle == null) {
            throw NullPointerException("$envName Got WinError ${Kernel32.INSTANCE.GetLastError()} when opening shared memory")
        }
        buffer = Kernel32.INSTANCE.MapViewOfFile(
            mmHandle,
            FILE_MAP_WRITE,
            0, 0, fileSizeBytes
        )
    }

    override fun needsResetSim(): Boolean {
        return Kernel32.INSTANCE.WaitForSingleObject(resetSim, 0) != WAIT_TIMEOUT
    }

    override fun waitForActionDone(maxWaitTimeMs: Int): Boolean {
        return Kernel32.INSTANCE.WaitForSingleObject(actionDone,maxWaitTimeMs) != WAIT_TIMEOUT
    }

    override fun needsResetAfterStep(): Boolean {
        return Kernel32.INSTANCE.WaitForSingleObject(resetAfterStep, 0) != WAIT_TIMEOUT
    }

    override fun signalActionReady(): Boolean {
        return Kernel32.INSTANCE.SetEvent(actionReady)
    }

    override fun setByte(offset: Long, byte: Byte) {
        buffer.setByte(offset, byte)
    }

    override fun setFloat(offset: Long, float: Float) {
        buffer.setFloat(offset, float)
    }

    override fun setInt(offset: Long, int: Int) {
        buffer.setInt(offset, int)
    }

    override fun readBytes(offset: Long, bytes: Int): ByteArray {
        return buffer.getByteArray(offset, bytes)
    }
}