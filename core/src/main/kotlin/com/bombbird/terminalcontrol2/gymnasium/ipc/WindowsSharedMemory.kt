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

    private val buffer: Pointer

    // Open named events
    val resetSim: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\${SharedMemoryIPC.RESET_PREFIX}$envId")
    val actionReady: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\${SharedMemoryIPC.ACTION_READY_PREFIX}$envId")
    val actionDone: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\${SharedMemoryIPC.ACTION_DONE_PREFIX}$envId")
    val resetAfterStep: HANDLE = Kernel32.INSTANCE.OpenEvent(EVENT_ALL_ACCESS, false, "Local\\${SharedMemoryIPC.RESET_AFTER_STEP_PREFIX}$envId")

    private val envName = "[env$envId]"

    init {
        val mmHandle = Kernel32.INSTANCE.OpenFileMapping(
            FILE_MAP_WRITE,
            false,
            "Local\\${SharedMemoryIPC.SHM_FILE_PREFIX}$envId"
        )
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

    override fun setByte(offset: Int, byte: Byte) {
        buffer.setByte(offset.toLong(), byte)
    }

    override fun setFloat(offset: Int, float: Float) {
        buffer.setFloat(offset.toLong(), float)
    }

    override fun setInt(offset: Int, int: Int) {
        buffer.setInt(offset.toLong(), int)
    }

    override fun readBytes(offset: Int, bytes: Int): ByteArray {
        return buffer.getByteArray(offset.toLong(), bytes)
    }
}