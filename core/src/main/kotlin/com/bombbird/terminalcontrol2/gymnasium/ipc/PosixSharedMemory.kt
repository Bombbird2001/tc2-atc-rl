package com.bombbird.terminalcontrol2.gymnasium.ipc

import com.sun.jna.Native
import com.sun.jna.Pointer
import com.sun.jna.platform.linux.ErrNo
import java.nio.ByteBuffer

class PosixSharedMemory(envId: Int, fileSizeBytes: Long): SharedMemoryIPC {
    companion object {
        const val O_RDWR = 0x0002
        const val PROT_READ = 0x1
        const val PROT_WRITE = 0x2
        const val MAP_SHARED = 0x01
    }

    private val buffer: ByteBuffer

    val resetSim = PosixLibC.sem_open("${SharedMemoryIPC.RESET_PREFIX}$envId", O_RDWR)
    val actionReady = PosixLibC.sem_open("${SharedMemoryIPC.ACTION_READY_PREFIX}$envId", O_RDWR)
    val actionDone = PosixLibC.sem_open("${SharedMemoryIPC.ACTION_DONE_PREFIX}$envId", O_RDWR)
    val resetAfterStep = PosixLibC.sem_open("${SharedMemoryIPC.RESET_AFTER_STEP_PREFIX}$envId", O_RDWR)

    init {
        val fd = PosixLibC.shm_open("${SharedMemoryIPC.SHM_FILE_PREFIX}$envId", O_RDWR, "666".toInt(8))
        if (fd < 0) throw NullPointerException("Unable to read shared memory file")

        val ptr = PosixLibC.mmap(null, fileSizeBytes, PROT_READ or PROT_WRITE, MAP_SHARED, fd, 0)
        if (Pointer.nativeValue(ptr) == -1L) throw NullPointerException("mmap failed")

        buffer = ptr.getByteBuffer(0, fileSizeBytes)
    }

    override fun needsResetSim(): Boolean {
        return PosixLibC.sem_trywait(resetSim) == 0
    }

    override fun waitForActionDone(maxWaitTimeMs: Int): Boolean {
        val timeout = Timespec((System.currentTimeMillis() + maxWaitTimeMs) / 1000, 0)
        val res = PosixLibC.sem_timedwait(actionDone, timeout)
        if (res == -1 && Native.getLastError() == ErrNo.EINTR) {
            // Try again, interrupted
            return waitForActionDone(maxWaitTimeMs)
        }
        return res == 0
    }

    override fun needsResetAfterStep(): Boolean {
        return PosixLibC.sem_trywait(resetAfterStep) == 0
    }

    override fun signalActionReady(): Boolean {
        return PosixLibC.sem_post(actionReady) == 0
    }

    override fun setByte(offset: Int, byte: Byte) {
        buffer.put(offset, byte)
    }

    override fun setFloat(offset: Int, float: Float) {
        buffer.putFloat(offset, float)
    }

    override fun setInt(offset: Int, int: Int) {
        buffer.putInt(offset, int)
    }

    override fun readBytes(offset: Int, bytes: Int): ByteArray {
        val result = ByteArray(bytes)
        buffer.position(offset)
        buffer.get(result)
        return result
    }
}