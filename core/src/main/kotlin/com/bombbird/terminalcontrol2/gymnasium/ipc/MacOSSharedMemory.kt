package com.bombbird.terminalcontrol2.gymnasium.ipc

import com.sun.jna.Native
import com.sun.jna.Pointer
import com.sun.jna.platform.linux.ErrNo
import java.nio.ByteBuffer

class MacOSSharedMemory(envId: String, private val fileSizeBytes: Long): SharedMemoryIPC {
    companion object {
        const val O_RDWR = 0x0002
        const val PROT_READ = 0x1
        const val PROT_WRITE = 0x2
        const val MAP_SHARED = 0x01
    }

    private val buffer: ByteBuffer

    val trainerInitializedName = "${SharedMemoryIPC.TRAINER_INITIALIZED}$envId"
    val resetSimName = "${SharedMemoryIPC.RESET_PREFIX}$envId"
    val actionReadyName = "${SharedMemoryIPC.ACTION_READY_PREFIX}$envId"
    val actionDoneName = "${SharedMemoryIPC.ACTION_DONE_PREFIX}$envId"
    val resetAfterStepName = "${SharedMemoryIPC.RESET_AFTER_STEP_PREFIX}$envId"

    val trainerInitialized = MacOSLibC.sem_open(trainerInitializedName, O_RDWR)
    val resetSim = MacOSLibC.sem_open(resetSimName, O_RDWR)
    val actionReady = MacOSLibC.sem_open(actionReadyName, O_RDWR)
    val actionDone = MacOSLibC.sem_open(actionDoneName, O_RDWR)
    val resetAfterStep = MacOSLibC.sem_open(resetAfterStepName, O_RDWR)

    private val fd: Int = MacOSLibC.shm_open("${SharedMemoryIPC.SHM_FILE_PREFIX}$envId", O_RDWR, "666".toInt(8))
    private val ptr: Pointer

    init {
        MacOSLibC.sem_unlink(trainerInitializedName)
        MacOSLibC.sem_unlink(resetSimName)
        MacOSLibC.sem_unlink(actionDoneName)
        MacOSLibC.sem_unlink(actionReadyName)
        MacOSLibC.sem_unlink(resetAfterStepName)

        if (fd < 0) throw NullPointerException("Unable to read shared memory file")

        ptr = MacOSLibC.mmap(null, fileSizeBytes, PROT_READ or PROT_WRITE, MAP_SHARED, fd, 0)
        if (Pointer.nativeValue(ptr) == -1L) throw NullPointerException("mmap failed")
        MacOSLibC.close(fd)

        buffer = ptr.getByteBuffer(0, fileSizeBytes)
    }

    override fun waitForTrainerInitialized() {
        var res = -1
        while (res != 0) {
            res = MacOSLibC.sem_wait(trainerInitialized)
        }
    }

    override fun needsResetSim(): Boolean {
        return MacOSLibC.sem_trywait(resetSim) == 0
    }

    override fun waitForActionDone(maxWaitTimeMs: Int): Boolean {
        val res = MacOSLibC.sem_wait(actionDone)
        if (res == -1 && Native.getLastError() == ErrNo.EINTR) {
            // Try again, interrupted
            return waitForActionDone(maxWaitTimeMs)
        }
        return res == 0
    }

    override fun needsResetAfterStep(): Boolean {
        return MacOSLibC.sem_trywait(resetAfterStep) == 0
    }

    override fun signalActionReady(): Boolean {
        return MacOSLibC.sem_post(actionReady) == 0
    }

    override fun setByte(offset: Int, byte: Byte) {
        buffer.put(offset, byte)
    }

    override fun copyByteArray(offset: Int, source: ByteBuffer) {
        source.position(0)
        buffer.position(offset)
        buffer.put(source)
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

    override fun readShort(offset: Int): Short {
        return buffer.getShort(offset)
    }

    override fun shutdown() {
        MacOSLibC.sem_close(trainerInitialized)
        MacOSLibC.sem_close(resetSim)
        MacOSLibC.sem_close(actionReady)
        MacOSLibC.sem_close(actionDone)
        MacOSLibC.sem_close(resetAfterStep)

        MacOSLibC.munmap(ptr, fileSizeBytes)
    }
}