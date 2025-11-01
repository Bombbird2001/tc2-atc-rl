package com.bombbird.terminalcontrol2.gymnasium.ipc

import java.nio.ByteBuffer

interface SharedMemoryIPC {
    companion object {
        const val SHM_FILE_PREFIX = "ATCSharedMem"
        const val TRAINER_INITIALIZED = "ATCTrainerInit"
        const val RESET_PREFIX = "ATCResetEvent"
        const val ACTION_READY_PREFIX = "ATCActionReadyEvent"
        const val ACTION_DONE_PREFIX = "ATCActionDoneEvent"
        const val RESET_AFTER_STEP_PREFIX = "ATCResetAfterEvent"
    }

    fun waitForTrainerInitialized()

    fun needsResetSim(): Boolean

    /**
     * Waits for the actionDone flag to be set; returns false if the flag is not set after waiting for
     * [maxWaitTimeMs], else returns true
     *
     * WARNING: The timeout does not work on MacOS - the semaphore will wait indefinitely if not signalled
     */
    fun waitForActionDone(maxWaitTimeMs: Int): Boolean

    fun needsResetAfterStep(): Boolean

    fun signalActionReady(): Boolean

    fun setByte(offset: Int, byte: Byte)

    fun copyByteArray(offset: Int, source: ByteBuffer)

    fun setFloat(offset: Int, float: Float)

    fun setInt(offset: Int, int: Int)

    fun readBytes(offset: Int, bytes: Int): ByteArray

    fun readShort(offset: Int): Short
}

object SharedMemoryIPCFactory {
    fun getSharedMemory(envId: String, fileSizeBytes: Int): SharedMemoryIPC {
        val osName = System.getProperty("os.name").lowercase()
        val isWindows = osName.contains("win")
        val isMac = osName.contains("mac")
        val isLinux = osName.contains("nux") || osName.contains("nix")

        if (isWindows) return WindowsSharedMemory(envId, fileSizeBytes)
        if (isLinux) return PosixSharedMemory(envId, fileSizeBytes.toLong())
        if (isMac) return MacOSSharedMemory(envId, fileSizeBytes.toLong())

        throw IllegalArgumentException("Unsupported OS: $osName")
    }
}