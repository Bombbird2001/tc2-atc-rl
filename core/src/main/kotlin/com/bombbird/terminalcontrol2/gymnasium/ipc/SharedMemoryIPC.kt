package com.bombbird.terminalcontrol2.gymnasium.ipc

interface SharedMemoryIPC {
    companion object {
        const val SHM_FILE_PREFIX = "ATCRLSharedMem"
        const val RESET_PREFIX = "ATCRLResetEvent"
        const val ACTION_READY_PREFIX = "ATCRLActionReadyEvent"
        const val ACTION_DONE_PREFIX = "ATCRLActionDoneEvent"
        const val RESET_AFTER_STEP_PREFIX = "ATCRLResetAfterEvent"
    }

    fun needsResetSim(): Boolean

    fun waitForActionDone(maxWaitTimeMs: Int): Boolean

    fun needsResetAfterStep(): Boolean

    fun signalActionReady(): Boolean

    fun setByte(offset: Long, byte: Byte)

    fun setFloat(offset: Long, float: Float)

    fun setInt(offset: Long, int: Int)

    fun readBytes(offset: Long, bytes: Int): ByteArray
}