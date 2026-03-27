package com.bombbird.terminalcontrol2.gymnasium

import com.bombbird.terminalcontrol2.global.AIRCRAFT_TO_SPAWN
import com.bombbird.terminalcontrol2.gymnasium.ipc.SharedMemoryIPC

class MetricsHandler(private val metricOffsetStart: Int) {
    companion object {
        const val LANDING_RATE = 0
        const val CONFLICT_RATE_NO_LOC = 1
        const val MVA_CONFLICT_RATE = 2
        const val WAKE_CONFLICT_RATE_NO_LOC = 3
        const val CONFLICT_RATE_LOC = 4
        const val WAKE_CONFLICT_RATE_LOC = 5
        const val CONFLICT_RATE_NO_LOC_BEFORE_RES = 6
        const val CONFLICT_RATE_LOC_BEFORE_RES = 7
        const val MVA_CONFLICT_RATE_BEFORE_RES = 8
        const val WAKE_CONFLICT_RATE_NO_LOC_BEFORE_RES = 9
        const val WAKE_CONFLICT_RATE_LOC_BEFORE_RES = 10
        const val CLEARANCE_CHANGE_RATE = 11
        const val AIRCRAFT_SPAWN_GROUP = 12

        const val DEFAULT_METRIC_SIZE_BYTES = 4
    }

    private val metricSizes = listOf(
        LANDING_RATE to DEFAULT_METRIC_SIZE_BYTES,
        CONFLICT_RATE_NO_LOC to DEFAULT_METRIC_SIZE_BYTES,
        MVA_CONFLICT_RATE to DEFAULT_METRIC_SIZE_BYTES,
        WAKE_CONFLICT_RATE_NO_LOC to DEFAULT_METRIC_SIZE_BYTES,
        CONFLICT_RATE_LOC to DEFAULT_METRIC_SIZE_BYTES,
        WAKE_CONFLICT_RATE_LOC to DEFAULT_METRIC_SIZE_BYTES,
        CONFLICT_RATE_NO_LOC_BEFORE_RES to DEFAULT_METRIC_SIZE_BYTES,
        CONFLICT_RATE_LOC_BEFORE_RES to DEFAULT_METRIC_SIZE_BYTES,
        MVA_CONFLICT_RATE_BEFORE_RES to DEFAULT_METRIC_SIZE_BYTES,
        WAKE_CONFLICT_RATE_NO_LOC_BEFORE_RES to DEFAULT_METRIC_SIZE_BYTES,
        WAKE_CONFLICT_RATE_LOC_BEFORE_RES to DEFAULT_METRIC_SIZE_BYTES,
        CLEARANCE_CHANGE_RATE to DEFAULT_METRIC_SIZE_BYTES,
        AIRCRAFT_SPAWN_GROUP to 1 * AIRCRAFT_TO_SPAWN,
    )

    val size = metricSizes.sumOf { it.second }

    private val metricOffsets = ArrayList<Triple<Int, Int, Int>>(metricSizes.size).apply {
        var accum = metricOffsetStart
        metricSizes.forEach {
            add(Triple(it.first, accum, it.second))
            accum += it.second
        }
    }

    private lateinit var sharedMemoryIPC: SharedMemoryIPC

    fun init(sharedMemoryIPC: SharedMemoryIPC) {
        if (::sharedMemoryIPC.isInitialized) throw IllegalStateException("SharedMemoryIPC already initialized")
        this.sharedMemoryIPC = sharedMemoryIPC
    }

    fun logToSharedMemory(metricId: Int, offset: Int, value: Float) {
        val (_, posStart, maxSize) = metricOffsets.find { it.first == metricId } ?: throw IllegalArgumentException("No metric with ID $metricId")
        if ((offset + 1) * DEFAULT_METRIC_SIZE_BYTES > maxSize) {
            throw IllegalArgumentException("Offset is too large; max offset is ${maxSize / DEFAULT_METRIC_SIZE_BYTES - 1}, got $offset")
        }
        sharedMemoryIPC.setFloat(posStart + offset * DEFAULT_METRIC_SIZE_BYTES, value)
    }

    fun logToSharedMemory(metricId: Int, offset: Int, value: Byte) {
        val (_, posStart, maxSize) = metricOffsets.find { it.first == metricId } ?: throw IllegalArgumentException("No metric with ID $metricId")
        if (offset >= maxSize) {
            throw IllegalArgumentException("Offset is too large; max offset is ${maxSize - 1}, got $offset")
        }
        sharedMemoryIPC.setByte(posStart + offset, value)
    }

    fun logToSharedMemory(metricId: Int, value: Float) {
        logToSharedMemory(metricId, 0, value)
    }
}