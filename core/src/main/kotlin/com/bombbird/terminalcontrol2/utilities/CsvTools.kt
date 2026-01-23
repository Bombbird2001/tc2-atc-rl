package com.bombbird.terminalcontrol2.utilities

import java.io.File
import java.io.FileWriter

object CsvTools {
    private const val AVG_HOLD_TIME_FILE = "average_holding_time.csv"
    private const val HOLD_COUNT_FILE = "holding_count.csv"
    private const val INDIVIDUAL_HOLD_TIME_FILE = "individual_holding_time.csv"
    private const val WAKE_CONFLICT_FILE = "wake_conflict.csv"
    private const val CONFLICT_FILE = "conflict.csv"
    private const val MVA_CONFLICT_FILE = "mva_restricted.csv"
    private const val ARRIVAL_RATE_FILE = "arrival_rate.csv"
    private const val REWARDS_FILE = "rewards.csv"

    fun clearAllMetricLogFiles() {
        deleteFile(AVG_HOLD_TIME_FILE)
        deleteFile(HOLD_COUNT_FILE)
        deleteFile(INDIVIDUAL_HOLD_TIME_FILE)
        deleteFile(WAKE_CONFLICT_FILE)
        deleteFile(CONFLICT_FILE)
        deleteFile(MVA_CONFLICT_FILE)
        deleteFile(ARRIVAL_RATE_FILE)
        deleteFile(REWARDS_FILE)
    }

    private fun deleteFile(fileName: String) {
        val file = File(fileName)

        if (file.exists()) {
            file.delete()
        }
    }

    fun writeToAverageHoldingTime(currTime: Float, holdingTime: Float) {
        writeToCsv(
            AVG_HOLD_TIME_FILE,
            listOf("Time (s)", "Holding time (last 30 aircraft)"),
            listOf(currTime, holdingTime)
        )
    }

    fun writeToHoldingCount(currTime: Float, holdingCount: Float) {
        writeToCsv(
            HOLD_COUNT_FILE,
            listOf("Time (s)", "Aircraft in hold"),
            listOf(currTime, holdingCount)
        )
    }

    fun writeToIndividualHoldTime(holdingTime: Float) {
        writeToCsv(
            INDIVIDUAL_HOLD_TIME_FILE,
            listOf("Holding time"),
            listOf(holdingTime)
        )
    }

    fun writeToWakeConflict(currTime: Float, conflicts: Float) {
        writeToCsv(
            WAKE_CONFLICT_FILE,
            listOf("Time (s)", "Conflicts"),
            listOf(currTime, conflicts)
        )
    }

    fun writeToConflict(currTime: Float, conflicts: Float) {
        writeToCsv(
            CONFLICT_FILE,
            listOf("Time (s)", "Conflicts"),
            listOf(currTime, conflicts)
        )
    }

    fun writeToMvaConflict(currTime: Float, conflicts: Float) {
        writeToCsv(
            MVA_CONFLICT_FILE,
            listOf("Time (s)", "Conflicts"),
            listOf(currTime, conflicts)
        )
    }

    fun writeToArrivalRate(currTime: Float, arriveRate: Float) {
        writeToCsv(
            ARRIVAL_RATE_FILE,
            listOf("Time (s)", "Arrival rate"),
            listOf(currTime, arriveRate)
        )
    }

    fun writeToRewards(episode: Int, rewards: Array<Float?>) {
        writeToCsv(
            REWARDS_FILE,
            listOf("Episode", *rewards.withIndex().map { "ac${it.index}" }.toTypedArray()),
            listOf(episode.toFloat(), *rewards)
        )
    }

    private fun writeToCsv(
        filePath: String,
        headers: List<String>,
        data: List<Float?>,
        append: Boolean = true
    ) {
        val file = File(filePath)

        if (!file.exists()) {
            file.parentFile?.mkdirs() // Create directories if needed
            file.createNewFile()
            FileWriter(file, true).use { writer ->
                writer.appendLine(headers.joinToString(","))
            }
        }

        // Append data
        FileWriter(file, append).use { writer ->
            writer.appendLine(data.joinToString(","))
        }
    }
}