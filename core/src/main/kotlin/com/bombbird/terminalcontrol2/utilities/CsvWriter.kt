package com.bombbird.terminalcontrol2.utilities

import java.io.File
import java.io.FileWriter

object CsvWriter {
    private const val WRITE_DIR = "runs/"
    private const val AVG_HOLD_TIME_FILE = "average_holding_time.csv"
    private const val HOLD_COUNT_FILE = "holding_count.csv"
    private const val INDIVIDUAL_HOLD_TIME_FILE = "individual_holding_time.csv"
    private const val WAKE_CONFLICT_FILE = "wake_conflict.csv"
    private const val CONFLICT_FILE = "conflict.csv"
    private const val MVA_CONFLICT_FILE = "mva_restricted.csv"
    private const val ARRIVAL_RATE_FILE = "arrival_rate.csv"
    private const val STEP_METRICS_FILE = "step_metrics.csv"
    private var RUN_DIR = ""

    fun setRunDirectory(runDirectory: String) {
        RUN_DIR = "$runDirectory/"
    }

    fun writeToAverageHoldingTime(currTime: Float, holdingTime: Float) {
        writeToCsv(
            WRITE_DIR + RUN_DIR + AVG_HOLD_TIME_FILE,
            listOf("Time (s)", "Holding time (last 30 aircraft)"),
            listOf(currTime, holdingTime)
        )
    }

    fun writeToHoldingCount(currTime: Float, holdingCount: Float) {
        writeToCsv(
            WRITE_DIR + RUN_DIR + HOLD_COUNT_FILE,
            listOf("Time (s)", "Aircraft in hold"),
            listOf(currTime, holdingCount)
        )
    }

    fun writeToIndividualHoldTime(holdingTime: Float) {
        writeToCsv(
            WRITE_DIR + RUN_DIR + INDIVIDUAL_HOLD_TIME_FILE,
            listOf("Holding time"),
            listOf(holdingTime)
        )
    }

    fun writeToWakeConflict(currTime: Float, conflicts: Float) {
        writeToCsv(
            WRITE_DIR + RUN_DIR + WAKE_CONFLICT_FILE,
            listOf("Time (s)", "Conflicts"),
            listOf(currTime, conflicts)
        )
    }

    fun writeToConflict(currTime: Float, conflicts: Float) {
        writeToCsv(
            WRITE_DIR + RUN_DIR + CONFLICT_FILE,
            listOf("Time (s)", "Conflicts"),
            listOf(currTime, conflicts)
        )
    }

    fun writeToMvaConflict(currTime: Float, conflicts: Float) {
        writeToCsv(
            WRITE_DIR + RUN_DIR + MVA_CONFLICT_FILE,
            listOf("Time (s)", "Conflicts"),
            listOf(currTime, conflicts)
        )
    }

    fun writeToArrivalRate(currTime: Float, arriveRate: Float) {
        writeToCsv(
            WRITE_DIR + RUN_DIR + ARRIVAL_RATE_FILE,
            listOf("Time (s)", "Arrival rate"),
            listOf(currTime, arriveRate)
        )
    }

    fun writeStepMetrics(episode: Int, step: Int, activeCount: Int, spawnCount: Int, mvaConflicts: Int,
                         acConflicts: Int, wakeConflicts: Int, rewards: Array<Float?>
    ) {
        writeToCsv(
            WRITE_DIR + RUN_DIR + STEP_METRICS_FILE,
            listOf(
                "Episode", "Step", "Active count", "Total spawned", "MVA conflicts", "Aircraft conflicts", "Wake conflicts",
                *rewards.withIndex().map { "ac${it.index}" }.toTypedArray()
            ),
            listOf(episode, step, activeCount, spawnCount, mvaConflicts, acConflicts, wakeConflicts, *rewards)
        )
    }

    private fun writeToCsv(
        filePath: String,
        headers: List<String>,
        data: List<Number?>,
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