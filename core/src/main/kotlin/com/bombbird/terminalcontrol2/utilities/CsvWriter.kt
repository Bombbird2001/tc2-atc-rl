package com.bombbird.terminalcontrol2.utilities

import java.io.File
import java.io.FileWriter

object CsvWriter {
    private const val AVG_HOLD_TIME_FILE = "average_holding_time.csv"
    private const val HOLD_COUNT_FILE = "holding_count.csv"
    private const val INDIVIDUAL_HOLD_TIME_FILE = "individual_holding_time.csv"
    private const val WAKE_CONFLICT_FILE = "wake_conflict.csv"
    private const val CONFLICT_FILE = "conflict.csv"
    private const val MVA_CONFLICT_FILE = "mva_restricted.csv"
    private const val ARRIVAL_RATE_FILE = "arrival_rate.csv"
    private const val STEP_METRICS_FILE = "step_metrics.csv"
    private const val AGENT_LIFESPAN_FILE = "agent_lifespan.csv"
    private var RUN_DIR: String? = null

    fun setRunDirectory(runDirectory: String?) {
        if (runDirectory == null) return
        RUN_DIR = "$runDirectory/"
        println("Set run directory to $RUN_DIR")
    }

    private fun checkRunDir(): Boolean {
        return RUN_DIR?.isNotBlank() ?: false
    }

    fun writeToAverageHoldingTime(episode: Int, currTime: Float, holdingTime: Float) {
        if (!checkRunDir()) return
        writeToCsv(
            RUN_DIR + AVG_HOLD_TIME_FILE,
            listOf("Episode", "Time (s)", "Holding time (last 30 aircraft)"),
            listOf(episode, currTime, holdingTime)
        )
    }

    fun writeToHoldingCount(episode: Int, currTime: Float, holdingCount: Float) {
        if (!checkRunDir()) return
        writeToCsv(
            RUN_DIR + HOLD_COUNT_FILE,
            listOf("Episode", "Time (s)", "Aircraft in hold"),
            listOf(episode, currTime, holdingCount)
        )
    }

    fun writeToIndividualHoldTime(episode: Int, holdingTime: Float) {
        if (!checkRunDir()) return
        writeToCsv(
            RUN_DIR + INDIVIDUAL_HOLD_TIME_FILE,
            listOf("Episode", "Holding time"),
            listOf(episode, holdingTime)
        )
    }

    fun writeToWakeConflict(currTime: Float, conflicts: Float) {
        if (!checkRunDir()) return
        writeToCsv(
            RUN_DIR + WAKE_CONFLICT_FILE,
            listOf("Time (s)", "Conflicts"),
            listOf(currTime, conflicts)
        )
    }

    fun writeToConflict(currTime: Float, conflicts: Float) {
        if (!checkRunDir()) return
        writeToCsv(
            RUN_DIR + CONFLICT_FILE,
            listOf("Time (s)", "Conflicts"),
            listOf(currTime, conflicts)
        )
    }

    fun writeToMvaConflict(currTime: Float, conflicts: Float) {
        if (!checkRunDir()) return
        writeToCsv(
            RUN_DIR + MVA_CONFLICT_FILE,
            listOf("Time (s)", "Conflicts"),
            listOf(currTime, conflicts)
        )
    }

    fun writeToArrivalRate(currTime: Float, arriveRate: Float) {
        if (!checkRunDir()) return
        writeToCsv(
            RUN_DIR + ARRIVAL_RATE_FILE,
            listOf("Time (s)", "Arrival rate"),
            listOf(currTime, arriveRate)
        )
    }

    fun writeStepMetrics(episode: Int, step: Int, activeCount: Int, spawnCount: Int, mvaConflicts: Int,
                         acConflicts: Int, wakeConflicts: Int, rewards: Array<Float?>
    ) {
        if (!checkRunDir()) return
        writeToCsv(
            RUN_DIR + STEP_METRICS_FILE,
            listOf(
                "Episode", "Step", "Active count", "Total spawned", "MVA conflicts", "Aircraft conflicts", "Wake conflicts",
                *rewards.withIndex().map { "ac${it.index}" }.toTypedArray()
            ),
            listOf(episode, step, activeCount, spawnCount, mvaConflicts, acConflicts, wakeConflicts, *rewards)
        )
    }

    fun writeAgentLifespan(episode: Int, agentGroup: Byte, lifespanS: Float) {
        if (!checkRunDir()) return
        writeToCsv(
            RUN_DIR + AGENT_LIFESPAN_FILE,
            listOf("Episode", "Spawn group", "Lifespan (s)"),
            listOf(episode, agentGroup, lifespanS)
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