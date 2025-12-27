package com.bombbird.terminalcontrol2.utilities

import java.io.File
import java.io.FileWriter

object CsvTools {
    fun writeToCsv(
        filePath: String,
        headers: List<String>,
        data: List<Float>,
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