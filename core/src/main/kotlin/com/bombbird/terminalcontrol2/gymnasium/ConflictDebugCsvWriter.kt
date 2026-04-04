package com.bombbird.terminalcontrol2.gymnasium

import com.bombbird.terminalcontrol2.gymnasium.staterestore.Snapshot
import com.bombbird.terminalcontrol2.traffic.conflict.Conflict
import com.bombbird.terminalcontrol2.utilities.convertWorldAndRenderDeg
import com.bombbird.terminalcontrol2.utilities.modulateHeading
import ktx.collections.GdxArray
import java.io.File
import java.io.FileWriter

internal fun appendConflictDebugCsv(
    outputPathOrDir: String,
    envId: String,
    snapshot: Snapshot,
    conflicts: GdxArray<Conflict>,
) {
    val file = resolveConflictDebugCsvFile(outputPathOrDir, envId)
    file.parentFile?.mkdirs()

    val needsHeader = !file.exists() || file.length() == 0L
    FileWriter(file, true).use { fw ->
        if (needsHeader) {
            fw.appendLine(
                listOf(
                    "record_type",
                    "write_id",
                    "env_id",
                    "snapshot_timestep",
                    "callsign",
                    "icao_type",
                    "x",
                    "y",
                    "altitude_ft",
                    "tas_kts",
                    "vs_fpm",
                    "track_deg",
                    "cleared_hdg_deg",
                    "cleared_alt_ft",
                    "cleared_ias_kt",
                    "loc_cap",
                    "ac1_callsign",
                    "ac2_callsign",
                    "reason_code",
                    "reason_name",
                ).joinToString(",")
            )
        }

        val writeId = System.currentTimeMillis().toString()
        val timestep = snapshot.timestep

        for ((callsign, data) in snapshot.aircraft) {
            val trackDeg = modulateHeading(convertWorldAndRenderDeg(data.direction.trackUnitVector.angleDeg()))
            val cleared = data.clearanceState
            fw.appendLine(
                csvJoin(
                    "SNAPSHOT_ROW",
                    writeId,
                    envId,
                    timestep,
                    callsign,
                    data.aircraftInfo.icaoType,
                    data.position.x,
                    data.position.y,
                    data.altitude.altitudeFt,
                    data.speed.speedKts,
                    data.speed.vertSpdFpm,
                    trackDeg,
                    cleared.vectorHdg,
                    cleared.clearedAlt,
                    cleared.clearedIas,
                    data.hasLocalizerCaptured,
                    "",
                    "",
                    "",
                    "",
                )
            )
        }

        for (i in 0 until conflicts.size) {
            val c = conflicts[i]
            val sc = c.getSerialisableObject()
            fw.appendLine(
                csvJoin(
                    "CONFLICT_ROW",
                    writeId,
                    envId,
                    timestep,
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    sc.name1,
                    sc.name2 ?: "",
                    sc.reason.toInt(),
                    conflictReasonName(sc.reason),
                )
            )
        }
    }
}

private fun resolveConflictDebugCsvFile(outputPathOrDir: String, envId: String): File {
    val f = File(outputPathOrDir)
    return if (outputPathOrDir.lowercase().endsWith(".csv")) {
        f
    } else {
        File(f, "conflict_debug_$envId.csv")
    }
}

private fun conflictReasonName(reason: Byte): String = when (reason) {
    Conflict.NORMAL_CONFLICT -> "NORMAL_CONFLICT"
    Conflict.SAME_APP_LESS_THAN_10NM -> "SAME_APP_LESS_THAN_10NM"
    Conflict.PARALLEL_DEP_APP -> "PARALLEL_DEP_APP"
    Conflict.PARALLEL_INDEP_APP_NTZ -> "PARALLEL_INDEP_APP_NTZ"
    Conflict.MVA -> "MVA"
    Conflict.SID_STAR_MVA -> "SID_STAR_MVA"
    Conflict.RESTRICTED -> "RESTRICTED"
    Conflict.WAKE_INFRINGE -> "WAKE_INFRINGE"
    Conflict.STORM -> "STORM"
    Conflict.EMERGENCY_SEPARATION_CONFLICT -> "EMERGENCY_SEPARATION_CONFLICT"
    Conflict.RL_AIRCRAFT_CONFLICT_INCREASED_MARGIN -> "RL_AIRCRAFT_CONFLICT_INCREASED_MARGIN"
    Conflict.RL_WAKE_CONFLICT_INCREASED_MARGIN -> "RL_WAKE_CONFLICT_INCREASED_MARGIN"
    else -> "UNKNOWN"
}

private fun csvJoin(vararg values: Any?): String = values.joinToString(",") { v ->
    val s = v?.toString() ?: ""
    if (s.contains(',') || s.contains('"') || s.contains('\n') || s.contains('\r')) {
        "\"" + s.replace("\"", "\"\"") + "\""
    } else {
        s
    }
}
