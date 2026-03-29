package com.bombbird.terminalcontrol2.traffic

import com.badlogic.ashley.core.Entity
import com.badlogic.ashley.utils.ImmutableArray
import com.badlogic.gdx.Gdx
import com.badlogic.gdx.files.FileHandle
import com.bombbird.terminalcontrol2.components.AirportArrivalStats
import com.bombbird.terminalcontrol2.components.AirportInfo
import com.bombbird.terminalcontrol2.components.ArrivalAirport
import com.bombbird.terminalcontrol2.components.FlightType
import com.bombbird.terminalcontrol2.components.SpawnGroup
import com.bombbird.terminalcontrol2.gymnasium.GymnasiumBridge
import com.bombbird.terminalcontrol2.global.AIRCRAFT_TO_SPAWN
import com.bombbird.terminalcontrol2.global.MAX_AIRCRAFT_ON_MAP
import com.bombbird.terminalcontrol2.global.SPAWN_INTERVAL_S
import com.bombbird.terminalcontrol2.global.WAIT_BETWEEN_SPAWNS
import com.bombbird.terminalcontrol2.gymnasium.staterestore.SpawnHandlerSnapshotData
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.utilities.nmToPx
import ktx.ashley.get
import kotlin.math.roundToInt

/**
 * RL training: spawns arrivals under [TrafficMode.ARRIVALS_TO_CONTROL].
 * Either random STAR-based spawns (default) or a scripted schedule loaded from CSV.
 */
class ArrivalsToControlSpawner {

    enum class Policy {
        /** Timer + [createRandomArrivalForAirport]; enforces episode spawn cap vs [AIRCRAFT_TO_SPAWN]. */
        RANDOM_RL,
        /** Uses [scriptedEntries]; does not enforce episode spawn cap; stops when the list is exhausted. */
        SCRIPTED
    }

    var policy: Policy = Policy.RANDOM_RL
        private set

    private var scriptedEntries: List<ScriptedSpawnEntry> = emptyList()
    private var scriptedNextIndex: Int = 0
    private var secondsSinceLastScriptedSpawn: Float = 0f
    val doneSpawning: Boolean
        get() {
            return policy == Policy.SCRIPTED && scriptedNextIndex >= scriptedEntries.size
        }

    /**
     * Activates scripted mode with the given entries (e.g. from [loadScriptedSpawnEntriesFromCsvString]).
     */
    fun loadScriptedSchedule(entries: List<ScriptedSpawnEntry>) {
        scriptedEntries = entries
        scriptedNextIndex = 0
        secondsSinceLastScriptedSpawn = 0f
        policy = Policy.SCRIPTED
    }

    /**
     * Loads a scripted spawn schedule from disk; see [loadScriptedSpawnEntriesFromCsvString] for the CSV schema
     * (same column layout as `scripted-flights` example CSVs).
     * @throws IllegalArgumentException if the file is missing or parsing fails
     */
    fun loadScheduleFromCsv(path: String) {
        val handle = Gdx.files.absolute(path)
        if (!handle.exists()) throw IllegalArgumentException("Spawn schedule file not found: $path")
        val schedule = loadScriptedSpawnEntriesFromCsv(handle).subList(0, 5)
        println("Loaded:\n${schedule.joinToString("\n")}")
        loadScriptedSchedule(schedule)
    }

    /** Captures scripted spawn progress for RL rollback ([scriptedNextIndex], [secondsSinceLastScriptedSpawn] as whole seconds). */
    fun getStateForSnapshot(): SpawnHandlerSnapshotData = SpawnHandlerSnapshotData(
        scriptedNextIndex = scriptedNextIndex,
        secondsSinceLastScriptedSpawn = secondsSinceLastScriptedSpawn.roundToInt()
    )

    /** Restores scripted spawn progress after restoreSnapshot; does not change policy or [scriptedEntries]. */
    fun applyStateFromSnapshot(data: SpawnHandlerSnapshotData) {
        scriptedNextIndex = data.scriptedNextIndex
        secondsSinceLastScriptedSpawn = data.secondsSinceLastScriptedSpawn.toFloat()
    }

    /** Clears scripted schedule and returns to random RL spawning. */
    fun clearScriptedSchedule() {
        scriptedEntries = emptyList()
        scriptedNextIndex = 0
        secondsSinceLastScriptedSpawn = 0f
        policy = Policy.RANDOM_RL
    }

    /**
     * Per-frame tick for [TrafficMode.ARRIVALS_TO_CONTROL] spawning.
     * @param interval system interval in seconds (typically 1f)
     */
    fun tickArrivalsToControl(
        interval: Float,
        gs: GameServer,
        bridge: GymnasiumBridge,
        airportArrivalStats: ImmutableArray<Entity>,
        arrivalFamilyEntities: ImmutableArray<Entity>
    ) {
        when (policy) {
            Policy.RANDOM_RL -> tickRandomRl(interval, gs, bridge, airportArrivalStats, arrivalFamilyEntities)
            Policy.SCRIPTED -> tickScripted(interval, gs, bridge, airportArrivalStats, arrivalFamilyEntities)
        }
    }

    private fun tickRandomRl(
        interval: Float,
        gs: GameServer,
        bridge: GymnasiumBridge,
        airportArrivalStats: ImmutableArray<Entity>,
        arrivalFamilyEntities: ImmutableArray<Entity>
    ) {
        for (i in 0 until airportArrivalStats.size()) {
            val arptEntity = airportArrivalStats[i]
            val arptArrStats = arptEntity[AirportArrivalStats.mapper] ?: continue
            arptArrStats.targetTrafficValue = MAX_AIRCRAFT_ON_MAP
            arptArrStats.arrivalSpawnTimer -= interval
            if (arptArrStats.arrivalSpawnTimer > 0 && WAIT_BETWEEN_SPAWNS) continue

            val arptId = arptEntity[AirportInfo.mapper]?.arptId ?: continue
            if (arptId != 0.toByte()) continue
            val arrivalCount = countArrivalsForAirport(arrivalFamilyEntities, arptId)
            arptArrStats.arrivalSpawnTimer = SPAWN_INTERVAL_S
            if (arrivalCount >= arptArrStats.targetTrafficValue) continue
            if (bridge.getEpisodeSpawnCount() >= AIRCRAFT_TO_SPAWN) continue
            createRandomArrivalForAirport(arptEntity, gs)
            bridge.incrementSpawnCount()
        }
    }

    private fun tickScripted(
        interval: Float,
        gs: GameServer,
        bridge: GymnasiumBridge,
        airportArrivalStats: ImmutableArray<Entity>,
        arrivalFamilyEntities: ImmutableArray<Entity>
    ) {
        if (doneSpawning) return

        val arptEntity = findAirportEntityWithId(airportArrivalStats, 0) ?: return
        val arptArrStats = arptEntity[AirportArrivalStats.mapper] ?: return
        arptArrStats.targetTrafficValue = MAX_AIRCRAFT_ON_MAP

        val arptId = arptEntity[AirportInfo.mapper]?.arptId ?: return
        val arrivalCount = countArrivalsForAirport(arrivalFamilyEntities, arptId)
        if (arrivalCount >= arptArrStats.targetTrafficValue) return

        secondsSinceLastScriptedSpawn += interval
        val entry = scriptedEntries[scriptedNextIndex]
        if (secondsSinceLastScriptedSpawn < entry.intervalSecFromLastSpawn) return

        createArrival(
            entry.callsign,
            entry.icaoType,
            arptEntity,
            gs,
            entry.xPx,
            entry.yPx,
            entry.altitudeFt,
            entry.trackDeg,
            disableAmendAltForNearbyTraffic = true
        )
        bridge.incrementSpawnCount()
        scriptedNextIndex++
        secondsSinceLastScriptedSpawn = 0f
    }

    private fun countArrivalsForAirport(arrivalFamilyEntities: ImmutableArray<Entity>, arptId: Byte): Int {
        var n = 0
        for (i in 0 until arrivalFamilyEntities.size()) {
            val e = arrivalFamilyEntities[i]
            if (e[FlightType.mapper]?.type == FlightType.ARRIVAL && e[ArrivalAirport.mapper]?.arptId == arptId) n++
        }
        return n
    }

    private fun findAirportEntityWithId(airportArrivalStats: ImmutableArray<Entity>, arptId: Byte): Entity? {
        for (i in 0 until airportArrivalStats.size()) {
            val e = airportArrivalStats[i]
            if (e[AirportInfo.mapper]?.arptId == arptId) return e
        }
        return null
    }
}

/** One scripted spawn: [intervalSecFromLastSpawn] is seconds after schedule start for index 0, then after each successful spawn. */
data class ScriptedSpawnEntry(
    val intervalSecFromLastSpawn: Float,
    val callsign: String,
    val icaoType: String,
    val xPx: Float,
    val yPx: Float,
    val altitudeFt: Float,
    val trackDeg: Float,
    val spawnGroup: Byte = SpawnGroup.SPAWN_NORTH
)

/**
 * Loads [ScriptedSpawnEntry] from CSV text using the `scripted-flights` schema (see e.g. `scripted-flights/dec_1.csv`).
 *
 * **Header row** (required): must name columns including `flight`, `aircraft_type`, `x`, `y`, `track`, `combined_alt`,
 * and `spawn_timestamp` (Unix time in seconds). Matching is case-insensitive.
 *
 * **Intervals:** [ScriptedSpawnEntry.intervalSecFromLastSpawn] for the first row is `0` (first spawn as soon as the
 * spawner allows). For each following row it is `spawn_timestamp[i] - spawn_timestamp[i-1]` in seconds.
 *
 * **Ordering:** rows must be non-decreasing by `spawn_timestamp`; otherwise [IllegalArgumentException] is thrown
 * (would imply a negative interval).
 *
 * Blank lines and `#` comments are skipped.
 */
fun loadScriptedSpawnEntriesFromCsvString(content: String): List<ScriptedSpawnEntry> {
    val lines = content.lines().map { it.trim() }.filter { it.isNotEmpty() && !it.startsWith("#") }
    if (lines.isEmpty()) return emptyList()

    val headerCells = lines[0].split(',').map { it.trim().lowercase() }
    val col = headerCells.mapIndexed { i, name -> name to i }.toMap()
    fun req(name: String): Int =
        col[name.lowercase()] ?: throw IllegalArgumentException("CSV header missing required column \"$name\"; got: ${lines[0]}")

    val idxFlight = req("flight")
    val idxType = req("aircraft_type")
    val idxX = req("x")
    val idxY = req("y")
    val idxTrack = req("track")
    val idxAlt = req("combined_alt")
    val idxSpawnTs = req("spawn_timestamp")

    val dataLines = lines.drop(1)
    if (dataLines.isEmpty()) return emptyList()

    val parsedRows = dataLines.map { line -> parseScriptedFlightsDataLine(line, idxFlight, idxType, idxX, idxY, idxTrack, idxAlt, idxSpawnTs) }

    var prevTs: Long? = null
    return parsedRows.mapIndexed { index, row ->
        val intervalSec = if (prevTs == null) {
            0f
        } else {
            val deltaSec = row.spawnTimestampSec - prevTs
            if (deltaSec < 0) {
                throw IllegalArgumentException(
                    "spawn_timestamp must be sorted ascending (non-decreasing); row ${index + 1} has ts=${row.spawnTimestampSec} " +
                        "after previous ts=$prevTs (negative interval ${deltaSec}s)"
                )
            }
            deltaSec.toFloat()
        }
        prevTs = row.spawnTimestampSec
        ScriptedSpawnEntry(
            intervalSecFromLastSpawn = intervalSec,
            callsign = row.callsign,
            icaoType = row.icaoType,
            xPx = row.xPx,
            yPx = row.yPx,
            altitudeFt = row.altitudeFt,
            trackDeg = row.trackDeg,
            spawnGroup = SpawnGroup.SPAWN_NORTH
        )
    }
}

private data class ParsedScriptedFlightRow(
    val callsign: String,
    val icaoType: String,
    val xPx: Float,
    val yPx: Float,
    val altitudeFt: Float,
    val trackDeg: Float,
    val spawnTimestampSec: Long
)

private fun parseScriptedFlightsDataLine(
    line: String,
    idxFlight: Int,
    idxType: Int,
    idxX: Int,
    idxY: Int,
    idxTrack: Int,
    idxAlt: Int,
    idxSpawnTs: Int
): ParsedScriptedFlightRow {
    val parts = line.split(',').map { it.trim() }
    fun getPart(i: Int, label: String): String {
        if (i >= parts.size) throw IllegalArgumentException("CSV row has too few columns (need index $i for $label): $line")
        return parts[i]
    }
    val callsign = getPart(idxFlight, "flight")
    val icaoType = getPart(idxType, "aircraft_type")
    val xPx = nmToPx(getPart(idxX, "x").toFloatOrNull() ?: throw IllegalArgumentException("Bad x: ${getPart(idxX, "x")} in line: $line"))
    val yPx = nmToPx(getPart(idxY, "y").toFloatOrNull() ?: throw IllegalArgumentException("Bad y: ${getPart(idxY, "y")} in line: $line"))
    val trackDeg = getPart(idxTrack, "track").toFloatOrNull() ?: throw IllegalArgumentException("Bad track: ${getPart(idxTrack, "track")} in line: $line")
    val altFt = getPart(idxAlt, "combined_alt").toFloatOrNull() ?: throw IllegalArgumentException("Bad combined_alt: ${getPart(idxAlt, "combined_alt")} in line: $line")
    val spawnTs = getPart(idxSpawnTs, "spawn_timestamp").toLongOrNull()
        ?: throw IllegalArgumentException("Bad spawn_timestamp: ${getPart(idxSpawnTs, "spawn_timestamp")} in line: $line")
    return ParsedScriptedFlightRow(callsign, icaoType, xPx, yPx, altFt, trackDeg, spawnTs)
}

/**
 * Loads [ScriptedSpawnEntry] from a [FileHandle] (e.g. [com.badlogic.gdx.files.FileHandle]).
 * See [loadScriptedSpawnEntriesFromCsvString] for the expected CSV schema.
 */
fun loadScriptedSpawnEntriesFromCsv(file: FileHandle): List<ScriptedSpawnEntry> =
    loadScriptedSpawnEntriesFromCsvString(file.readString())
