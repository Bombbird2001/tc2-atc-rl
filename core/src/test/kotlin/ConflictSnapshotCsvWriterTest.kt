import com.badlogic.gdx.utils.Array as GdxArray
import com.bombbird.terminalcontrol2.components.*
import com.bombbird.terminalcontrol2.gymnasium.appendConflictDebugCsv
import com.bombbird.terminalcontrol2.gymnasium.staterestore.buildSnapshot
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.GAME_SERVER_THREAD_NAME
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.traffic.conflict.Conflict
import io.kotest.core.spec.style.FunSpec
import io.kotest.matchers.collections.shouldContain
import io.kotest.matchers.shouldBe
import java.io.File
import java.nio.file.Files
import ktx.ashley.get
import com.bombbird.terminalcontrol2.entities.Aircraft

object ConflictSnapshotCsvWriterTest : FunSpec() {
    init {
        testInitialiseGameAndServer()

        beforeEach {
            Thread.currentThread().name = GAME_SERVER_THREAD_NAME
        }

        test("appendConflictDebugCsv writes header once and appends sequential snapshot+conflict rows") {
            val gs = GAME.gameServer!!
            clearAllAircraft(gs)
            val ac1 = addTestAircraft(gs, "CSVA1", 10f, 20f, 5000f)
            val ac2 = addTestAircraft(gs, "CSVB2", 15f, 25f, 6000f)

            val snap1 = buildSnapshot(gs).copy(timestep = 111)
            val conflicts = GdxArray<Conflict>(1).also {
                it.add(Conflict(ac1.entity, ac2.entity, null, 3f, Conflict.NORMAL_CONFLICT))
            }

            val dir = Files.createTempDirectory("tc2_conflict_csv").toFile()
            val out = File(dir, "debug.csv").absolutePath

            appendConflictDebugCsv(out, envId = "env_test", snapshot = snap1, conflicts = conflicts)
            val sizeAfter1 = File(out).readLines().size
            sizeAfter1 shouldBe (1 + snap1.aircraft.size + conflicts.size)

            val snap2 = buildSnapshot(gs).copy(timestep = 222)
            appendConflictDebugCsv(out, envId = "env_test", snapshot = snap2, conflicts = conflicts)
            val lines = File(out).readLines()

            // Header once
            lines.first().split(',') shouldContain "record_type"
            lines.count { it.startsWith("record_type,") } shouldBe 1

            // At least one row of each type
            lines.any { it.startsWith("SNAPSHOT_ROW,") } shouldBe true
            lines.any { it.startsWith("CONFLICT_ROW,") } shouldBe true

            // Required clearance columns exist in header
            val header = lines.first()
            header.split(',') shouldContain "cleared_hdg_deg"
            header.split(',') shouldContain "cleared_alt_ft"
            header.split(',') shouldContain "cleared_ias_kt"
        }
    }
}

private fun addTestAircraft(gs: GameServer, callsign: String, x: Float, y: Float, alt: Float): Aircraft {
    val ac = Aircraft(callsign, x, y, alt, "B738", FlightType.ARRIVAL, false)
    // Ensure buildSnapshot has the required components (it uses !! for several).
    if (ac.entity[Speed.mapper] == null) ac.entity.add(Speed())
    if (ac.entity[Direction.mapper] == null) ac.entity.add(Direction())
    if (ac.entity[GroundTrack.mapper] == null) ac.entity.add(GroundTrack())
    if (ac.entity[IndicatedAirSpeed.mapper] == null) ac.entity.add(IndicatedAirSpeed(250f))
    if (ac.entity[CommandTarget.mapper] == null) ac.entity.add(CommandTarget())
    if (ac.entity[Acceleration.mapper] == null) ac.entity.add(Acceleration())
    if (ac.entity[WakeTolerance.mapper] == null) ac.entity.add(WakeTolerance())
    if (ac.entity[WakeTrail.mapper] == null) ac.entity.add(WakeTrail())
    if (ac.entity[SpawnGroup.mapper] == null) ac.entity.add(SpawnGroup(0))
    gs.aircraft.put(callsign, ac)
    return ac
}

private fun clearAllAircraft(gs: GameServer) {
    val toRemove = mutableListOf<String>()
    for (i in 0 until gs.aircraft.size) {
        val key = gs.aircraft.getKeyAt(i)
        if (key != null) toRemove.add(key)
    }
    toRemove.forEach { gs.aircraft.removeKey(it) }
}

