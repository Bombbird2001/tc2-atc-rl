import com.badlogic.ashley.core.Entity
import com.badlogic.ashley.utils.ImmutableArray
import com.bombbird.terminalcontrol2.ai.Agent
import com.badlogic.gdx.utils.Array as GdxArray
import com.bombbird.terminalcontrol2.components.AirportArrivalStats
import com.bombbird.terminalcontrol2.components.ApproachChildren
import com.bombbird.terminalcontrol2.components.SpawnGroup
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.entities.Airport
import com.bombbird.terminalcontrol2.global.AIRCRAFT_TO_SPAWN
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.GAME_SERVER_THREAD_NAME
import com.bombbird.terminalcontrol2.navigation.Approach
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.traffic.ArrivalsToControlSpawner
import com.bombbird.terminalcontrol2.traffic.ScriptedSpawnEntry
import com.bombbird.terminalcontrol2.traffic.despawnAircraft
import com.bombbird.terminalcontrol2.traffic.loadScriptedSpawnEntriesFromCsvString
import com.bombbird.terminalcontrol2.utilities.nmToPx
import com.bombbird.terminalcontrol2.components.RunwayLabel
import com.bombbird.terminalcontrol2.components.STARChildren
import com.bombbird.terminalcontrol2.navigation.SidStar
import com.bombbird.terminalcontrol2.utilities.UsabilityFilter
import io.kotest.assertions.throwables.shouldThrow
import io.kotest.core.spec.style.FunSpec
import io.kotest.matchers.shouldBe
import ktx.ashley.get
import ktx.collections.GdxArrayMap
import kotlin.random.Random

private class FakeAgent(private val fixedCount: Int): Agent {
    var incrementCount = 0
        private set
    override fun getEpisodeSpawnCount() = fixedCount
    override fun incrementSpawnCount() {
        incrementCount++
    }

    override fun despawnAircraft(aircraft: Entity) {}

    override fun init() {}

    override fun reset() {}

    override fun update(
        aircraft: GdxArrayMap<String, Aircraft>,
        deltaTime: Float,
        stopServer: () -> Unit,
    ) {}
}

object ArrivalsToControlSpawnerTest : FunSpec() {
    init {
        testInitialiseGameAndServer()

        beforeEach {
            Thread.currentThread().name = GAME_SERVER_THREAD_NAME
        }

        test("loadScriptedSpawnEntriesFromCsvString parses scripted-flights schema and derives intervals from spawn_timestamp") {
            val csv = """
                hex,episode_no,date,flight,aircraft_type,x,y,track,dist_nm,combined_alt,spawn_timestamp,landing_timestamp
                a,1,2025-01-01,TEST1,B738,100.0,200.0,270.0,40.0,14000.0,1764547954,1764548819
                b,1,2025-01-01,TEST2,A320,110.0,210.0,280.0,40.0,15000.0,1764547999,1764549129
            """.trimIndent()
            val entries = loadScriptedSpawnEntriesFromCsvString(csv)
            entries.size shouldBe 2
            entries[0] shouldBe ScriptedSpawnEntry(
                0f,
                "TEST1",
                "B738",
                nmToPx(100f),
                nmToPx(200f),
                14000f,
                270f,
                SpawnGroup.SPAWN_NORTH
            )
            entries[1].intervalSecFromLastSpawn shouldBe 45f
            entries[1].callsign shouldBe "TEST2"
            entries[1].icaoType shouldBe "A320"
        }

        test("loadScriptedSpawnEntriesFromCsvString rejects malformed row") {
            shouldThrow<IllegalArgumentException> {
                loadScriptedSpawnEntriesFromCsvString("hex,flight\nonly-one-col")
            }
        }

        test("loadScriptedSpawnEntriesFromCsvString throws when spawn_timestamp is not ascending") {
            val csv = """
                hex,episode_no,date,flight,aircraft_type,x,y,track,dist_nm,combined_alt,spawn_timestamp,landing_timestamp
                a,1,d,A,B738,1,2,270,40,10000,2000,3000
                b,1,d,B,B738,1,2,270,40,10000,1000,3000
            """.trimIndent()
            shouldThrow<IllegalArgumentException> {
                loadScriptedSpawnEntriesFromCsvString(csv)
            }
        }

        test("default policy is random RL; loadScriptedSchedule switches to scripted; clear resets") {
            val spawner = ArrivalsToControlSpawner()
            spawner.policy shouldBe ArrivalsToControlSpawner.Policy.RANDOM_RL
            spawner.loadScriptedSchedule(
                listOf(ScriptedSpawnEntry(1f, "A", "B738", 0f, 0f, 10000f, 0f))
            )
            spawner.policy shouldBe ArrivalsToControlSpawner.Policy.SCRIPTED
            spawner.clearScriptedSchedule()
            spawner.policy shouldBe ArrivalsToControlSpawner.Policy.RANDOM_RL
        }

        test("random RL does not increment spawn count when episode cap reached") {
            val gs = GAME.gameServer!!
            val arptEntity = ensureTestAirportWithIls02L(gs)
            arptEntity[AirportArrivalStats.mapper]!!.arrivalSpawnTimer = -1f
            val stats = immutableArrayOf(arptEntity)
            val arrivals = emptyEntityArray()
            val spawner = ArrivalsToControlSpawner()
            val bridge = FakeAgent(AIRCRAFT_TO_SPAWN)
            repeat(AIRCRAFT_TO_SPAWN + 1) {
                spawner.tickArrivalsToControl(1f, gs, bridge, stats, arrivals)
                arptEntity[AirportArrivalStats.mapper]!!.arrivalSpawnTimer = -1f
            }
            bridge.incrementCount shouldBe AIRCRAFT_TO_SPAWN
        }

        test("scripted spawn increments when episode cap reached (not blocked by cap)") {
            val gs = GAME.gameServer!!
            clearAllAircraftForSpawnerTest(gs)
            val arptEntity = ensureTestAirportWithIls02L(gs)
            arptEntity[AirportArrivalStats.mapper]!!.arrivalSpawnTimer = -1f
            arptEntity[STARChildren.mapper]!!.starMap.put("GABAL2A", SidStar.STAR("GABAL2A", UsabilityFilter.DAY_AND_NIGHT, ""))
            arptEntity[STARChildren.mapper]!!.starMap.put("RAKTO2A", SidStar.STAR("RAKTO2A", UsabilityFilter.DAY_AND_NIGHT, ""))
            arptEntity[STARChildren.mapper]!!.starMap.put("VEROP2A", SidStar.STAR("VEROP2A", UsabilityFilter.DAY_AND_NIGHT, ""))
            arptEntity[STARChildren.mapper]!!.starMap.put("TABUN1A", SidStar.STAR("TABUN1A", UsabilityFilter.DAY_AND_NIGHT, ""))
            val stats = immutableArrayOf(arptEntity)
            val arrivals = emptyEntityArray()
            val callsign = "SC${Random.nextInt(100000, 999999)}"
            val spawner = ArrivalsToControlSpawner()
            spawner.loadScriptedSchedule(
                listOf(
                    ScriptedSpawnEntry(
                        0f,
                        callsign,
                        "B738",
                        100f,
                        200f,
                        12000f,
                        270f
                    )
                )
            )
            val agent = FakeAgent(AIRCRAFT_TO_SPAWN)
            spawner.tickArrivalsToControl(1f, gs, agent, stats, arrivals)
            agent.incrementCount shouldBe 1
            gs.aircraft.containsKey(callsign) shouldBe true
        }
    }
}

private fun immutableArrayOf(vararg entities: Entity): ImmutableArray<Entity> {
    val a = GdxArray<Entity>()
    entities.forEach { a.add(it) }
    return ImmutableArray(a)
}

private fun emptyEntityArray(): ImmutableArray<Entity> = ImmutableArray(GdxArray())

private fun clearAllAircraftForSpawnerTest(gs: GameServer) {
    val toDespawn = mutableListOf<Entity>()
    for (i in 0 until gs.aircraft.size) {
        gs.aircraft.getValueAt(i)?.entity?.let { toDespawn.add(it) }
    }
    toDespawn.forEach { despawnAircraft(it) }
}

private fun ensureTestAirportWithIls02L(gs: GameServer): Entity {
    val arpt0 = 0.toByte()
    val existing = gs.airports[arpt0]
    if (existing != null) {
        val entity = existing.entity
        val ac = entity[ApproachChildren.mapper]!!
        if (!ac.approachMap.containsKey("ILS 02L")) {
            val approach = Approach("ILS 02L", 0, 0, 0f, 0f, 200, 1200, false, 0)
            approach.addLocalizer(270f, 20)
            ac.approachMap.put("ILS 02L", approach)
        }
        return entity
    }
    val newAirport = Airport(0, "TST", "Test", 1, 0, 0f, 0f, 0, "XXXX", false).also {
        it.addRunway(0, "02L", 0f, 0f, 270f, 3500, 0, 0, 0, "", "", RunwayLabel.BEFORE)
    }
    val approach = Approach("ILS 02L", 0, 0, 0f, 0f, 200, 1200, false, 0)
    approach.addLocalizer(270f, 20)
    newAirport.entity[ApproachChildren.mapper]!!.approachMap.put("ILS 02L", approach)
    gs.airports.put(arpt0, newAirport)
    return newAirport.entity
}
