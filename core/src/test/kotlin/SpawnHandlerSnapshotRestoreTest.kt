import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.GAME_SERVER_THREAD_NAME
import com.bombbird.terminalcontrol2.gymnasium.staterestore.SpawnHandlerSnapshotData
import com.bombbird.terminalcontrol2.gymnasium.staterestore.buildSnapshot
import com.bombbird.terminalcontrol2.gymnasium.staterestore.restoreSnapshot
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.systems.TrafficSystemInterval
import com.bombbird.terminalcontrol2.traffic.ArrivalsToControlSpawner
import com.bombbird.terminalcontrol2.traffic.ScriptedSpawnEntry
import io.kotest.core.spec.style.FunSpec
import io.kotest.matchers.shouldBe
import testInitialiseGameAndServer

object SpawnHandlerSnapshotRestoreTest : FunSpec() {
    init {
        testInitialiseGameAndServer()

        beforeEach {
            Thread.currentThread().name = GAME_SERVER_THREAD_NAME
        }

        test("ArrivalsToControlSpawner getStateForSnapshot and applyStateFromSnapshot round-trip") {
            val spawner = ArrivalsToControlSpawner()
            spawner.loadScriptedSchedule(
                listOf(
                    ScriptedSpawnEntry(0f, "A", "B738", 0f, 0f, 10000f, 0f),
                    ScriptedSpawnEntry(10f, "B", "B738", 1f, 1f, 10000f, 0f)
                )
            )
            val state = SpawnHandlerSnapshotData(scriptedNextIndex = 3, secondsSinceLastScriptedSpawn = 42, spawnedCount = 1)
            spawner.applyStateFromSnapshot(state)
            spawner.getStateForSnapshot() shouldBe SpawnHandlerSnapshotData(3, 42, 1)
        }

        test("buildSnapshot captures spawn handler state; restoreSnapshot restores it") {
            val gs = GAME.gameServer!!
            ensureTrafficSystem(gs)
            val spawner = gs.engine.getSystem(TrafficSystemInterval::class.java)!!.arrivalsToControlSpawner
            spawner.loadScriptedSchedule(
                listOf(ScriptedSpawnEntry(0f, "X", "B738", 0f, 0f, 10000f, 0f))
            )
            spawner.applyStateFromSnapshot(SpawnHandlerSnapshotData(7, 33, 2))

            val snapshot = buildSnapshot(gs)
            snapshot.spawnHandlerState shouldBe SpawnHandlerSnapshotData(7, 33, 2)

            spawner.applyStateFromSnapshot(SpawnHandlerSnapshotData(0, 0, 0))
            spawner.getStateForSnapshot() shouldBe SpawnHandlerSnapshotData(0, 0, 0)

            restoreSnapshot(snapshot, gs)
            spawner.getStateForSnapshot() shouldBe SpawnHandlerSnapshotData(7, 33, 2)
        }
    }
}

private fun ensureTrafficSystem(gs: GameServer) {
    if (gs.engine.getSystem(TrafficSystemInterval::class.java) == null) {
        gs.engine.addSystem(TrafficSystemInterval(ArrivalsToControlSpawner()))
    }
}
