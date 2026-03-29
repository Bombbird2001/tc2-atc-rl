import com.badlogic.gdx.utils.ArrayMap.Entries
import com.bombbird.terminalcontrol2.components.*
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.entities.Airport
import com.bombbird.terminalcontrol2.entities.WakeZone
import com.bombbird.terminalcontrol2.gymnasium.PythonGymnasiumBridge
import com.bombbird.terminalcontrol2.gymnasium.staterestore.*
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.GAME_SERVER_THREAD_NAME
import com.bombbird.terminalcontrol2.global.MAX_RL_AIRCRAFT
import com.badlogic.gdx.utils.Queue
import com.bombbird.terminalcontrol2.navigation.Approach
import com.bombbird.terminalcontrol2.navigation.ClearanceState
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.networking.RLHeadlessTrainingConfig
import com.bombbird.terminalcontrol2.systems.TrafficSystemInterval
import com.bombbird.terminalcontrol2.traffic.despawnAircraft
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.ai.reward.RewardHandler
import com.bombbird.terminalcontrol2.traffic.ArrivalsToControlSpawner
import io.kotest.assertions.throwables.shouldThrow
import io.kotest.core.spec.style.FunSpec
import io.kotest.matchers.floats.plusOrMinus
import io.kotest.matchers.nulls.shouldBeNull
import io.kotest.matchers.nulls.shouldNotBeNull
import io.kotest.matchers.shouldBe
import ktx.ashley.get
import ktx.ashley.getSystem
import ktx.ashley.has
import ktx.ashley.plusAssign
import ktx.ashley.remove

/**
 * Unit tests for RL state snapshot and restore (forward-looking conflict resolution).
 * Covers: component persistence, spawn/despawn semantics, wake turbulence restore.
 */
object RLStateRestoreTest : FunSpec() {

    init {
        testInitialiseGameAndServer()

        beforeEach {
            Thread.currentThread().name = GAME_SERVER_THREAD_NAME
            GAME.gameServer?.let { gs ->
                ensureTrafficSystem(gs)
                clearAllAircraft(gs)
                clearAllRunwayOccupied(gs)
            }
        }

        test("Component values match after snapshot and restore (no spawn/despawn)") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val ac = addTestAircraft(gs, "TST01", 100f, 200f, 5000f)
            ac.entity[Position.mapper]!!.apply { x = 150f; y = 250f }
            ac.entity[Altitude.mapper]!!.altitudeFt = 6000f
            ac.entity[Speed.mapper]!!.apply { speedKts = 220f; vertSpdFpm = 500f }
            ac.entity[CommandTarget.mapper]!!.apply { targetHdgDeg = 90f; targetAltFt = 7000; targetIasKt = 250 }
            ac.entity[Acceleration.mapper]!!.apply { dSpeedMps2 = 1.5f; dVertSpdMps2 = 2.5f; dAngularSpdDps2 = 3f }
            ac.entity[FlightType.mapper]!!.type = FlightType.EN_ROUTE
            ac.entity[ClearanceAct.mapper]!!.actingClearance.clearanceState.apply {
                clearedAlt = 7000
                clearedIas = 250
                vectorHdg = 90
            }
            val snapshot = buildSnapshot(gs)
            // Mutate state
            ac.entity[Position.mapper]!!.apply { x = 999f; y = 999f }
            ac.entity[Altitude.mapper]!!.altitudeFt = 10000f
            ac.entity[Speed.mapper]!!.speedKts = 100f
            ac.entity[CommandTarget.mapper]!!.targetHdgDeg = 180f
            ac.entity[Acceleration.mapper]!!.dSpeedMps2 = 99f
            ac.entity[FlightType.mapper]!!.type = FlightType.DEPARTURE
            ac.entity[ClearanceAct.mapper]!!.actingClearance.clearanceState.clearedAlt = 15000
            ac.entity[SpawnGroup.mapper]!!.spawnGroup = 1
            restoreSnapshot(snapshot, gs)
            (ac.entity[Position.mapper]!!.x) shouldBe 150f
            (ac.entity[Position.mapper]!!.y) shouldBe 250f
            (ac.entity[Altitude.mapper]!!.altitudeFt) shouldBe 6000f
            (ac.entity[Speed.mapper]!!.speedKts) shouldBe 220f
            (ac.entity[Speed.mapper]!!.vertSpdFpm) shouldBe 500f
            (ac.entity[CommandTarget.mapper]!!.targetHdgDeg) shouldBe 90f
            (ac.entity[CommandTarget.mapper]!!.targetAltFt) shouldBe 7000
            (ac.entity[CommandTarget.mapper]!!.targetIasKt) shouldBe 250
            (ac.entity[ClearanceAct.mapper]!!.actingClearance.clearanceState.clearedAlt) shouldBe 7000
            (ac.entity[ClearanceAct.mapper]!!.actingClearance.clearanceState.clearedIas) shouldBe 250
            (ac.entity[ClearanceAct.mapper]!!.actingClearance.clearanceState.vectorHdg) shouldBe 90
            (ac.entity[Acceleration.mapper]!!.dSpeedMps2) shouldBe 1.5f
            (ac.entity[Acceleration.mapper]!!.dVertSpdMps2) shouldBe 2.5f
            (ac.entity[Acceleration.mapper]!!.dAngularSpdDps2) shouldBe 3f
            (ac.entity[FlightType.mapper]!!.type) shouldBe FlightType.EN_ROUTE
            (ac.entity[SpawnGroup.mapper]!!.spawnGroup) shouldBe 0
        }

        test("Clearance state (clearedAlt, clearedIas, vectorHdg, expedite) is restored correctly") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val ac = addTestAircraft(gs, "CLR01", 0f, 0f, 3000f)
            val cs = ac.entity[ClearanceAct.mapper]!!.actingClearance.clearanceState
            cs.clearedAlt = 11000
            cs.clearedIas = 280
            cs.vectorHdg = 180
            cs.expedite = true
            cs.routePrimaryName = "TESTRTE"
            val snapshot = buildSnapshot(gs)
            cs.clearedAlt = 0
            cs.clearedIas = 0
            cs.vectorHdg = null
            cs.expedite = false
            cs.routePrimaryName = ""
            restoreSnapshot(snapshot, gs)
            val restored = gs.aircraft.get("CLR01")!!.entity[ClearanceAct.mapper]!!.actingClearance.clearanceState
            restored.clearedAlt shouldBe 11000
            restored.clearedIas shouldBe 280
            restored.vectorHdg shouldBe 180
            restored.expedite shouldBe true
            restored.routePrimaryName shouldBe "TESTRTE"
        }

        test("LastRestrictions, WakeTolerance, Acceleration and FlightType are restored correctly") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val ac = addTestAircraft(gs, "LRA01", 0f, 0f, 5000f)
            ac.entity.plusAssign(LastRestrictions(3000, 15000, 250))
            ac.entity[WakeTolerance.mapper]!!.accumulation = 0.75f
            ac.entity[Acceleration.mapper]!!.apply { dSpeedMps2 = 0.5f; dVertSpdMps2 = 1.2f; dAngularSpdDps2 = 2f }
            ac.entity[FlightType.mapper]!!.type = FlightType.DEPARTURE
            val snapshot = buildSnapshot(gs)
            (ac.entity[LastRestrictions.mapper]!!.minAltFt) shouldBe 3000
            ac.entity[LastRestrictions.mapper]!!.maxAltFt = 0
            ac.entity[WakeTolerance.mapper]!!.accumulation = 0f
            ac.entity[Acceleration.mapper]!!.dSpeedMps2 = 99f
            ac.entity[FlightType.mapper]!!.type = FlightType.ARRIVAL
            restoreSnapshot(snapshot, gs)
            ac.entity[LastRestrictions.mapper]!!.minAltFt shouldBe 3000
            ac.entity[LastRestrictions.mapper]!!.maxAltFt shouldBe 15000
            ac.entity[LastRestrictions.mapper]!!.maxSpdKt shouldBe 250
            ac.entity[WakeTolerance.mapper]!!.accumulation shouldBe 0.75f
            ac.entity[Acceleration.mapper]!!.dSpeedMps2 shouldBe 0.5f
            ac.entity[Acceleration.mapper]!!.dVertSpdMps2 shouldBe 1.2f
            ac.entity[Acceleration.mapper]!!.dAngularSpdDps2 shouldBe 2f
            ac.entity[FlightType.mapper]!!.type shouldBe FlightType.DEPARTURE
        }

        test("RecentGoAround and DivergentDepartureAllowed are restored correctly") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val ac = addTestAircraft(gs, "RGA01", 0f, 0f, 2000f)
            ac.entity.plusAssign(RecentGoAround(45f, 1))
            ac.entity.plusAssign(DivergentDepartureAllowed(90f))
            val snapshot = buildSnapshot(gs)
            ac.entity[RecentGoAround.mapper]!!.timeLeft = 0f
            ac.entity[DivergentDepartureAllowed.mapper]!!.timeLeft = 0f
            restoreSnapshot(snapshot, gs)
            ac.entity[RecentGoAround.mapper]!!.timeLeft shouldBe 45f
            ac.entity[RecentGoAround.mapper]!!.reason shouldBe 1
            ac.entity[DivergentDepartureAllowed.mapper]!!.timeLeft shouldBe 90f
        }

        test("TakeoffClimb, LandingRoll and EmergencyPending are restored correctly") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val ac = addTestAircraft(gs, "TAG01", 0f, 0f, 1000f)
            ac.entity.plusAssign(TakeoffClimb(2000f))
            ac.entity.plusAssign(LandingRoll())
            ac.entity.plusAssign(EmergencyPending(true, EmergencyPending.ENGINE_FAIL, 5000))
            val snapshot = buildSnapshot(gs)
            ac.entity[Position.mapper]!!.x = 999f
            ac.entity[EmergencyPending.mapper]!!.apply { active = false; activationAlt = 0 }
            restoreSnapshot(snapshot, gs)
            ac.entity.has(TakeoffClimb.mapper) shouldBe true
            ac.entity.has(LandingRoll.mapper) shouldBe true
            val ep = ac.entity[EmergencyPending.mapper].shouldNotBeNull()
            ep.active shouldBe true
            ep.type shouldBe EmergencyPending.ENGINE_FAIL
            ep.activationAlt shouldBe 5000
            (ac.entity[Position.mapper]!!.x) shouldBe 0f
        }

        test("Pending clearances queue and timeLeft are restored correctly") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val ac = addTestAircraft(gs, "PND01", 0f, 0f, 4000f)
            val q = Queue<ClearanceState.PendingClearanceState>(5)
            q.addLast(ClearanceState.PendingClearanceState(5f, ClearanceState(clearedAlt = 6000, clearedIas = 250)))
            q.addLast(ClearanceState.PendingClearanceState(12f, ClearanceState(clearedAlt = 8000, clearedIas = 220)))
            ac.entity.plusAssign(PendingClearances(q))
            val snapshot = buildSnapshot(gs)
            ac.entity[PendingClearances.mapper]!!.clearanceQueue.clear()
            ac.entity[PendingClearances.mapper]!!.clearanceQueue.addLast(
                ClearanceState.PendingClearanceState(99f, ClearanceState(clearedAlt = 0))
            )
            restoreSnapshot(snapshot, gs)
            val restoredQ = gs.aircraft.get("PND01")!!.entity[PendingClearances.mapper]!!.clearanceQueue
            restoredQ.size shouldBe 2
            val it = Queue.QueueIterator(restoredQ)
            val first = it.next()
            first.timeLeft shouldBe 5f
            first.clearanceState.clearedAlt shouldBe 6000
            first.clearanceState.clearedIas shouldBe 250
            val second = it.next()
            second.timeLeft shouldBe 12f
            second.clearanceState.clearedAlt shouldBe 8000
            second.clearanceState.clearedIas shouldBe 220
        }

        test("Two aircraft: both present after restore with correct components") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            addTestAircraft(gs, "TST02", 10f, 20f, 3000f)
            addTestAircraft(gs, "TST03", 30f, 40f, 4000f)
            val snapshot = buildSnapshot(gs)
            gs.aircraft.size shouldBe 2
            // Change state then restore
            gs.aircraft.get("TST02")!!.entity[Position.mapper]!!.x = 999f
            restoreSnapshot(snapshot, gs)
            gs.aircraft.size shouldBe 2
            (gs.aircraft.get("TST02")!!.entity[Position.mapper]!!.x) shouldBe 10f
            (gs.aircraft.get("TST03")!!.entity[Position.mapper]!!.x) shouldBe 30f
        }

        test("Despawned aircraft is re-created on restore") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            addTestAircraft(gs, "TST04", 50f, 60f, 7000f)
            val snapshot = buildSnapshot(gs)
            val ac = gs.aircraft.get("TST04").shouldNotBeNull()
            despawnAircraft(ac.entity)
            gs.aircraft.get("TST04").shouldBeNull()
            restoreSnapshot(snapshot, gs)
            val restoredAc = gs.aircraft.get("TST04").shouldNotBeNull()
            (restoredAc.entity[Position.mapper]!!.x) shouldBe 50f
            (restoredAc.entity[Position.mapper]!!.y) shouldBe 60f
            (restoredAc.entity[Altitude.mapper]!!.altitudeFt) shouldBe 7000f
        }

        test("Aircraft spawned after snapshot is despawned on restore") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            addTestAircraft(gs, "TST05", 1f, 2f, 2000f)
            val snapshot = buildSnapshot(gs)
            addTestAircraft(gs, "TST06", 3f, 4f, 3000f)
            gs.aircraft.size shouldBe 2
            restoreSnapshot(snapshot, gs)
            gs.aircraft.size shouldBe 1
            gs.aircraft.get("TST05").shouldNotBeNull()
            gs.aircraft.get("TST06").shouldBeNull()
        }

        test("Wake trail is restored with correct count and distNmCounter") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val ac = addTestAircraft(gs, "TST07", 0f, 0f, 5000f)
            val trail = ac.entity[WakeTrail.mapper].shouldNotBeNull()
            trail.distNmCounter = 1.5f
            val pos1 = Position(10f, 20f)
            val pos2 = Position(30f, 40f)
            val wz1 = WakeZone(0f, 0f, 10f, 20f, 5000f, "TST07", 'H', 'B', null, null)
            val wz2 = WakeZone(10f, 20f, 30f, 40f, 5000f, "TST07", 'H', 'B', null, null)
            trail.wakeZones.addLast(pos1 to wz1)
            trail.wakeZones.addLast(pos2 to wz2)
            gs.engine.getSystem<TrafficSystemInterval>().addWakeZone(wz1)
            gs.engine.getSystem<TrafficSystemInterval>().addWakeZone(wz2)
            val snapshot = buildSnapshot(gs)
            trail.wakeZones.clear()
            trail.distNmCounter = 99f
            restoreSnapshot(snapshot, gs)
            val restoredAc = gs.aircraft.get("TST07").shouldNotBeNull()
            val restoredTrail = restoredAc.entity[WakeTrail.mapper].shouldNotBeNull()
            (restoredTrail.distNmCounter) shouldBe (1.5f.plusOrMinus(0.001f))
            restoredTrail.wakeZones.size shouldBe 2
        }

        test("RLStateRestoreManager addSnapshot keeps only latest n snapshots") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val manager = RLStateRestoreManager(3)
            manager.snapshotCount() shouldBe 0
            addTestAircraft(gs, "MGR01", 0f, 0f, 1000f)
            manager.addSnapshot(manager.getSnapshot(gs))
            manager.addSnapshot(manager.getSnapshot(gs))
            manager.addSnapshot(manager.getSnapshot(gs))
            manager.snapshotCount() shouldBe 3
            manager.addSnapshot(manager.getSnapshot(gs))
            manager.snapshotCount() shouldBe 3
        }

        test("RLStateRestoreManager restoreSnapshot removes later snapshots") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val manager = RLStateRestoreManager(5)
            val actions1 = mapOf("MGR02" to intArrayOf(1, 2, 3))
            val actions2 = mapOf("MGR02" to intArrayOf(4, 5, 6))
            val actions3 = mapOf("MGR02" to intArrayOf(7, 8, 9))
            
            addTestAircraft(gs, "MGR02", 1f, 1f, 2000f)
            manager.addSnapshot(manager.getSnapshot(gs))
            manager.updateLatestSnapshotActions(actions1)
            
            gs.aircraft.get("MGR02")!!.entity[Position.mapper]!!.x = 2f
            manager.addSnapshot(manager.getSnapshot(gs))
            manager.updateLatestSnapshotActions(actions2)
            
            gs.aircraft.get("MGR02")!!.entity[Position.mapper]!!.x = 3f
            manager.addSnapshot(manager.getSnapshot(gs))
            manager.updateLatestSnapshotActions(actions3)
            
            manager.snapshotCount() shouldBe 3
            val restoredSnapshot = manager.restoreSnapshot(2, gs)
            
            manager.snapshotCount() shouldBe 2
            (gs.aircraft.get("MGR02")!!.entity[Position.mapper]!!.x) shouldBe 2f
            
            // Verify that the returned snapshot has the correct actions
            restoredSnapshot.actions["MGR02"]?.get(0) shouldBe 4
            restoredSnapshot.actions["MGR02"]?.get(1) shouldBe 5
            restoredSnapshot.actions["MGR02"]?.get(2) shouldBe 6
        }

        test("RLStateRestoreManager clearing of later snapshots after restore is correct") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val manager = RLStateRestoreManager(5)
            addTestAircraft(gs, "MGR03", 10f, 10f, 1000f)
            manager.addSnapshot(manager.getSnapshot(gs))  // snapshot A: x=10
            manager.updateLatestSnapshotActions(mapOf("MGR03" to intArrayOf(1, 1, 1)))
            
            gs.aircraft.get("MGR03")!!.entity[Position.mapper]!!.x = 20f
            manager.addSnapshot(manager.getSnapshot(gs))  // snapshot B: x=20
            manager.updateLatestSnapshotActions(mapOf("MGR03" to intArrayOf(2, 2, 2)))
            
            gs.aircraft.get("MGR03")!!.entity[Position.mapper]!!.x = 30f
            manager.addSnapshot(manager.getSnapshot(gs))  // snapshot C: x=30
            manager.updateLatestSnapshotActions(mapOf("MGR03" to intArrayOf(3, 3, 3)))
            
            gs.aircraft.get("MGR03")!!.entity[Position.mapper]!!.x = 40f
            manager.addSnapshot(manager.getSnapshot(gs))  // snapshot D: x=40
            manager.updateLatestSnapshotActions(mapOf("MGR03" to intArrayOf(4, 4, 4)))
            
            manager.snapshotCount() shouldBe 4
            
            // Check getSnapshotAt correctly retrieves snapshot B (index from the end)
            val bSnap = manager.getSnapshotAt(3)
            bSnap.shouldNotBeNull()
            bSnap.actions["MGR03"]?.get(0) shouldBe 2
            
            val snapshotB = manager.restoreSnapshot(3, gs)  // restore to snapshot B (second oldest of 4)
            manager.snapshotCount() shouldBe 2  // only A and B remain
            (gs.aircraft.get("MGR03")!!.entity[Position.mapper]!!.x) shouldBe 20f
            snapshotB.actions["MGR03"]?.get(0) shouldBe 2
            
            val snapshotA = manager.restoreSnapshot(2, gs)  // restore to snapshot A (oldest of remaining 2)
            manager.snapshotCount() shouldBe 1
            (gs.aircraft.get("MGR03")!!.entity[Position.mapper]!!.x) shouldBe 10f
            snapshotA.actions["MGR03"]?.get(0) shouldBe 1
        }

        test("RLStateRestoreManager restoreSnapshot throws when stepsAgo greater than snapshot count") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val manager = RLStateRestoreManager(3)
            addTestAircraft(gs, "MGR04", 0f, 0f, 1000f)
            manager.addSnapshot(manager.getSnapshot(gs))
            manager.snapshotCount() shouldBe 1
            shouldThrow<IllegalArgumentException> {
                manager.restoreSnapshot(2, gs)
            }
            manager.snapshotCount() shouldBe 1
        }

        test("RLStateRestoreManager restoreSnapshot throws when stepsAgo is zero or negative") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val manager = RLStateRestoreManager(3)
            addTestAircraft(gs, "MGR05", 0f, 0f, 1000f)
            manager.addSnapshot(manager.getSnapshot(gs))
            shouldThrow<IllegalArgumentException> {
                manager.restoreSnapshot(0, gs)
            }
            shouldThrow<IllegalArgumentException> {
                manager.restoreSnapshot(-1, gs)
            }
        }

        test("RunwayOccupied is restored when runway was occupied at snapshot time") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            ensureAirportWithRunway(gs)
            val arpt0 = 0.toByte()
            val rwy = gs.airports[arpt0]!!.entity[RunwayChildren.mapper]!!.rwyMap[arpt0]!!
            rwy.entity.plusAssign(RunwayOccupied())
            val snapshot = buildSnapshot(gs)
            rwy.entity.remove<RunwayOccupied>()
            restoreSnapshot(snapshot, gs)
            val rwyAfter = gs.airports[arpt0]!!.entity[RunwayChildren.mapper]!!.rwyMap[arpt0]!!
            rwyAfter.entity.has(RunwayOccupied.mapper) shouldBe true
        }

        test("RunwayOccupied is cleared when runway was not occupied at snapshot time") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            ensureAirportWithRunway(gs)
            val arpt0 = 0.toByte()
            val rwy = gs.airports[arpt0]!!.entity[RunwayChildren.mapper]!!.rwyMap[arpt0]!!
            val snapshot = buildSnapshot(gs)
            rwy.entity.plusAssign(RunwayOccupied())
            restoreSnapshot(snapshot, gs)
            val rwyAfter = gs.airports[arpt0]!!.entity[RunwayChildren.mapper]!!.rwyMap[arpt0]!!
            rwyAfter.entity.has(RunwayOccupied.mapper) shouldBe false
        }

        test("LocalizerArmed is restored when aircraft had it at snapshot time") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            ensureAirportWithRunway(gs)
            val appEntity = ensureAirportWithIlsApproach(gs)
            val ac = addTestAircraft(gs, "LOC01", 0f, 0f, 3000f)
            ac.entity.plusAssign(ArrivalAirport(0))
            ac.entity.plusAssign(LocalizerArmed(appEntity))
            val snapshot = buildSnapshot(gs)
            ac.entity.remove<LocalizerArmed>()
            restoreSnapshot(snapshot, gs)
            val acAfter = gs.aircraft.get("LOC01")!!.entity
            acAfter.has(LocalizerArmed.mapper) shouldBe true
        }

        test("LocalizerArmed is cleared when aircraft did not have it at snapshot time") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            ensureAirportWithRunway(gs)
            val appEntity = ensureAirportWithIlsApproach(gs)
            val ac = addTestAircraft(gs, "LOC02", 0f, 0f, 3000f)
            ac.entity.plusAssign(ArrivalAirport(0))
            val snapshot = buildSnapshot(gs)
            ac.entity.plusAssign(LocalizerArmed(appEntity))
            restoreSnapshot(snapshot, gs)
            val acAfter = gs.aircraft.get("LOC02")!!.entity
            acAfter.has(LocalizerArmed.mapper) shouldBe false
        }

        test("DecelerateTo240kts, AppDecelerateTo190kts, DecelerateToAppSpd are restored when present at snapshot time") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val ac = addTestAircraft(gs, "SPD01", 0f, 0f, 8000f)
            ac.entity.plusAssign(DecelerateTo240kts())
            ac.entity.plusAssign(AppDecelerateTo190kts())
            ac.entity.plusAssign(DecelerateToAppSpd())
            val snapshot = buildSnapshot(gs)
            ac.entity.remove<DecelerateTo240kts>()
            ac.entity.remove<AppDecelerateTo190kts>()
            ac.entity.remove<DecelerateToAppSpd>()
            restoreSnapshot(snapshot, gs)
            val acAfter = gs.aircraft.get("SPD01")!!.entity
            acAfter.has(DecelerateTo240kts.mapper) shouldBe true
            acAfter.has(AppDecelerateTo190kts.mapper) shouldBe true
            acAfter.has(DecelerateToAppSpd.mapper) shouldBe true
        }

        test("DecelerateTo240kts, AppDecelerateTo190kts, DecelerateToAppSpd are cleared when not present at snapshot time") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            val ac = addTestAircraft(gs, "SPD02", 0f, 0f, 8000f)
            val snapshot = buildSnapshot(gs)
            ac.entity.plusAssign(DecelerateTo240kts())
            ac.entity.plusAssign(AppDecelerateTo190kts())
            ac.entity.plusAssign(DecelerateToAppSpd())
            restoreSnapshot(snapshot, gs)
            val acAfter = gs.aircraft.get("SPD02")!!.entity
            acAfter.has(DecelerateTo240kts.mapper) shouldBe false
            acAfter.has(AppDecelerateTo190kts.mapper) shouldBe false
            acAfter.has(DecelerateToAppSpd.mapper) shouldBe false
        }

        test("Arrival spawn timer is restored per airport") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            ensureAirportWithRunway(gs)
            val arpt0 = 0.toByte()
            val airport = gs.airports[arpt0]!!
            val stats = airport.entity[AirportArrivalStats.mapper]!!
            stats.arrivalSpawnTimer = 42.5f
            stats.previousArrivalSpawnOffsetS = 1.2f
            val snapshot = buildSnapshot(gs)
            stats.arrivalSpawnTimer = 99f
            stats.previousArrivalSpawnOffsetS = 99f
            restoreSnapshot(snapshot, gs)
            val statsAfter = gs.airports[arpt0]!!.entity[AirportArrivalStats.mapper]!!
            statsAfter.arrivalSpawnTimer shouldBe 42.5f
            statsAfter.previousArrivalSpawnOffsetS shouldBe 1.2f
        }

        test("Snapshot copy preserves bridge and reward state") {
            val gs = GAME.gameServer.shouldNotBeNull()
            ensureTrafficSystem(gs)
            addTestAircraft(gs, "BR01", 0f, 0f, 5000f)
            val baseSnapshot = buildSnapshot(gs)
            val rewardState = RewardHandlerSnapshotData(
                acPrevLocDistPx = Array(MAX_RL_AIRCRAFT) { if (it == 0) 100f else null },
                acPrevAlt = Array(MAX_RL_AIRCRAFT) { if (it == 0) 4000f else null },
                acPrevClearance = Array(MAX_RL_AIRCRAFT) { null },
                mvaConflictCount = 2,
                aircraftConflictCountNoLoc = 1,
                aircraftConflictCountLoc = 2,
                wakeConflictCountNoLoc = 3,
                wakeConflictCountLoc = 4,
            )
            val fullSnapshot = baseSnapshot.copy(
                bridgeSpawnedInSession = 5,
                bridgeLandedInSession = 2,
                bridgeAddedInSession = 5,
                bridgeAgentCallsigns = listOf("BR01", null, null),
                rewardHandlerState = rewardState,
                actions = mapOf()
            )
            fullSnapshot.bridgeSpawnedInSession shouldBe 5
            fullSnapshot.bridgeLandedInSession shouldBe 2
            fullSnapshot.bridgeAddedInSession shouldBe 5
            fullSnapshot.bridgeAgentCallsigns shouldBe listOf("BR01", null, null)
            val restoredReward = fullSnapshot.rewardHandlerState!!
            restoredReward.mvaConflictCount shouldBe 2
            restoredReward.aircraftConflictCountNoLoc shouldBe 1
            restoredReward.aircraftConflictCountLoc shouldBe 2
            restoredReward.wakeConflictCountNoLoc shouldBe 3
            restoredReward.wakeConflictCountLoc shouldBe 4
            restoredReward.acPrevLocDistPx[0] shouldBe 100f
            restoredReward.acPrevAlt[0] shouldBe 4000f
        }

        test("RewardHandler getStateForSnapshot and applyState round-trip") {
            val conflictManager = ConflictManager()
            val handler = RewardHandler(
                conflictManager,
                RLHeadlessTrainingConfig(
                    envId = "test",
                    evalMode = false,
                    goalReward = 1f,
                    mvaConflictPenalty = 0.5f,
                    aircraftConflictPenalty = 0.5f,
                    wakeConflictPenalty = 0.5f,
                    randomSpawnChance = 0f,
                    scriptedSpawnFile = null,
                ),
            )
            val state = RewardHandlerSnapshotData(
                acPrevLocDistPx = Array(MAX_RL_AIRCRAFT) { if (it == 0) 200f else null },
                acPrevAlt = Array(MAX_RL_AIRCRAFT) { if (it == 0) 6000f else null },
                acPrevClearance = Array(MAX_RL_AIRCRAFT) { i ->
                    if (i == 0) ClearanceState().apply { clearedAlt = 7000; clearedIas = 250 } else null
                },
                mvaConflictCount = 4,
                aircraftConflictCountNoLoc = 5,
                aircraftConflictCountLoc = 3,
                wakeConflictCountNoLoc = 6,
                wakeConflictCountLoc = 7,
            )
            handler.rewardReset()
            handler.applyState(state)
            val restored = handler.getStateForSnapshot()
            restored.mvaConflictCount shouldBe 4
            restored.aircraftConflictCountNoLoc shouldBe 5
            restored.aircraftConflictCountLoc shouldBe 3
            restored.wakeConflictCountNoLoc shouldBe 6
            restored.wakeConflictCountLoc shouldBe 7
            restored.acPrevLocDistPx[0] shouldBe 200f
            restored.acPrevAlt[0] shouldBe 6000f
            restored.acPrevClearance[0]!!.clearedAlt shouldBe 7000
            restored.acPrevClearance[0]!!.clearedIas shouldBe 250
        }

        test("PythonGymnasiumBridge option generation correctly orders options by distance from original") {
            // Heading options should be sorted by absolute distance from original
            PythonGymnasiumBridge.getHeadingOptions(2).toList() shouldBe listOf(1, 3, 0, 4) // 1 and 3 are dist 1; 0 and 4 are dist 2. Order of ties doesn't strictly matter as long as dist is ascending
            PythonGymnasiumBridge.getHeadingOptions(0).toList() shouldBe listOf(1, 2, 3, 4)
            PythonGymnasiumBridge.getHeadingOptions(4).toList() shouldBe listOf(3, 2, 1, 0)
            
            // Wake IAS options should only include values < original, sorted descending (closest to original first)
            PythonGymnasiumBridge.getIasOptions(2).toList() shouldBe listOf(1, 0)
            PythonGymnasiumBridge.getIasOptions(4).toList() shouldBe listOf(3, 2, 1, 0)
            PythonGymnasiumBridge.getIasOptions(0).toList() shouldBe emptyList<Int>()
        }
    }

    private fun clearAllAircraft(gs: GameServer) {
        val toDespawn = mutableListOf<com.badlogic.ashley.core.Entity>()
        for (i in 0 until gs.aircraft.size) {
            gs.aircraft.getValueAt(i)?.entity?.let { toDespawn.add(it) }
        }
        toDespawn.forEach { despawnAircraft(it) }
    }

    private fun clearAllRunwayOccupied(gs: GameServer) {
        for (arptEntry in Entries(gs.airports)) {
            val airport = arptEntry.value
            val rwyChildren = airport.entity[RunwayChildren.mapper] ?: continue
            for (rwyEntry in Entries(rwyChildren.rwyMap)) {
                val rwy = rwyEntry.value
                val rwyEntity = rwy.entity
                if (rwyEntity.has(RunwayOccupied.mapper)) {
                    rwyEntity.remove<RunwayOccupied>()
                    rwyEntity[OppositeRunway.mapper]?.oppRwy?.remove<RunwayOccupied>()
                }
            }
        }
    }

    private fun ensureTrafficSystem(gs: GameServer) {
        gs.engine.addSystem(TrafficSystemInterval(ArrivalsToControlSpawner()))
    }

    private fun ensureAirportWithRunway(gs: GameServer) {
        val arpt0 = 0.toByte()
        val airport = gs.airports[arpt0]
        if (airport != null && airport.entity[RunwayChildren.mapper]?.rwyMap?.containsKey(arpt0) == true) return
        
        val newAirport = Airport(0, "TST", "Test", 1, 0, 0f, 0f, 0, "XXXX", false).also {
            it.addRunway(0, "36", 0f, 0f, 270f, 3500, 0, 0, 0, "", "", RunwayLabel.BEFORE)
        }
        gs.airports.put(arpt0, newAirport)
    }

    /** Ensures airport has an ILS approach with localizer; returns the approach entity for LocalizerArmed. */
    private fun ensureAirportWithIlsApproach(gs: GameServer): com.badlogic.ashley.core.Entity {
        ensureAirportWithRunway(gs)
        val arpt0 = 0.toByte()
        val airport = gs.airports[arpt0]!!
        val approachChildren = airport.entity[ApproachChildren.mapper]!!
        val existing = approachChildren.approachMap["ILS 36"]
        if (existing != null) return existing.entity
        val approach = Approach("ILS 36", 0, 0, 0f, 0f, 200, 1200, false, 0)
        approach.addLocalizer(360f, 20)
        approachChildren.approachMap.put("ILS 36", approach)
        return approach.entity
    }

    private fun addTestAircraft(gs: GameServer, callsign: String, x: Float, y: Float, alt: Float): Aircraft {
        val ac = Aircraft(callsign, x, y, alt, "B738", FlightType.ARRIVAL, false)
        ac.entity.plusAssign(ClearanceAct(ClearanceState().ActingClearance()))
        ac.entity.plusAssign(SpawnGroup(0))
        gs.aircraft.put(callsign, ac)
        return ac
    }
}
