package com.bombbird.terminalcontrol2.gymnasium.staterestore

import com.badlogic.ashley.core.Entity
import com.badlogic.gdx.math.Vector2
import com.badlogic.gdx.utils.ArrayMap.Entries
import com.badlogic.gdx.utils.Queue.QueueIterator
import com.bombbird.terminalcontrol2.components.*
import com.bombbird.terminalcontrol2.entities.WakeZone
import com.bombbird.terminalcontrol2.navigation.ClearanceState
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.systems.TrafficSystemInterval
import ktx.ashley.get
import ktx.ashley.has

/**
 * Builds a [Snapshot] from the current [GameServer] state (all aircraft in gs.aircraft).
 * Uses deep copy of components; no JSON. Approach refs stored as (arptId, approachName, rwyId).
 */
fun buildSnapshot(gs: GameServer): Snapshot {
    val aircraftMap = mutableMapOf<String, AircraftSnapshotData>()
    for (i in 0 until gs.aircraft.size) {
        val ac = gs.aircraft.getValueAt(i) ?: continue
        val e = ac.entity
        val callsign = e[AircraftInfo.mapper]?.icaoCallsign ?: continue
        aircraftMap[callsign] = buildAircraftSnapshotData(e)
    }
    val runwayOccupied = buildRunwayOccupiedSet(gs)
    val arrivalSpawnTimers = buildArrivalSpawnTimers(gs)
    val spawnHandlerState = gs.engine.getSystem(TrafficSystemInterval::class.java)?.arrivalsToControlSpawner?.getStateForSnapshot()
    return Snapshot(
        aircraft = aircraftMap,
        runwayOccupied = runwayOccupied,
        arrivalSpawnTimers = arrivalSpawnTimers,
        spawnHandlerState = spawnHandlerState
    )
}

private fun buildArrivalSpawnTimers(gs: GameServer): Map<Byte, Pair<Float, Float>> {
    val map = mutableMapOf<Byte, Pair<Float, Float>>()
    for (arptEntry in Entries(gs.airports)) {
        val arptId = arptEntry.key
        val airport = arptEntry.value
        val stats = airport.entity[AirportArrivalStats.mapper] ?: continue
        map[arptId] = stats.arrivalSpawnTimer to stats.previousArrivalSpawnOffsetS
    }
    return map
}

private fun buildRunwayOccupiedSet(gs: GameServer): Set<Pair<Byte, Byte>> {
    val set = mutableSetOf<Pair<Byte, Byte>>()
    for (arptEntry in Entries(gs.airports)) {
        val arptId = arptEntry.key
        val airport = arptEntry.value
        val rwyChildren = airport.entity[RunwayChildren.mapper] ?: continue
        for (rwyEntry in Entries(rwyChildren.rwyMap)) {
            val rwyId = rwyEntry.key
            val rwy = rwyEntry.value
            if (rwy.entity.has(RunwayOccupied.mapper)) set.add(arptId to rwyId)
        }
    }
    return set
}

private fun buildAircraftSnapshotData(e: Entity): AircraftSnapshotData {
    val pos = e[Position.mapper]!!
    val alt = e[Altitude.mapper]!!
    val speed = e[Speed.mapper]!!
    val dir = e[Direction.mapper]!!
    val gt = e[GroundTrack.mapper]!!
    val ias = e[IndicatedAirSpeed.mapper]!!
    val acInfo = e[AircraftInfo.mapper]!!
    val cmd = e[CommandTarget.mapper]!!
    val acc = e[Acceleration.mapper]!!
    val ft = e[FlightType.mapper]!!
    val wt = e[WakeTolerance.mapper]!!
    val clearanceState = e[ClearanceAct.mapper]?.actingClearance?.clearanceState
        ?: e[PendingClearances.mapper]?.clearanceQueue?.last()?.clearanceState
        ?: ClearanceState()
    val pendingList = e[PendingClearances.mapper]?.clearanceQueue?.let { q ->
        mutableListOf<Pair<Float, ClearanceState>>().apply {
            for (entry in QueueIterator(q)) add(entry.timeLeft to copyClearanceState(entry.clearanceState))
        }
    } ?: emptyList()
    val lastRestr = e[LastRestrictions.mapper]
    val arrArpt = e[ArrivalAirport.mapper]
    val depArpt = e[DepartureAirport.mapper]
    val wakeTrail = e[WakeTrail.mapper]!!
    val wakeList = mutableListOf<Pair<Position, WakeZone?>>()
    for (point in QueueIterator(wakeTrail.wakeZones)) wakeList.add(Position(point.first.x, point.first.y) to point.second)
    val wakePoints: List<Pair<Position, WakeZoneSnapshotData?>> = wakeList.mapIndexed { idx, pair ->
        val posCopy = pair.first
        val zone = pair.second
        val zoneData = zone?.let { wz ->
            val wi = wz.entity[WakeInfo.mapper]!!
            val wa = wz.entity[Altitude.mapper]!!
            val wp = wz.entity[Position.mapper]
            val nextPos = wakeList.getOrNull(idx + 1)?.first
            WakeZoneSnapshotData(
                prevPosX = nextPos?.x ?: posCopy.x,
                prevPosY = nextPos?.y ?: posCopy.y,
                currPosX = wp?.x ?: posCopy.x,
                currPosY = wp?.y ?: posCopy.y,
                wakeAlt = wa.altitudeFt,
                callsign = wi.aircraftCallsign,
                leadingWake = wi.leadingWake,
                leadingRecat = wi.leadingRecat,
                approachAirportId = wi.approachAirportId,
                approachName = wi.approachName,
                distFromAircraft = wi.distFromAircraft
            )
        }
        posCopy to zoneData
    }
    val locApp = e[LocalizerArmed.mapper]?.locApp ?: e[LocalizerCaptured.mapper]?.locApp
    val gsApp = e[GlideSlopeArmed.mapper]?.gsApp ?: e[GlideSlopeCaptured.mapper]?.gsApp
    val visApp = e[VisualCaptured.mapper]?.parentApp
    val stepDownApp = e[StepDownApproach.mapper]?.stepDownApp
    val circlingApp = e[CirclingApproach.mapper]?.circlingApp
    val approachInfo = locApp?.get(ApproachInfo.mapper) ?: gsApp?.get(ApproachInfo.mapper) ?: visApp?.get(ApproachInfo.mapper) ?: stepDownApp?.get(ApproachInfo.mapper) ?: circlingApp?.get(ApproachInfo.mapper)
    val arptId = approachInfo?.airportId
    val appName = approachInfo?.approachName
    val rwyId = approachInfo?.rwyId
    val spawnInfo = e[SpawnGroup.mapper]!!
    return AircraftSnapshotData(
        position = Position(pos.x, pos.y),
        altitude = Altitude(alt.altitudeFt),
        speed = Speed(speed.speedKts, speed.vertSpdFpm, speed.angularSpdDps),
        direction = Direction(Vector2(dir.trackUnitVector)),
        groundTrack = GroundTrack(Vector2(gt.trackVectorPxps)),
        indicatedAirSpeed = IndicatedAirSpeed(ias.iasKt),
        aircraftInfo = AircraftInfo(acInfo.icaoCallsign, acInfo.icaoType).apply { aircraftPerf = acInfo.aircraftPerf },
        clearanceState = copyClearanceState(clearanceState),
        pendingClearances = pendingList,
        commandTarget = CommandTarget(cmd.targetHdgDeg, cmd.turnDir, cmd.targetAltFt, cmd.targetIasKt),
        acceleration = Acceleration(acc.dSpeedMps2, acc.dVertSpdMps2, acc.dAngularSpdDps2),
        flightType = FlightType(ft.type),
        arrivalAirport = arrArpt?.let { ArrivalAirport(it.arptId) },
        departureAirport = depArpt?.let { DepartureAirport(it.arptId, it.rwyId) },
        lastRestrictions = lastRestr?.let { LastRestrictions(it.minAltFt, it.maxAltFt, it.maxSpdKt) },
        wakeTolerance = WakeTolerance(wt.accumulation),
        wakeTrail = WakeTrailSnapshotData(wakeTrail.distNmCounter, wakePoints),
        hasLocalizerArmed = e.has(LocalizerArmed.mapper),
        hasLocalizerCaptured = e.has(LocalizerCaptured.mapper),
        hasGlideSlopeCaptured = e.has(GlideSlopeCaptured.mapper),
        hasVisualCaptured = e.has(VisualCaptured.mapper),
        approachRefArptId = arptId,
        approachRefName = appName,
        approachRefRwyId = rwyId,
        hasLandingRoll = e.has(LandingRoll.mapper),
        recentGoAround = e[RecentGoAround.mapper]?.let { RecentGoAround(it.timeLeft, it.reason) },
        divergentDepartureAllowed = e[DivergentDepartureAllowed.mapper]?.let { DivergentDepartureAllowed(it.timeLeft) },
        hasTakeoffClimb = e.has(TakeoffClimb.mapper),
        emergencyPending = e[EmergencyPending.mapper]?.let { if (it.active) EmergencyPending(it.active, it.type, it.activationAlt) else null },
        circlingApproachPhase = e[CirclingApproach.mapper]?.phase?.toInt(),
        hasStepDownApproach = e.has(StepDownApproach.mapper),
        hasGlideSlopeArmed = e.has(GlideSlopeArmed.mapper),
        hasDecelerateTo240kts = e.has(DecelerateTo240kts.mapper),
        hasAppDecelerateTo190kts = e.has(AppDecelerateTo190kts.mapper),
        hasDecelerateToAppSpd = e.has(DecelerateToAppSpd.mapper),
        spawnGroup = spawnInfo.spawnGroup,
        spawnX = spawnInfo.spawnX,
        spawnY = spawnInfo.spawnY,
        spawnOrder = spawnInfo.spawnOrder,
    )
}
