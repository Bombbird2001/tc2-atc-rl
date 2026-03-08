package com.bombbird.terminalcontrol2.gymnasium.staterestore

import com.badlogic.gdx.math.Vector2
import com.badlogic.gdx.utils.ArrayMap.Entries
import com.badlogic.gdx.utils.Queue
import com.bombbird.terminalcontrol2.components.*
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.entities.WakeZone
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.components.ApproachChildren
import com.bombbird.terminalcontrol2.components.RunwayChildren
import com.bombbird.terminalcontrol2.components.VisualApproach
import com.bombbird.terminalcontrol2.components.OppositeRunway
import com.bombbird.terminalcontrol2.navigation.ClearanceState
import com.bombbird.terminalcontrol2.navigation.Route
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.systems.TrafficSystemInterval
import com.bombbird.terminalcontrol2.traffic.despawnAircraft
import ktx.ashley.get
import ktx.ashley.getSystem
import ktx.ashley.has
import ktx.ashley.plusAssign
import ktx.ashley.remove

/**
 * Restores [gs] to the state captured in [snapshot].
 * Same entity when possible (overwrite components in place); re-creates despawned aircraft; despawns aircraft not in snapshot.
 */
fun restoreSnapshot(snapshot: Snapshot, gs: GameServer) {
    val trafficSystem = gs.engine.getSystem<TrafficSystemInterval>()
    val snapshotCallsigns = snapshot.callsigns()

    // 1. Despawn aircraft that are not in the snapshot (spawned after snapshot time)
    val toDespawn = mutableListOf<com.badlogic.ashley.core.Entity>()
    for (i in 0 until gs.aircraft.size) {
        val ac = gs.aircraft.getValueAt(i) ?: continue
        val callsign = ac.entity[AircraftInfo.mapper]?.icaoCallsign ?: continue
        if (callsign !in snapshotCallsigns) toDespawn.add(ac.entity)
    }
    toDespawn.forEach { despawnAircraft(it) }

    // 2. For each aircraft in snapshot: remove its wake zones (we will re-add from snapshot)
    for (callsign in snapshotCallsigns) {
        val ac = gs.aircraft.get(callsign) ?: continue
        trafficSystem.removeAircraftWakeZones(ac.entity)
    }

    // 3. For each aircraft in snapshot: overwrite in place or create new entity
    for ((callsign, data) in snapshot.aircraft) {
        val existing = gs.aircraft.get(callsign)
        if (existing != null) {
            applyAircraftSnapshotData(existing.entity, data)
        } else {
            val ac = Aircraft(
                callsign,
                data.position.x,
                data.position.y,
                data.altitude.altitudeFt,
                data.aircraftInfo.icaoType,
                data.flightType.type,
                false
            )
            gs.aircraft.put(callsign, ac)
            applyAircraftSnapshotData(ac.entity, data)
            resolveApproachRefs(ac.entity, data)
        }
    }

    // 4. Rebuild wake trails: create WakeZone entities and add to traffic system, set WakeTrail on each aircraft
    for ((callsign, data) in snapshot.aircraft) {
        val ac = gs.aircraft.get(callsign) ?: continue
        val e = ac.entity
        val wakeTrail = e[WakeTrail.mapper] ?: continue
        wakeTrail.wakeZones.clear()
        wakeTrail.distNmCounter = data.wakeTrail.distNmCounter
        for ((pos, zoneData) in data.wakeTrail.points) {
            val wz = zoneData?.let { zd ->
                WakeZone(
                    zd.prevPosX, zd.prevPosY, zd.currPosX, zd.currPosY,
                    zd.wakeAlt, zd.callsign, zd.leadingWake, zd.leadingRecat,
                    zd.approachAirportId, zd.approachName
                ).also { zone ->
                    zone.entity[WakeInfo.mapper]?.distFromAircraft = zd.distFromAircraft
                }
            }
            wakeTrail.wakeZones.addLast(Position(pos.x, pos.y) to wz)
            wz?.let { trafficSystem.addWakeZone(it) }
        }
    }

    // 5. Resolve approach refs for existing aircraft (in case we overwrote components that had refs)
    for ((callsign, data) in snapshot.aircraft) {
        val ac = gs.aircraft.get(callsign) ?: continue
        resolveApproachRefs(ac.entity, data)
    }

    // 6. Restore RunwayOccupied on runways to match snapshot
    restoreRunwayOccupied(snapshot.runwayOccupied, gs)

    // 7. Restore arrival spawn timers per airport
    for ((arptId, pair) in snapshot.arrivalSpawnTimers) {
        val airport = gs.airports.get(arptId) ?: continue
        val stats = airport.entity[AirportArrivalStats.mapper] ?: continue
        stats.arrivalSpawnTimer = pair.first
        stats.previousArrivalSpawnOffsetS = pair.second
    }
}

private fun restoreRunwayOccupied(runwayOccupied: Set<Pair<Byte, Byte>>, gs: GameServer) {
    for (arptEntry in Entries(gs.airports)) {
        val arptId = arptEntry.key
        val airport = arptEntry.value
        val rwyChildren = airport.entity[RunwayChildren.mapper] ?: continue
        for (rwyEntry in Entries(rwyChildren.rwyMap)) {
            val rwyId = rwyEntry.key
            val rwy = rwyEntry.value
            val rwyEntity = rwy.entity
            val key = arptId to rwyId
            val shouldBeOccupied = key in runwayOccupied
            val hasOccupied = rwyEntity.has(RunwayOccupied.mapper)
            if (hasOccupied && !shouldBeOccupied) {
                rwyEntity.remove<RunwayOccupied>()
                rwyEntity[OppositeRunway.mapper]?.oppRwy?.remove<RunwayOccupied>()
            } else if (!hasOccupied && shouldBeOccupied) {
                rwyEntity.plusAssign(RunwayOccupied())
                rwyEntity[OppositeRunway.mapper]?.oppRwy?.plusAssign(RunwayOccupied())
            }
        }
    }
}

private fun applyAircraftSnapshotData(e: com.badlogic.ashley.core.Entity, data: AircraftSnapshotData) {
    e[Position.mapper]?.apply { x = data.position.x; y = data.position.y }
        ?: e.plusAssign(Position(data.position.x, data.position.y))
    e[Altitude.mapper]?.apply { altitudeFt = data.altitude.altitudeFt }
        ?: e.plusAssign(Altitude(data.altitude.altitudeFt))
    e[Speed.mapper]?.apply {
        speedKts = data.speed.speedKts
        vertSpdFpm = data.speed.vertSpdFpm
        angularSpdDps = data.speed.angularSpdDps
    } ?: e.plusAssign(Speed(data.speed.speedKts, data.speed.vertSpdFpm, data.speed.angularSpdDps))
    e[Direction.mapper]?.trackUnitVector?.set(data.direction.trackUnitVector)
        ?: e.plusAssign(Direction(Vector2(data.direction.trackUnitVector)))
    e[GroundTrack.mapper]?.trackVectorPxps?.set(data.groundTrack.trackVectorPxps)
        ?: e.plusAssign(GroundTrack(Vector2(data.groundTrack.trackVectorPxps)))
    e[IndicatedAirSpeed.mapper]?.apply { iasKt = data.indicatedAirSpeed.iasKt }
        ?: e.plusAssign(IndicatedAirSpeed(data.indicatedAirSpeed.iasKt))
    e[AircraftInfo.mapper]?.apply {
        icaoCallsign = data.aircraftInfo.icaoCallsign
        icaoType = data.aircraftInfo.icaoType
        aircraftPerf = data.aircraftInfo.aircraftPerf
    }
    e[CommandTarget.mapper]?.apply {
        targetHdgDeg = data.commandTarget.targetHdgDeg
        turnDir = data.commandTarget.turnDir
        targetAltFt = data.commandTarget.targetAltFt
        targetIasKt = data.commandTarget.targetIasKt
    } ?: e.plusAssign(CommandTarget(data.commandTarget.targetHdgDeg, data.commandTarget.turnDir, data.commandTarget.targetAltFt, data.commandTarget.targetIasKt))
    e[Acceleration.mapper]?.apply {
        dSpeedMps2 = data.acceleration.dSpeedMps2
        dVertSpdMps2 = data.acceleration.dVertSpdMps2
        dAngularSpdDps2 = data.acceleration.dAngularSpdDps2
    } ?: e.plusAssign(Acceleration(data.acceleration.dSpeedMps2, data.acceleration.dVertSpdMps2, data.acceleration.dAngularSpdDps2))
    e[FlightType.mapper]?.apply { type = data.flightType.type } ?: e.plusAssign(FlightType(data.flightType.type))
    e[WakeTolerance.mapper]?.apply { accumulation = data.wakeTolerance.accumulation }
        ?: e.plusAssign(WakeTolerance(data.wakeTolerance.accumulation))
    data.arrivalAirport?.let { e.remove<ArrivalAirport>(); e.plusAssign(ArrivalAirport(it.arptId)) }
        ?: e.remove<ArrivalAirport>()
    data.departureAirport?.let { e.remove<DepartureAirport>(); e.plusAssign(DepartureAirport(it.arptId, it.rwyId)) }
        ?: e.remove<DepartureAirport>()
    data.lastRestrictions?.let { r ->
        (e[LastRestrictions.mapper] ?: LastRestrictions().also { e.plusAssign(it) }).apply {
            minAltFt = r.minAltFt
            maxAltFt = r.maxAltFt
            maxSpdKt = r.maxSpdKt
        }
    }
    e[ClearanceAct.mapper]?.actingClearance?.clearanceState?.apply {
        routePrimaryName = data.clearanceState.routePrimaryName
        route.setToRouteCopy(data.clearanceState.route)
        hiddenLegs.setToRouteCopy(data.clearanceState.hiddenLegs)
        vectorHdg = data.clearanceState.vectorHdg
        vectorTurnDir = data.clearanceState.vectorTurnDir
        clearedAlt = data.clearanceState.clearedAlt
        expedite = data.clearanceState.expedite
        clearedIas = data.clearanceState.clearedIas
        minIas = data.clearanceState.minIas
        maxIas = data.clearanceState.maxIas
        optimalIas = data.clearanceState.optimalIas
        clearedApp = data.clearanceState.clearedApp
        clearedTrans = data.clearanceState.clearedTrans
        cancelLastMaxSpd = data.clearanceState.cancelLastMaxSpd
        initiateGoAround = data.clearanceState.initiateGoAround
    } ?: e.plusAssign(ClearanceAct(ClearanceState(
        data.clearanceState.routePrimaryName,
        Route().apply { setToRouteCopy(data.clearanceState.route) },
        Route().apply { setToRouteCopy(data.clearanceState.hiddenLegs) },
        data.clearanceState.vectorHdg, data.clearanceState.vectorTurnDir,
        data.clearanceState.clearedAlt, data.clearanceState.expedite, data.clearanceState.clearedIas,
        data.clearanceState.minIas, data.clearanceState.maxIas, data.clearanceState.optimalIas,
        data.clearanceState.clearedApp, data.clearanceState.clearedTrans,
        data.clearanceState.cancelLastMaxSpd, data.clearanceState.initiateGoAround
    ).ActingClearance()))
    e[PendingClearances.mapper]?.clearanceQueue?.clear()
    if (data.pendingClearances.isNotEmpty()) {
        val q = e[PendingClearances.mapper]?.clearanceQueue ?: Queue<ClearanceState.PendingClearanceState>().also { e.plusAssign(PendingClearances(it)) }
        q.clear()
        for ((timeLeft, state) in data.pendingClearances) {
            q.addLast(ClearanceState.PendingClearanceState(timeLeft, copyClearanceState(state)))
        }
    } else e.remove<PendingClearances>()
    if (data.hasLandingRoll) e.plusAssign(LandingRoll()) else e.remove<LandingRoll>()
    data.recentGoAround?.let { e.remove<RecentGoAround>(); e.plusAssign(RecentGoAround(it.timeLeft, it.reason)) }
        ?: e.remove<RecentGoAround>()
    data.divergentDepartureAllowed?.let { e.remove<DivergentDepartureAllowed>(); e.plusAssign(DivergentDepartureAllowed(it.timeLeft)) }
        ?: e.remove<DivergentDepartureAllowed>()
    if (data.hasTakeoffClimb) e.plusAssign(TakeoffClimb()) else e.remove<TakeoffClimb>()
    data.emergencyPending?.let { e.remove<EmergencyPending>(); e.plusAssign(EmergencyPending(it.active, it.type, it.activationAlt)) }
        ?: e.remove<EmergencyPending>()
    if (data.circlingApproachPhase != null) {
        e.remove<CirclingApproach>()
        e.plusAssign(CirclingApproach(com.badlogic.ashley.core.Entity(), 0, data.circlingApproachPhase.toByte()))
    } else e.remove<CirclingApproach>()
    if (data.hasStepDownApproach) e.plusAssign(StepDownApproach(com.badlogic.ashley.core.Entity()))
    else e.remove<StepDownApproach>()
    if (!data.hasGlideSlopeArmed) e.remove<GlideSlopeArmed>()
    if (!data.hasLocalizerArmed) e.remove<LocalizerArmed>()
    if (!data.hasLocalizerCaptured) e.remove<LocalizerCaptured>()
    if (!data.hasGlideSlopeCaptured) e.remove<GlideSlopeCaptured>()
    if (!data.hasVisualCaptured) e.remove<VisualCaptured>()
    if (data.hasDecelerateTo240kts) e.plusAssign(DecelerateTo240kts()) else e.remove<DecelerateTo240kts>()
    if (data.hasAppDecelerateTo190kts) e.plusAssign(AppDecelerateTo190kts()) else e.remove<AppDecelerateTo190kts>()
    if (data.hasDecelerateToAppSpd) e.plusAssign(DecelerateToAppSpd()) else e.remove<DecelerateToAppSpd>()
}

private fun resolveApproachRefs(e: com.badlogic.ashley.core.Entity, data: AircraftSnapshotData) {
    val arptId = data.approachRefArptId ?: return
    val appName = data.approachRefName ?: return
    val arpt = GAME.gameServer?.airports?.get(arptId)?.entity ?: return
    val rwyId = data.approachRefRwyId
    val appEntity = arpt[ApproachChildren.mapper]?.approachMap?.get(appName)?.entity
        ?: arpt[RunwayChildren.mapper]?.rwyMap?.get(rwyId)?.entity?.get(VisualApproach.mapper)?.visual
        ?: return
    if (data.hasLocalizerCaptured) {
        e.remove<LocalizerCaptured>()
        e.plusAssign(LocalizerCaptured(appEntity))
    }
    if (data.hasGlideSlopeCaptured) {
        e.remove<GlideSlopeCaptured>()
        e.plusAssign(GlideSlopeCaptured(appEntity))
    }
    if (data.hasVisualCaptured) {
        e.remove<VisualCaptured>()
        e.plusAssign(VisualCaptured(appEntity))
    }
    if (data.hasGlideSlopeArmed) {
        e.remove<GlideSlopeArmed>()
        e.plusAssign(GlideSlopeArmed(appEntity))
    }
    if (data.hasLocalizerArmed) {
        e.remove<LocalizerArmed>()
        e.plusAssign(LocalizerArmed(appEntity))
    }
    if (data.hasStepDownApproach) {
        e.remove<StepDownApproach>()
        e.plusAssign(StepDownApproach(appEntity))
    }
    data.circlingApproachPhase?.let { phase ->
        e[CirclingApproach.mapper]?.let { cir ->
            cir.circlingApp = appEntity
            cir.phase = phase.toByte()
        }
    }
}
