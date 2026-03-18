package com.bombbird.terminalcontrol2.gymnasium.staterestore

import com.bombbird.terminalcontrol2.components.*
import com.bombbird.terminalcontrol2.navigation.ClearanceState
import com.bombbird.terminalcontrol2.navigation.Route

/**
 * Snapshot of game state for RL conflict resolution rollback.
 * Holds deep copies of all aircraft and their wake data; runway occupied state; no entity references.
 * Optional bridge/reward/traffic state for use when snapshot is taken from PythonGymnasiumBridge.
 */
data class Snapshot(
    val aircraft: Map<String, AircraftSnapshotData>,
    /** Monotonically increasing snapshot timestep within a rollout (managed by RLStateRestoreManager). */
    val timestep: Int = 0,
    /** Set of (arptId, rwyId) for runways that had RunwayOccupied at snapshot time. */
    val runwayOccupied: Set<Pair<Byte, Byte>> = emptySet(),
    /** Per-airport (arptId) arrival spawn timer and previous offset (arrivalSpawnTimer, previousArrivalSpawnOffsetS). */
    val arrivalSpawnTimers: Map<Byte, Pair<Float, Float>> = emptyMap(),
    val bridgeSpawnedInSession: Int? = null,
    val bridgeLandedInSession: Int? = null,
    val bridgeAddedInSession: Int? = null,
    val bridgeAgentCallsigns: List<String?>? = null,
    val rewardHandlerState: RewardHandlerSnapshotData? = null,
    val actions: Map<String, IntArray> = emptyMap()
) {
    fun callsigns(): Set<String> = aircraft.keys
}

/** Snapshot of RewardHandler state for restore (prev LOC dist, prev alt, prev clearance, conflict counts). */
data class RewardHandlerSnapshotData(
    val acPrevLocDistPx: Array<Float?>,
    val acPrevAlt: Array<Float?>,
    val acPrevClearance: Array<ClearanceState?>,
    val mvaConflictCount: Int,
    val aircraftConflictCountNoLoc: Int,
    val aircraftConflictCountLoc: Int,
    val wakeConflictCountNoLoc: Int,
    val wakeConflictCountLoc: Int
)

/**
 * Data required to recreate one aircraft's state (components + wake trail).
 * Approach/runway refs stored as (arptId, approachName, rwyId) and resolved on restore.
 * Route zones (ArrivalRouteZone/DepartureRouteZone) are not stored; re-created aircraft get empty zones.
 */
data class AircraftSnapshotData(
    val position: Position,
    val altitude: Altitude,
    val speed: Speed,
    val direction: Direction,
    val groundTrack: GroundTrack,
    val indicatedAirSpeed: IndicatedAirSpeed,
    val aircraftInfo: AircraftInfo,
    val clearanceState: ClearanceState,
    val pendingClearances: List<Pair<Float, ClearanceState>>,
    val commandTarget: CommandTarget,
    val acceleration: Acceleration,
    val flightType: FlightType,
    val arrivalAirport: ArrivalAirport?,
    val departureAirport: DepartureAirport?,
    val lastRestrictions: LastRestrictions?,
    val wakeTolerance: WakeTolerance,
    val wakeTrail: WakeTrailSnapshotData,
    val hasLocalizerArmed: Boolean,
    val hasLocalizerCaptured: Boolean,
    val hasGlideSlopeCaptured: Boolean,
    val hasVisualCaptured: Boolean,
    val approachRefArptId: Byte?,
    val approachRefName: String?,
    val approachRefRwyId: Byte?,
    val hasLandingRoll: Boolean,
    val recentGoAround: RecentGoAround?,
    val divergentDepartureAllowed: DivergentDepartureAllowed?,
    val hasTakeoffClimb: Boolean,
    val emergencyPending: EmergencyPending?,
    val circlingApproachPhase: Int?,
    val hasStepDownApproach: Boolean,
    val hasGlideSlopeArmed: Boolean,
    val hasDecelerateTo240kts: Boolean,
    val hasAppDecelerateTo190kts: Boolean,
    val hasDecelerateToAppSpd: Boolean
)

/** Snapshot of WakeTrail: distNmCounter and list of (position, optional wake zone params). */
data class WakeTrailSnapshotData(
    val distNmCounter: Float,
    val points: List<Pair<Position, WakeZoneSnapshotData?>>
)

/** Data to recreate a WakeZone (constructor params + distFromAircraft). */
data class WakeZoneSnapshotData(
    val prevPosX: Float,
    val prevPosY: Float,
    val currPosX: Float,
    val currPosY: Float,
    val wakeAlt: Float,
    val callsign: String,
    val leadingWake: Char,
    val leadingRecat: Char,
    val approachAirportId: Byte?,
    val approachName: String?,
    val distFromAircraft: Float
)

/** Deep copy of ClearanceState (routes copied). */
fun copyClearanceState(c: ClearanceState): ClearanceState = ClearanceState(
    c.routePrimaryName,
    Route().apply { setToRouteCopy(c.route) },
    Route().apply { setToRouteCopy(c.hiddenLegs) },
    c.vectorHdg, c.vectorTurnDir, c.clearedAlt, c.expedite, c.clearedIas, c.minIas, c.maxIas, c.optimalIas,
    c.clearedApp, c.clearedTrans, c.cancelLastMaxSpd, c.initiateGoAround
)
