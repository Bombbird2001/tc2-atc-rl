package com.bombbird.terminalcontrol2.ai.reward

import com.badlogic.ashley.core.Entity
import com.badlogic.gdx.math.MathUtils
import com.bombbird.terminalcontrol2.components.AircraftInfo
import com.bombbird.terminalcontrol2.components.Altitude
import com.bombbird.terminalcontrol2.components.ApproachChildren
import com.bombbird.terminalcontrol2.components.IndicatedAirSpeed
import com.bombbird.terminalcontrol2.components.LandingRoll
import com.bombbird.terminalcontrol2.components.LocalizerCaptured
import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.global.CLEARANCE_CHANGE_PENALTY
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.LOC_PROX_PENALTY
import com.bombbird.terminalcontrol2.global.MAX_RL_AIRCRAFT
import com.bombbird.terminalcontrol2.global.PER_STEP_PENALTY
import com.bombbird.terminalcontrol2.gymnasium.staterestore.RewardHandlerSnapshotData
import com.bombbird.terminalcontrol2.gymnasium.staterestore.copyClearanceState
import com.bombbird.terminalcontrol2.navigation.ClearanceState
import com.bombbird.terminalcontrol2.navigation.distPxFromLoc
import com.bombbird.terminalcontrol2.traffic.conflict.Conflict
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.traffic.conflict.PotentialConflict
import com.bombbird.terminalcontrol2.utilities.byte
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import com.bombbird.terminalcontrol2.utilities.nmToPx
import ktx.ashley.get
import ktx.ashley.has
import ktx.collections.GdxArray
import ktx.collections.GdxArrayMap

class RewardHandler(
    private val conflictManager: ConflictManager, private val eval: Boolean, private val goalReward: Float,
    private val mvaConflictPenalty: Float, private val aircraftConflictPenalty: Float, private val wakeConflictPenalty: Float
) {
    companion object {
        val EMPTY_POTENTIAL_CONFLICTS = GdxArray<PotentialConflict>(1)
    }

    private val acPrevLocDistPx: Array<Float?> = Array(MAX_RL_AIRCRAFT) { null }
    private val acPrevAlt: Array<Float?> = Array(MAX_RL_AIRCRAFT) { null }
    private val acPrevClearance: Array<ClearanceState?> = Array(MAX_RL_AIRCRAFT) { null }
    var mvaConflictCount = 0
        private set
    var aircraftConflictCountNoLoc = 0
        private set
    var aircraftConflictCountLoc = 0
        private set
    var wakeConflictCountNoLoc = 0
        private set
    var wakeConflictCountLoc = 0
        private set

    private val targetApproach = lazy {
        GAME.gameServer?.airports?.get(0)?.entity?.get(ApproachChildren.mapper)?.approachMap?.get("ILS 02L")!!
    }

    fun rewardReset() {
        for (i in 0 until MAX_RL_AIRCRAFT) {
            acPrevLocDistPx[i] = null
            acPrevAlt[i] = null
            acPrevClearance[i] = null
        }
        mvaConflictCount = 0
        aircraftConflictCountNoLoc = 0
        aircraftConflictCountLoc = 0
        wakeConflictCountNoLoc = 0
        wakeConflictCountLoc = 0
    }

    /** Returns a snapshot of current state for RL state restore (rollback). */
    fun getStateForSnapshot(): RewardHandlerSnapshotData = RewardHandlerSnapshotData(
        acPrevLocDistPx = acPrevLocDistPx.copyOf(),
        acPrevAlt = acPrevAlt.copyOf(),
        acPrevClearance = Array(acPrevClearance.size) { i -> acPrevClearance[i]?.let { copyClearanceState(it) } },
        mvaConflictCount = mvaConflictCount,
        aircraftConflictCountNoLoc = aircraftConflictCountNoLoc,
        aircraftConflictCountLoc = aircraftConflictCountLoc,
        wakeConflictCountNoLoc = wakeConflictCountNoLoc,
        wakeConflictCountLoc = wakeConflictCountLoc,
    )

    /** Applies a previously snapshotted state (used after restore). */
    fun applyState(state: RewardHandlerSnapshotData) {
        for (i in 0 until minOf(acPrevLocDistPx.size, state.acPrevLocDistPx.size)) {
            acPrevLocDistPx[i] = state.acPrevLocDistPx[i]
            acPrevAlt[i] = state.acPrevAlt[i]
            acPrevClearance[i] = state.acPrevClearance[i]
        }
        mvaConflictCount = state.mvaConflictCount
        aircraftConflictCountNoLoc = state.aircraftConflictCountNoLoc
        aircraftConflictCountLoc = state.aircraftConflictCountLoc
        wakeConflictCountNoLoc = state.wakeConflictCountNoLoc
        wakeConflictCountLoc = state.wakeConflictCountLoc
    }

    fun rewardStep(aircraft: Array<Entity?>, aircraftMap: GdxArrayMap<String, Aircraft>, conflicts: GdxArray<Conflict>): Array<Float?> {
        if (aircraft.size != MAX_RL_AIRCRAFT) throw IllegalStateException("Expected $MAX_RL_AIRCRAFT but found ${aircraft.size}")

        if (eval) GAME.gameServer?.sendConflicts(conflicts, EMPTY_POTENTIAL_CONFLICTS)

        val rewards = Array<Float?>(MAX_RL_AIRCRAFT) { null }

        for (i in 0 until aircraft.size) {
            val currAircraft = aircraft[i] ?: continue

            // Aircraft x, y, alt, gs, track
            val currAcInfo = currAircraft[AircraftInfo.mapper]!!
            val currPos = currAircraft[Position.mapper]!!
            val currAlt = currAircraft[Altitude.mapper]!!
            val currIas = currAircraft[IndicatedAirSpeed.mapper]!!
            val currClearance = getLatestClearanceState(currAircraft)!!
            val currLocCap = if (currAircraft.has(LocalizerCaptured.mapper) || currAircraft.has(LandingRoll.mapper)) 1.byte else 0.byte

            var acReward = acPrevClearance[i]?.let { prevClearance ->
                val clearanceChangePenalty = (if (prevClearance.vectorHdg != currClearance.vectorHdg) CLEARANCE_CHANGE_PENALTY else 0f) +
                     (if (prevClearance.clearedAlt != currClearance.clearedAlt) CLEARANCE_CHANGE_PENALTY else 0f) +
                     (
                             if (prevClearance.clearedIas > currClearance.clearedIas) CLEARANCE_CHANGE_PENALTY
                             else if (prevClearance.clearedIas < currClearance.clearedIas) {
                                 CLEARANCE_CHANGE_PENALTY * (currClearance.clearedIas - prevClearance.clearedIas) / 5
                             } else 0f
                     )
                -clearanceChangePenalty
            } ?: 0f
            acPrevClearance[i] = currClearance.copy()

            if (!aircraftMap.containsKey(currAcInfo.icaoCallsign)) {
                // Lump sum reward on landing
                acReward += goalReward
            }

            // Reward from previous action
            if (!eval) {
                // Decrease in distance towards LOC line segment (x4 penalty if distance increases)
                // + decrease in altitude (x4 penalty if altitude increases)
                val newLocDistPx = distPxFromLoc(currPos, targetApproach.value.entity, 6)
                val prevLocDist = acPrevLocDistPx[i]
                val prevAlt = acPrevAlt[i]
                if (prevLocDist != null && prevAlt != null) {
                    val deltaDist = prevLocDist - newLocDistPx
                    val distReward = if (deltaDist >= 0) deltaDist / 1600 else deltaDist / 400
                    val deltaAlt = prevAlt - currAlt.altitudeFt
                    val altReward = if (deltaAlt >= 0) deltaAlt / 12000 else deltaAlt / 3000
                    acReward += distReward + altReward
                }

                acPrevLocDistPx[i] = newLocDistPx
                acPrevAlt[i] = currAlt.altitudeFt

                // Discourage aircraft from loitering too long close to LOC
                if (currLocCap == 0.byte && newLocDistPx < nmToPx(4) && currAlt.altitudeFt <= 6010) acReward -= LOC_PROX_PENALTY

//                if (currLocCap == 1.byte) {
//                    val currIas = currAircraft[IndicatedAirSpeed.mapper]?.iasKt!!
//                    val appEntity = currAircraft[GlideSlopeCaptured.mapper]?.gsApp ?: currAircraft[LocalizerCaptured.mapper]?.locApp ?: currAircraft[VisualCaptured.mapper]?.visApp
//                    val rwyThrPos = appEntity?.get(ApproachInfo.mapper)?.rwyObj?.entity?.get(CustomPosition.mapper)
//                    if (rwyThrPos != null) {
//                        val distNm = pxToNm(calculateDistanceBetweenPoints(currPos.x, currPos.y, rwyThrPos.x, rwyThrPos.y))
//
//                        // Compute max allowed speed for distance from runway
//                        // Max 230 knots @9nm, linear down to 170 knots @3nm
//                        val maxSpd = max(140 + distNm * 10, 170f)
//                        if (distNm < 9.0f && maxSpd < currIas) acReward -= HIGH_APP_SPD_PENALTY
//                    }
//                }
            }

            // Constant per time step penalty
            // Multiplier for low speeds (increased drag from flaps) or low altitudes (increased drag due to air density)
            // 1 @>=200 knots, 1.5 @<=150 knots
            // 1.1 @0 feet, 1@10000 feet, 0.8@>=30000 feet
            val iasMult = MathUtils.clamp((300 - currIas.iasKt) / 100f, 1f, 1.5f)
            val altMult = 1  // MathUtils.clamp(1 + (10000 - currAlt.altitudeFt) / 10000, 0.8f, 1f)
            // Constant factor of 1 for time + fuel consumption factor
            val multiplier = 1 + iasMult * altMult
            acReward -= PER_STEP_PENALTY * multiplier


            // Assign negative reward for conflict(s) involving this aircraft
            conflicts.filter { it.entity1 == currAircraft || it.entity2 == currAircraft }.forEach { conflict ->
                val ac1Loc = conflict.entity1.has(LocalizerCaptured.mapper)
                val ac2Loc = conflict.entity2?.has(LocalizerCaptured.mapper) ?: false
                if (conflict.entity2 != null) {
                    if (conflict.reason == Conflict.RL_AIRCRAFT_CONFLICT_INCREASED_MARGIN) {
                        // Use stricter rules for calculating rewards
                        acReward -= aircraftConflictPenalty
                    } else {
                        // But the actual rules when evaluating conflict rate
                        if (ac1Loc && ac2Loc) aircraftConflictCountNoLoc++ else aircraftConflictCountNoLoc++
                    }
                } else {
                    if (conflict.reason == Conflict.WAKE_INFRINGE) {
                        if (ac1Loc) wakeConflictCountLoc++ else wakeConflictCountNoLoc++
                    } else if (conflict.reason == Conflict.RL_WAKE_CONFLICT_INCREASED_MARGIN) {
                        acReward -= wakeConflictPenalty
                    } else {
                        acReward -= mvaConflictPenalty
                        mvaConflictCount++
                    }
                }
            }

            rewards[i] = acReward
        }

        return rewards
    }
}