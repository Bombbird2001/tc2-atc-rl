package com.bombbird.terminalcontrol2.ai.reward

import com.badlogic.ashley.core.Entity
import com.bombbird.terminalcontrol2.components.AircraftInfo
import com.bombbird.terminalcontrol2.components.Altitude
import com.bombbird.terminalcontrol2.components.ApproachChildren
import com.bombbird.terminalcontrol2.components.LandingRoll
import com.bombbird.terminalcontrol2.components.LocalizerCaptured
import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.global.CHECK_AIRCRAFT_CONFLICT
import com.bombbird.terminalcontrol2.global.CLEARANCE_CHANGE_PENALTY
import com.bombbird.terminalcontrol2.global.DIST_SCORE_A
import com.bombbird.terminalcontrol2.global.DIST_SCORE_B
import com.bombbird.terminalcontrol2.global.DIST_SCORE_C
import com.bombbird.terminalcontrol2.global.DIST_SCORE_M
import com.bombbird.terminalcontrol2.global.DIST_SCORE_N
import com.bombbird.terminalcontrol2.global.DIST_SCORE_V2_PENALTY
import com.bombbird.terminalcontrol2.global.DIST_SCORE_V2_THRESHOLD_NM
import com.bombbird.terminalcontrol2.global.ENABLE_PROXIMITY_SCORE
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.LOC_PROX_PENALTY
import com.bombbird.terminalcontrol2.global.MAX_RL_AIRCRAFT
import com.bombbird.terminalcontrol2.global.PER_STEP_PENALTY
import com.bombbird.terminalcontrol2.navigation.ClearanceState
import com.bombbird.terminalcontrol2.navigation.distPxFromLoc
import com.bombbird.terminalcontrol2.traffic.conflict.Conflict
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.traffic.conflict.PotentialConflict
import com.bombbird.terminalcontrol2.utilities.byte
import com.bombbird.terminalcontrol2.utilities.calculateDistanceBetweenPoints
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import com.bombbird.terminalcontrol2.utilities.nmToPx
import com.bombbird.terminalcontrol2.utilities.pxToNm
import ktx.ashley.get
import ktx.ashley.has
import ktx.collections.GdxArray
import ktx.collections.GdxArrayMap
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.max

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
    var aircraftConflictCount = 0
        private set
    var wakeConflictCount = 0
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
        aircraftConflictCount = 0
        wakeConflictCount = 0
    }

    fun rewardStep(aircraft: Array<Entity?>, aircraftMap: GdxArrayMap<String, Aircraft>, conflicts: GdxArray<Conflict>): Array<Float?> {
        if (aircraft.size != MAX_RL_AIRCRAFT) throw IllegalStateException("Expected $MAX_RL_AIRCRAFT but found ${aircraft.size}")

        if (eval) GAME.gameServer?.sendConflicts(conflicts, EMPTY_POTENTIAL_CONFLICTS)

        // Proximity score = (c * e ^ (-a * (dist_nm_between - b))) * max(0, n - m * (altitude_ft_between / 1000)), where a, b, c, m, n are constants
        val proximityRewardScores = Array(MAX_RL_AIRCRAFT) { 0f }
        if (CHECK_AIRCRAFT_CONFLICT && ENABLE_PROXIMITY_SCORE) {
            for (i in 0 until aircraft.size) {
                val pos1 = aircraft[i]?.get(Position.mapper) ?: continue
                val alt1 = aircraft[i]?.get(Altitude.mapper) ?: continue
                for (j in i + 1 until aircraft.size) {
                    val pos2 = aircraft[j]?.get(Position.mapper) ?: continue
                    val alt2 = aircraft[j]?.get(Altitude.mapper) ?: continue
                    val distNm = pxToNm(calculateDistanceBetweenPoints(pos1.x, pos1.y, pos2.x, pos2.y))
                    val altFt = abs(alt1.altitudeFt - alt2.altitudeFt)
                    val proximityScore = DIST_SCORE_C * exp(DIST_SCORE_A * (distNm - DIST_SCORE_B)) * max(0f, DIST_SCORE_N - DIST_SCORE_M * (altFt / 1000))
//                val proximityScore = if (distNm >= DIST_SCORE_V2_THRESHOLD_NM || altFt >= 975) 0f else DIST_SCORE_V2_PENALTY
                    proximityRewardScores[i] += proximityScore
                    proximityRewardScores[j] += proximityScore
                }
            }
        }

        val rewards = Array<Float?>(MAX_RL_AIRCRAFT) { null }

        for (i in 0 until aircraft.size) {
            val currAircraft = aircraft[i] ?: continue

            // Aircraft x, y, alt, gs, track
            val currAcInfo = currAircraft[AircraftInfo.mapper]!!
            val currPos = currAircraft[Position.mapper]!!
            val currAlt = currAircraft[Altitude.mapper]!!
            val currClearance = getLatestClearanceState(currAircraft)!!
            val currLocCap = if (currAircraft.has(LocalizerCaptured.mapper) || currAircraft.has(LandingRoll.mapper)) 1.byte else 0.byte

            var acReward = acPrevClearance[i]?.let { prevClearance ->
                val clearanceChangePenalty = (if (prevClearance.vectorHdg != currClearance.vectorHdg) CLEARANCE_CHANGE_PENALTY else 0f) +
                     (if (prevClearance.clearedAlt != currClearance.clearedAlt) CLEARANCE_CHANGE_PENALTY else 0f) +
                     (if (prevClearance.clearedIas != currClearance.clearedIas) CLEARANCE_CHANGE_PENALTY else 0f)
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

                // Subtract sum of proximity score between this and every other aircraft
                acReward -= proximityRewardScores[i]

                // Discourage aircraft from loitering too long close to LOC
                if (currLocCap == 0.byte && newLocDistPx < nmToPx(4) && currAlt.altitudeFt <= 6010) acReward -= LOC_PROX_PENALTY
            }

            // Constant per time step penalty
            acReward -= PER_STEP_PENALTY

            // Assign negative reward for conflict involving this aircraft
            val conflict = conflicts.find { it.entity1 == currAircraft || it.entity2 == currAircraft }
            if (conflict != null) {
                if (conflict.entity2 != null) {
                    if (conflict.reason == Conflict.RL_AIRCRAFT_CONFLICT_INCREASED_MARGIN) {
                        // Use stricter rules for calculating rewards
                        acReward -= aircraftConflictPenalty
                    } else {
                        // But the actual rules when evaluating conflict rate
                        aircraftConflictCount++
                    }
                } else {
                    if (conflict.reason == Conflict.WAKE_INFRINGE) {
                        acReward -= wakeConflictPenalty
                        wakeConflictCount++
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