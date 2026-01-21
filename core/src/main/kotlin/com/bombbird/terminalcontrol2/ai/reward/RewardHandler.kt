package com.bombbird.terminalcontrol2.ai.reward

import com.badlogic.ashley.core.Entity
import com.badlogic.ashley.utils.ImmutableArray
import com.bombbird.terminalcontrol2.components.Altitude
import com.bombbird.terminalcontrol2.components.ApproachChildren
import com.bombbird.terminalcontrol2.components.LocalizerCaptured
import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.global.CHECK_AIRCRAFT_CONFLICT
import com.bombbird.terminalcontrol2.global.CHECK_MVA_CONFLICT
import com.bombbird.terminalcontrol2.global.CLEARANCE_CHANGE_PENALTY
import com.bombbird.terminalcontrol2.global.CONFLICT_PENALTY
import com.bombbird.terminalcontrol2.global.GAME
import com.bombbird.terminalcontrol2.global.LOC_CAP_REWARD
import com.bombbird.terminalcontrol2.global.MAX_RL_AIRCRAFT
import com.bombbird.terminalcontrol2.global.PER_STEP_PENALTY
import com.bombbird.terminalcontrol2.navigation.ClearanceState
import com.bombbird.terminalcontrol2.navigation.distPxFromLoc
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.utilities.byte
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import ktx.ashley.get
import ktx.ashley.has
import ktx.collections.GdxArray
import ktx.collections.toGdxArray

class RewardHandler {
    private val conflictManager = ConflictManager()

    private val acOnLoc: Array<Boolean?> = Array(MAX_RL_AIRCRAFT) { null }
    private val acPrevLocDistPx: Array<Float?> = Array(MAX_RL_AIRCRAFT) { null }
    private val acPrevAlt: Array<Float?> = Array(MAX_RL_AIRCRAFT) { null }
    private val acPrevClearance: Array<ClearanceState?> = Array(MAX_RL_AIRCRAFT) { null }

    private val targetApproach = lazy {
        GAME.gameServer?.airports?.get(0)?.entity?.get(ApproachChildren.mapper)?.approachMap?.get("ILS 02L")!!
    }

    fun rewardReset() {
        for (i in 0 until MAX_RL_AIRCRAFT) {
            acOnLoc[i] = null
            acPrevLocDistPx[i] = null
            acPrevAlt[i] = null
            acPrevClearance[i] = null
        }
    }

    fun rewardStep(aircraft: Array<Entity?>): Array<Float?> {
        if (aircraft.size != MAX_RL_AIRCRAFT) throw IllegalStateException("Expected $MAX_RL_AIRCRAFT but found ${aircraft.size}")

        val conflicts = if (CHECK_AIRCRAFT_CONFLICT || CHECK_MVA_CONFLICT) {
            // Conflict check
            conflictManager.getConflictsRL(ImmutableArray(aircraft.filterNotNull().toGdxArray()))
        } else GdxArray()

        val rewards = Array<Float?>(MAX_RL_AIRCRAFT) { null }

        for (i in 0 until aircraft.size) {
            val currAircraft = aircraft[i] ?: continue

            // Aircraft x, y, alt, gs, track
            val currPos = currAircraft[Position.mapper]!!
            val currAlt = currAircraft[Altitude.mapper]!!
            val currClearance = getLatestClearanceState(currAircraft)!!
            val currLocCap = if (currAircraft.has(LocalizerCaptured.mapper)) 1.byte else 0.byte

            var acReward = acPrevClearance[i]?.let { prevClearance ->
                val clearanceChangePenalty = (if (prevClearance.vectorHdg != currClearance.vectorHdg) CLEARANCE_CHANGE_PENALTY else 0f) +
                     (if (prevClearance.clearedAlt != currClearance.clearedAlt) CLEARANCE_CHANGE_PENALTY else 0f) +
                     (if (prevClearance.clearedIas != currClearance.clearedIas) CLEARANCE_CHANGE_PENALTY else 0f)
                -clearanceChangePenalty
            } ?: 0f
            acPrevClearance[i] = currClearance.copy()

            var ignorePositiveRewards = false

            if (currLocCap == 1.byte) {
                // If aircraft has previously captured LOC, ignore its rewards
                if (acOnLoc[i] == true) ignorePositiveRewards = true
                else {
                    // Lump sum reward on LOC capture
                    acReward += LOC_CAP_REWARD
                    acOnLoc[i] = true
                    acPrevLocDistPx[i] = null
                    acPrevAlt[i] = null
                }
            } else {
                acOnLoc[i] = false
            }

            if (!ignorePositiveRewards) {
                // Reward from previous action
                // Constant per time step penalty + decrease in distance towards LOC line segment (x4 penalty if distance increases)
                // + decrease in altitude (x4 penalty if altitude increases)
                val newLocDistPx = distPxFromLoc(currPos, targetApproach.value.entity, 6)
                val prevLocDist = acPrevLocDistPx[i]
                val prevAlt = acPrevAlt[i]
                if (prevLocDist != null && prevAlt != null) {
                    val deltaDist = prevLocDist - newLocDistPx
                    val distReward = if (deltaDist >= 0) deltaDist / 1600 else deltaDist / 400
                    val deltaAlt = prevAlt - currAlt.altitudeFt
                    val altReward = if (deltaAlt >= 0) deltaAlt / 12000 else deltaAlt / 3000
                    acReward += distReward + altReward - PER_STEP_PENALTY
                }

                acPrevLocDistPx[i] = newLocDistPx
                acPrevAlt[i] = currAlt.altitudeFt
            }

            // Assign negative reward for conflict involving this aircraft
            if (conflicts.find { it.entity1 == currAircraft || it.entity2 == currAircraft } != null) {
                acReward -= CONFLICT_PENALTY
            }
            // TODO Smaller negative reward for all other aircraft?

            // Discourage aircraft from loitering too long close to LOC
//                if (newLocDistPx < nmToPx(4) && currAlt.altitudeFt <= 6010) totalAcReward -= 0.06f

            rewards[i] = acReward
        }

        return rewards
    }
}