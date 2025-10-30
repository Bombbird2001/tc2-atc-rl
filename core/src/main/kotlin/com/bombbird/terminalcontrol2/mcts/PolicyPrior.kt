package com.bombbird.terminalcontrol2.mcts

import com.badlogic.gdx.math.MathUtils
import com.bombbird.terminalcontrol2.global.MAG_HDG_DEV
import com.bombbird.terminalcontrol2.utilities.calculateDistanceBetweenPoints
import com.bombbird.terminalcontrol2.utilities.distPxFromPolygon
import com.bombbird.terminalcontrol2.utilities.getRequiredTrack
import com.bombbird.terminalcontrol2.utilities.pxToNm
import kotlin.math.abs
import kotlin.math.roundToInt

const val TARGET_START_POS_X = 3.5f
const val TARGET_START_POS_Y = 34.5f
const val TARGET_END_POS_X = -274.17f
const val TARGET_END_POS_Y = -612.43f
const val TARGET_CENTER_POS_X = (TARGET_START_POS_X + TARGET_END_POS_X) / 2f
const val TARGET_CENTER_POS_Y = (TARGET_START_POS_Y + TARGET_END_POS_Y) / 2f

private fun getNormalizedDistribution(unnormalized: Array<Float>): Array<Float> {
    val normFactor = unnormalized.sum()
    return unnormalized.map { it / normFactor }.toTypedArray()
}

private fun getNormalizedDistribution(unnormalized: Iterable<Float>): Array<Float> {
    val normFactor = unnormalized.sum()
    return unnormalized.map { it / normFactor }.toTypedArray()
}

abstract class AircraftSelectPolicyPrior {
    abstract fun getProbabilityDistributionForAction(acState: Array<MCTSAircraftState>): Array<Float>
}

class ManualAircraftSelectPolicyPrior: AircraftSelectPolicyPrior() {
    override fun getProbabilityDistributionForAction(acState: Array<MCTSAircraftState>): Array<Float> {
        // Higher probability for aircraft closer to LOC
        // Base probability scale of 1 when >= 25nm
        // Then linear increase to 2 as distance decreases to 0nm
        // Then normalise as required
        val unnormalized = acState.map { acState ->
            val distNm = pxToNm(
                distPxFromPolygon(
                    floatArrayOf(TARGET_START_POS_X, TARGET_START_POS_Y, TARGET_END_POS_X, TARGET_END_POS_Y),
                    acState.posX, acState.posY
                )
            )
            val unclamped = 2 - distNm / 25
            MathUtils.clamp(unclamped, 1f, 2f)
        }

        return getNormalizedDistribution(unnormalized)
    }
}

abstract class HeadingSelectPolicyPrior {
    abstract fun getProbabilityDistributionForAction(acState: Array<MCTSAircraftState>, selectedAircraft: Int): Array<Float>
}

class ManualHeadingSelectPolicyPrior: HeadingSelectPolicyPrior() {
    override fun getProbabilityDistributionForAction(acState: Array<MCTSAircraftState>, selectedAircraft: Int): Array<Float> {
        // Case 1: Aircraft is still >= 2nm from center point
        // Higher probability for heading pointing towards center of LOC
        // Reduce probability as it spreads out from the optimal heading
        // Case 2: Aircraft is less than 2nm from center point
        // Higher probability to maintain current heading
        // Reduce probability as it spreads out from the optimal heading
        // Spread out rate: Base probability of 1 for >= 30 degrees offset from highest
        // Linear increase to 13 for highest probability choice

        val ac = acState[selectedAircraft]
        val distFromCenterNm = pxToNm(calculateDistanceBetweenPoints(ac.posX, ac.posY, TARGET_CENTER_POS_X, TARGET_CENTER_POS_Y))

        val bestHeading = if (distFromCenterNm >= 2) {
            getRequiredTrack(ac.posX, ac.posY, TARGET_CENTER_POS_X, TARGET_CENTER_POS_Y) + MAG_HDG_DEV
        } else {
            ac.clearedHdg.toFloat()
        }
        var bestAction = (bestHeading / 5).roundToInt()
        if (bestAction == 72) bestAction = 0

        val unnormalized = Array(72) {
            var bucketsAway = abs(it - bestAction)
            if (bucketsAway > 36) bucketsAway = abs(bucketsAway - 72)
            MathUtils.clamp(13 - bucketsAway * 2f, 1f, 13f)
        }

        return getNormalizedDistribution(unnormalized)
    }
}

abstract class AltitudeSelectPolicyPrior {
    abstract fun getProbabilityDistributionForAction(
        acState: Array<MCTSAircraftState>,
        selectedAircraft: Int,
        selectedHeading: Int
    ): Array<Float>
}

class ManualAltitudeSelectPolicyPrior: AltitudeSelectPolicyPrior() {
    override fun getProbabilityDistributionForAction(
        acState: Array<MCTSAircraftState>,
        selectedAircraft: Int,
        selectedHeading: Int
    ): Array<Float> {
        // Higher probability for lower altitude
        // Base probability of 1 at FL150, then linear increase to 3 as
        // altitude decreases to 3000 feet, then stays at 3 for 2000 feet
        val unnormalized = (0 until 14).map {
            MathUtils.clamp(3f + 1f / 6 - it / 6, 1f, 3f)
        }

        return getNormalizedDistribution(unnormalized)
    }
}

abstract class SpeedSelectPolicyPrior {
    abstract fun getProbabilityDistributionForAction(
        acState: Array<MCTSAircraftState>,
        selectedAircraft: Int,
        selectedHeading: Int,
        selectedAltitude: Int,
    ): Array<Float>
}

class ManualSpeedSelectPolicyPrior: SpeedSelectPolicyPrior() {
    override fun getProbabilityDistributionForAction(
        acState: Array<MCTSAircraftState>,
        selectedAircraft: Int,
        selectedHeading: Int,
        selectedAltitude: Int,
    ): Array<Float> {
        // Higher probability for higher speeds
        // Base probability of 1 at 160 knots, then linear increase to 2
        // as speed increases to 220 knots, then stays at 2 till 250 knots
        val unnormalized = (0 until 10).map {
            MathUtils.clamp(1f + it / 6f, 1f, 2f)
        }

        return getNormalizedDistribution(unnormalized)
    }
}