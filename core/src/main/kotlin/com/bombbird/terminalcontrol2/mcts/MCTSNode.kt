package com.bombbird.terminalcontrol2.mcts

import com.bombbird.terminalcontrol2.global.MAX_AIRCRAFT
import kotlin.math.sqrt

abstract class MCTSNode(val acStates: Array<MCTSAircraftState>) {
    companion object {
        const val AIRCRAFT_SELECT_ACTIONS = MAX_AIRCRAFT
        const val HEADING_SELECT_ACTIONS = 72
        const val ALTITUDE_SELECT_ACTIONS = 14
        const val SPEED_SELECT_ACTIONS = 10
        const val EXPLORATION_CONSTANT = 2
    }

    protected abstract val childNodes: Array<MCTSNode?>
    abstract val hasPartialActions: Boolean
    protected abstract val actionProbabilities: Array<Float>
    private var timesVisited = 0
    private var totalReward = 0f

    init {
        if (acStates.size > MAX_AIRCRAFT) {
            throw IllegalArgumentException("Maximum $MAX_AIRCRAFT aircraft allowed")
        }
    }

    abstract fun takeAction(action: Int): MCTSNode

    abstract fun isTerminal(): Boolean

    abstract fun getTerminalReward(): Float

    fun getExpectedReward(): Float {
        return totalReward / timesVisited
    }

    fun getPriorProbabilityForAction(action: Int): Float {
        return actionProbabilities[action]
    }

    fun getPUCTAction(): Int {
        return childNodes.indices.maxBy { action ->
            val childNode = childNodes[action]
            val qScore = childNode?.getExpectedReward() ?: 0f
            val visitRatio = sqrt(timesVisited.toFloat()) / (1 + (childNode?.timesVisited ?: 0))

            return@maxBy qScore + EXPLORATION_CONSTANT * getPriorProbabilityForAction(action) * visitRatio
        }
    }

    fun episodeEnd(reward: Float) {
        timesVisited++
        totalReward += reward
    }

    fun isLeafNode(): Boolean {
        return timesVisited == 0
    }

    override fun toString(): String {
        return javaClass.simpleName
    }
}

class AircraftSelectNode(acStates: Array<MCTSAircraftState>, val ended: Boolean, val endReward: Float): MCTSNode(acStates) {
    companion object {
        val AIRCRAFT_SELECT_PRIOR = ManualAircraftSelectPolicyPrior()
    }

    override val childNodes: Array<MCTSNode?> = Array(AIRCRAFT_SELECT_ACTIONS) { null }
    override val hasPartialActions = false
    override val actionProbabilities by lazy {
        AIRCRAFT_SELECT_PRIOR.getProbabilityDistributionForAction(acStates)
    }

    override fun takeAction(action: Int): MCTSNode {
        return childNodes[action] ?: HeadingSelectNode(acStates, action).apply {
            childNodes[action] = this
        }
    }

    override fun isTerminal(): Boolean {
        return ended
    }

    override fun getTerminalReward(): Float {
        return endReward
    }
}

class HeadingSelectNode(acStates: Array<MCTSAircraftState>, val aircraftSelected: Int): MCTSNode(acStates) {
    companion object {
        val HEADING_SELECT_PRIOR = ManualHeadingSelectPolicyPrior()
    }

    override val childNodes: Array<MCTSNode?> = Array(HEADING_SELECT_ACTIONS) { null }
    override val hasPartialActions = true
    override val actionProbabilities by lazy {
        HEADING_SELECT_PRIOR.getProbabilityDistributionForAction(acStates, aircraftSelected)
    }

    override fun takeAction(action: Int): MCTSNode {
        return childNodes[action] ?: AltitudeSelectNode(acStates, aircraftSelected, action).apply {
            childNodes[action] = this
        }
    }

    override fun isTerminal(): Boolean {
        return false
    }

    override fun getTerminalReward(): Float {
        return Float.MIN_VALUE
    }
}

class AltitudeSelectNode(
    acStates: Array<MCTSAircraftState>, val aircraftSelected: Int,
    val headingSelected: Int
): MCTSNode(acStates) {
    companion object {
        val ALTITUDE_SELECT_PRIOR = ManualAltitudeSelectPolicyPrior()
    }

    override val childNodes: Array<MCTSNode?> = Array(ALTITUDE_SELECT_ACTIONS) { null }
    override val hasPartialActions = true
    override val actionProbabilities by lazy {
        ALTITUDE_SELECT_PRIOR.getProbabilityDistributionForAction(
            acStates,
            aircraftSelected,
            headingSelected
        )
    }

    override fun takeAction(action: Int): MCTSNode {
        return childNodes[action] ?: SpeedSelectNode(acStates, aircraftSelected, headingSelected, action).apply {
            childNodes[action] = this
        }
    }

    override fun isTerminal(): Boolean {
        return false
    }

    override fun getTerminalReward(): Float {
        return Float.MIN_VALUE
    }
}

class SpeedSelectNode(
    acStates: Array<MCTSAircraftState>, val aircraftSelected: Int,
    val headingSelected: Int, val altitudeSelected: Int,
): MCTSNode(acStates) {
    companion object {
        val SPEED_SELECT_PRIOR = ManualSpeedSelectPolicyPrior()
    }

    override val childNodes: Array<MCTSNode?> = Array(SPEED_SELECT_ACTIONS) { null }
    override val hasPartialActions = true
    override val actionProbabilities by lazy {
        SPEED_SELECT_PRIOR.getProbabilityDistributionForAction(
            acStates,
            aircraftSelected,
            headingSelected,
            altitudeSelected
        )
    }

    override fun takeAction(action: Int): MCTSNode {
        return childNodes[action] ?: simulateWithActions().apply {
            childNodes[action] = this
        }
    }

    private fun simulateWithActions(): AircraftSelectNode {
        // TODO("Simulate selected actions in simulator")

        // Dummy value for now
        return AircraftSelectNode(arrayOf(MCTSAircraftState(
            -500f, -250f, 3000f, 230f, 90f,
            0f, 0f, 3000, 90, 220, false
        )), false, Float.MAX_VALUE)
    }

    override fun isTerminal(): Boolean {
        return false
    }

    override fun getTerminalReward(): Float {
        return Float.MIN_VALUE
    }
}