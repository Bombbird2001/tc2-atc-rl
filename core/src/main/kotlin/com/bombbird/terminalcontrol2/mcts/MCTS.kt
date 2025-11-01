package com.bombbird.terminalcontrol2.mcts

import kotlin.math.max

class MCTS(val root: AircraftSelectNode) {
    companion object {
        const val SEARCH_DEPTH_MAX = 1200  // Max 300 timesteps (x4 actions per timestep = 1200)
        const val TRUNCATION_REWARD = -1f
        const val DISCOUNT_FACTOR = 0.99f
    }

    private var maxDepth = 0
    private var nodeVisitCount = 0

    fun searchAction(timeLimitMs: Long): Array<Int> {
        val startTime = System.currentTimeMillis()
        while (System.currentTimeMillis() - startTime < timeLimitMs) {
            searchReward(root, 0)
        }
        println("Visited nodes $nodeVisitCount times")
        println("Max depth: $maxDepth")
        println("Expected reward: ${root.getExpectedReward()}")

        // Select action chain of length 4 that terminates with highest expected rewards
        var currNode: MCTSNode = root
        val actions = Array(4) { -1 }
        for (i in 0 until 4) {
            val bestAction = currNode.selectBestAction()
            currNode = currNode.getNodeForAction(bestAction) ?: break
            actions[i] = bestAction
        }

        println(actions.joinToString(" "))

        return actions
    }

    fun searchReward(node: MCTSNode, depth: Int): Float {
        maxDepth = max(maxDepth, depth)
        nodeVisitCount++
        if (depth >= SEARCH_DEPTH_MAX) return -1f

        if (node.isTerminal()) return node.getTerminalReward()

        // We are quite certain that a random rollout will cause the task to fail before reaching the max
        // depth - return the default truncated reward
        if (node.isLeafNode()) {
            node.episodeEnd(TRUNCATION_REWARD)
            return TRUNCATION_REWARD
        }

        val action = node.getPUCTAction()
//        println("Selected action $action for node $node")
        val selectedNode = node.takeAction(action)
        val episodeReward = searchReward(selectedNode, depth + 1)
        node.episodeEnd(episodeReward)

        // If this node has no partial actions, it is the result of a complete action taken
        // Hence we discount the rewards if it is positive (but we keep negative failure rewards)
        if (episodeReward < 0) return episodeReward
        return (if (node.hasPartialActions) DISCOUNT_FACTOR else 1f) * episodeReward
    }
}