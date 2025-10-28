@file:JvmName("MCTSLauncher")

package com.bombbird.terminalcontrol2.atcrl

import com.bombbird.terminalcontrol2.mcts.AircraftSelectNode
import com.bombbird.terminalcontrol2.mcts.MCTS
import com.bombbird.terminalcontrol2.mcts.MCTSAircraftState

fun main(args: Array<String>) {
    val rootNode = AircraftSelectNode(arrayOf(
        MCTSAircraftState(-750f, -250f, 7000f, 250f, 280f,
            0f, 0f, 7000, 280, 240, false)
    ), false, Float.MAX_VALUE)
    val mcts = MCTS(rootNode)
    mcts.searchAction(10000)
}