package com.bombbird.terminalcontrol2.mcts

data class MCTSAircraftState(
    val posX: Float,
    val posY: Float,
    val altitudeFt: Float,
    val gsKts: Float,
    val hdgDeg: Float,
    val angularSpdDps: Float,
    val vertSpdFpm: Float,
    val clearedAlt: Int,
    val clearedHdg: Short,
    val clearedIas: Short,
    val locCap: Boolean
)