@file:JvmName("RLLauncher")

package com.bombbird.terminalcontrol2.atcrl

import com.badlogic.gdx.Gdx
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Files
import com.bombbird.terminalcontrol2.TerminalControl2
import com.bombbird.terminalcontrol2.files.StubExternalFileHandler
import com.bombbird.terminalcontrol2.integrations.StubAchievementHandler
import com.bombbird.terminalcontrol2.integrations.StubDiscordHandler
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.networking.RLHeadlessTrainingConfig
import com.bombbird.terminalcontrol2.sounds.StubTextToSpeech

/** Launches a headless application for the purposes of atc-rl. */
fun main(args: Array<String>) {
    if (args.size != 8) {
        throw IllegalArgumentException(
            "Must pass arguments for envId, evalMode, goalReward, mvaConflictPenalty, aircraftConflictPenalty, " +
                    "wakeConflictPenalty, randomSpawnChance, and scriptedSpawnFile (use empty string for random-only)"
        )
    }
    TerminalControl2(StubExternalFileHandler, StubTextToSpeech, StubDiscordHandler, StubAchievementHandler)
    Gdx.files = Lwjgl3Files()
    val scripted = args[7].takeIf { it.isNotBlank() }
    GameServer.newRLGameServer(
        "TCWS",
        RLHeadlessTrainingConfig(
            envId = args[0],
            evalMode = args[1] == "1",
            goalReward = args[2].toFloat(),
            mvaConflictPenalty = args[3].toFloat(),
            aircraftConflictPenalty = args[4].toFloat(),
            wakeConflictPenalty = args[5].toFloat(),
            randomSpawnChance = args[6].toFloat(),
            scriptedSpawnFile = scripted,
        )
    )
}