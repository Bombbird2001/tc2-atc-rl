@file:JvmName("RLLauncher")

package com.bombbird.terminalcontrol2.atcrl

import com.badlogic.gdx.Gdx
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Files
import com.bombbird.terminalcontrol2.TerminalControl2
import com.bombbird.terminalcontrol2.files.StubExternalFileHandler
import com.bombbird.terminalcontrol2.integrations.StubAchievementHandler
import com.bombbird.terminalcontrol2.integrations.StubDiscordHandler
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.sounds.StubTextToSpeech

/** Launches a headless application for the purposes of atc-rl. */
fun main(args: Array<String>) {
    if (args.size != 1 || args[0].toIntOrNull() == null) {
        throw IllegalArgumentException("Must pass an integer for envId")
    }
    TerminalControl2(StubExternalFileHandler, StubTextToSpeech, StubDiscordHandler, StubAchievementHandler)
    Gdx.files = Lwjgl3Files()
    GameServer.newRLGameServer("TCWS", args[0].toInt())
}