package com.bombbird.terminalcontrol2.gymnasium

import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.networking.GameServer
import ktx.collections.GdxArrayMap

interface GymnasiumBridge {
    fun update(aircraft: GdxArrayMap<String, Aircraft>, stopServer: () -> Unit, gs: GameServer, resetAircraft: () -> GdxArrayMap<String, Aircraft>)

    fun getEpisodeSpawnCount(): Int

    fun incrementSpawnCount()
}


object StubGymnasiumBridge: GymnasiumBridge {
    override fun update(aircraft: GdxArrayMap<String, Aircraft>, stopServer: () -> Unit, gs: GameServer, resetAircraft: () -> GdxArrayMap<String, Aircraft>) {}

    override fun getEpisodeSpawnCount(): Int = 0

    override fun incrementSpawnCount() {}
}