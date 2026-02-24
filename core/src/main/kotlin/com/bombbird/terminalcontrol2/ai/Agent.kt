package com.bombbird.terminalcontrol2.ai

import com.badlogic.ashley.core.Entity
import com.bombbird.terminalcontrol2.entities.Aircraft
import ktx.collections.GdxArrayMap

interface Agent {
    fun init()

    fun reset()

    fun update(aircraft: GdxArrayMap<String, Aircraft>, deltaTime: Float, stopServer: () -> Unit, resetEpisode: () -> Unit)

    fun getEpisodeSpawnCount(): Int

    fun incrementSpawnCount()

    fun despawnAircraft(aircraft: Entity)
}

object StubAgent: Agent {
    override fun init() {}

    override fun reset() {}

    override fun update(aircraft: GdxArrayMap<String, Aircraft>, deltaTime: Float, stopServer: () -> Unit, resetEpisode: () -> Unit) {}

    override fun getEpisodeSpawnCount(): Int = 0

    override fun incrementSpawnCount() {}

    override fun despawnAircraft(aircraft: Entity) {}
}