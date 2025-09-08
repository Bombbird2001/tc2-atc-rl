package com.bombbird.terminalcontrol2.gymnasium

import com.bombbird.terminalcontrol2.entities.Aircraft
import ktx.collections.GdxArrayMap

interface GymnasiumBridge {
    fun update(aircraft: GdxArrayMap<String, Aircraft>, resetAircraft: () -> GdxArrayMap<String, Aircraft>)
}


object StubGymnasiumBridge: GymnasiumBridge {
    override fun update(aircraft: GdxArrayMap<String, Aircraft>, resetAircraft: () -> GdxArrayMap<String, Aircraft>) {}
}