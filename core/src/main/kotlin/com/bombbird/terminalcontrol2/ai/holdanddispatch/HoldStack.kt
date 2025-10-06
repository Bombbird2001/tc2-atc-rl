package com.bombbird.terminalcontrol2.ai.holdanddispatch

import com.badlogic.gdx.utils.Queue
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.navigation.Route
import com.bombbird.terminalcontrol2.navigation.createCustomHoldWaypoint

class AircraftHold(val aircraft: Aircraft, val timeEnteredHold: Int)

class HoldStack(
    posX: Float, posY: Float, private val minAlt: Int, private val inboundHdg: Short,
    private val legDistNm: Byte, private val turnDir: Byte, customName: String
) {
    private val wptId = createCustomHoldWaypoint(posX, posY, customName)
    private val inHoldStack = Queue<AircraftHold>()
    private var tick = 0

    fun tick() {
        tick++
    }

    fun getFirstAircraft(): AircraftHold {
        return inHoldStack.first()
    }

    fun removeFirstAircraft(): AircraftHold {
        return inHoldStack.removeFirst()
    }

    fun addAircraft(aircraft: Aircraft) {
        inHoldStack.addLast(AircraftHold(aircraft, tick))
    }

    fun getHoldLeg(): Route.HoldLeg {
        return Route.HoldLeg(wptId, null, minAlt, 230, 240, inboundHdg, legDistNm, turnDir)
    }

    fun getWptLeg(): Route.WaypointLeg {
        return Route.WaypointLeg(wptId, null, minAlt, 250, legActive = true, altRestrActive = true, spdRestrActive = true)
    }
}