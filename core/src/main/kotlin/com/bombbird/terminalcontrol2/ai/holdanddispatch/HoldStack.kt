package com.bombbird.terminalcontrol2.ai.holdanddispatch

import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.navigation.Route
import com.bombbird.terminalcontrol2.navigation.calculateDistToGo
import com.bombbird.terminalcontrol2.navigation.createCustomHoldWaypoint
import com.bombbird.terminalcontrol2.utilities.addNewClearanceToPendingClearances
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import com.bombbird.terminalcontrol2.utilities.nmToPx
import ktx.ashley.get
import ktx.collections.GdxArray
import ktx.collections.sortBy

class AircraftHold(val aircraft: Aircraft, val tickEnteredHold: Int)

class HoldStack(
    posX: Float, posY: Float, val minAlt: Int, private val inboundHdg: Short,
    private val legDistNm: Byte, private val turnDir: Byte, customName: String,
    private val holdAltInterval: Int
) {
    private val wptId = createCustomHoldWaypoint(posX, posY, customName)
    private val inHoldStack = GdxArray<AircraftHold>()
    private val pendingEnterHold = GdxArray<Aircraft>()
    private var tick = 0

    fun tick() {
        tick++
    }

    fun sortHoldAircraftByClearedAlt() {
        inHoldStack.sortBy { hold -> getLatestClearanceState(hold.aircraft.entity)!!.clearedAlt }
    }

    fun getFirstInHoldAircraft(): AircraftHold? {
        if (inHoldStack.isEmpty) return null
        return inHoldStack[0]
    }

    fun removeFirstInHoldAircraft(): AircraftHold {
        return inHoldStack.removeIndex(0)
    }

    fun getHighestHoldingAltitude(): Int {
        if (inHoldStack.isEmpty) return -1
        return inHoldStack.maxOf { hold -> getLatestClearanceState(hold.aircraft.entity)!!.clearedAlt }
    }

    fun clearAllHoldingAircraftDownwards() {
        for (hold in inHoldStack) {
            val ac = hold.aircraft.entity
            val clearance = getLatestClearanceState(ac)!!
            val newClearance = clearance.copy(clearedAlt = clearance.clearedAlt - holdAltInterval, route = Route().apply { setToRouteCopy(clearance.route) })
            addNewClearanceToPendingClearances(ac, newClearance, 0)
        }
    }

    fun addAircraftToHold(aircraft: Aircraft) {
        inHoldStack.add(AircraftHold(aircraft, tick))
    }

    fun addAircraftToEnteringHold(aircraft: Aircraft) {
        pendingEnterHold.add(aircraft)
    }

    fun removeAircraftFromEnteringHold(aircraft: Aircraft) {
        pendingEnterHold.removeValue(aircraft, true)
    }

    fun getHighestEnteringHoldAltitude(): Int {
        if (pendingEnterHold.isEmpty) {
            if (inHoldStack.isEmpty) {
                return -1
            }

            return inHoldStack.maxOf { holdInfo ->
                getLatestClearanceState(holdInfo.aircraft.entity)!!.clearedAlt
            }
        }

        return pendingEnterHold.maxOf { ac ->
            getLatestClearanceState(ac.entity)!!.clearedAlt
        }
    }

    fun getAircraftEnteringHoldInLowerLayerFarEnough(currentAlt: Int, minDistNm: Int): Aircraft? {
        for (ac in pendingEnterHold) {
            val latestClearance = getLatestClearanceState(ac.entity)!!
            // Aircraft must be exactly one interval below in altitude
            if (latestClearance.clearedAlt != currentAlt - holdAltInterval) continue
            // Less than 2 legs -> aircraft has already reached hold -> break
            if (latestClearance.route.size < 2) break
            val acPos = ac.entity[Position.mapper]!!
            // Dist to go less than required -> break
            if (calculateDistToGo(acPos, latestClearance.route[0],
                    latestClearance.route[latestClearance.route.size - 2], latestClearance.route) < nmToPx(minDistNm)
            ) break
            return ac
        }

        return null
    }

    fun clearAllPendingEnterAircraftDownwards() {
        for (ac in pendingEnterHold) {
            val clearance = getLatestClearanceState(ac.entity)!!
            val newClearance = clearance.copy(clearedAlt = clearance.clearedAlt - holdAltInterval, route = Route().apply { setToRouteCopy(clearance.route) })
            addNewClearanceToPendingClearances(ac.entity, newClearance, 0)
        }
    }

    fun getHoldLeg(): Route.HoldLeg {
        return Route.HoldLeg(wptId, null, minAlt, 230, 240, inboundHdg, legDistNm, turnDir)
    }

    fun getWptLeg(): Route.WaypointLeg {
        return Route.WaypointLeg(wptId, null, minAlt, 250, legActive = true, altRestrActive = true, spdRestrActive = true)
    }
}