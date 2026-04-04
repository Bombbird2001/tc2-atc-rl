package com.bombbird.terminalcontrol2.ai.holdanddispatch

import com.bombbird.terminalcontrol2.components.AircraftInfo
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
import ktx.collections.GdxArrayMap
import ktx.collections.isNotEmpty
import ktx.collections.set
import ktx.collections.sortBy
import kotlin.math.max

class AircraftHold(val aircraft: Aircraft, val timeEnteredHold: Float)

class HoldStack(
    posX: Float, posY: Float, val minAlt: Int, private val inboundHdg: Short,
    private val legDistNm: Byte, private val turnDir: Byte, customName: String,
    private val holdAltInterval: Int
) {
    private val wptId = createCustomHoldWaypoint(posX, posY, customName)
    private val inHoldStack = GdxArray<AircraftHold>()
    private val pendingEnterHold = GdxArray<Aircraft>()

    fun checkAircraftClearedAltNoConflict() {
        val altitudesTaken = GdxArrayMap<Int, Pair<String, String>>()
        for (hold in inHoldStack) {
            val alt = getLatestClearanceState(hold.aircraft.entity)!!.clearedAlt
            val callsign = hold.aircraft.entity[AircraftInfo.mapper]!!.icaoCallsign
            if (altitudesTaken.containsKey(alt)) {
                println(inHoldStack.map { "${it.aircraft.entity[AircraftInfo.mapper]!!.icaoCallsign} ${getLatestClearanceState(it.aircraft.entity)!!.clearedAlt}" })
                println(pendingEnterHold.map { "${it.entity[AircraftInfo.mapper]!!.icaoCallsign} ${getLatestClearanceState(it.entity)!!.clearedAlt}" })
                throw IllegalStateException("$callsign cleared to $alt ft in hold, but it is already taken by ${altitudesTaken[alt]}")
            }
            altitudesTaken[alt] = callsign to "in hold"
        }

        for (aircraft in pendingEnterHold) {
            val alt = getLatestClearanceState(aircraft.entity)!!.clearedAlt
            val callsign = aircraft.entity[AircraftInfo.mapper]!!.icaoCallsign
            if (altitudesTaken.containsKey(alt)) {
                println(inHoldStack.map { "${it.aircraft.entity[AircraftInfo.mapper]!!.icaoCallsign} ${getLatestClearanceState(it.aircraft.entity)!!.clearedAlt}" })
                println(pendingEnterHold.map { "${it.entity[AircraftInfo.mapper]!!.icaoCallsign} ${getLatestClearanceState(it.entity)!!.clearedAlt}" })
                throw IllegalStateException("$callsign cleared to $alt ft before hold, but it is already taken by ${altitudesTaken[alt]}")
            }
            altitudesTaken[alt] = callsign to "pending hold"
        }
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
//            println("Cleared ${ac[AircraftInfo.mapper]!!.icaoCallsign} in hold to ${newClearance.clearedAlt}")
        }
    }

    fun addAircraftToHold(aircraft: Aircraft, timeEnteredHold: Float) {
        inHoldStack.add(AircraftHold(aircraft, timeEnteredHold))
    }

    fun addAircraftToEnteringHold(aircraft: Aircraft) {
        pendingEnterHold.add(aircraft)
    }

    fun removeAircraftFromEnteringHold(aircraft: Aircraft) {
        pendingEnterHold.removeValue(aircraft, true)
    }

    fun getHighestEnteringHoldAltitude(): Int {
        val inHoldMax = if (inHoldStack.isNotEmpty()) inHoldStack.maxOf { holdInfo ->
            getLatestClearanceState(holdInfo.aircraft.entity)!!.clearedAlt
        } else -1

        val pendingHoldMax = if (pendingEnterHold.isNotEmpty()) pendingEnterHold.maxOf { ac ->
            getLatestClearanceState(ac.entity)!!.clearedAlt
        } else -1

        return max(inHoldMax, pendingHoldMax)
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
//            println("Cleared ${ac.entity[AircraftInfo.mapper]!!.icaoCallsign} pending hold to ${newClearance.clearedAlt}")
        }
    }

    fun getHoldLeg(): Route.HoldLeg {
        return Route.HoldLeg(wptId, null, minAlt, 230, 240, inboundHdg, legDistNm, turnDir)
    }

    fun getWptLeg(): Route.WaypointLeg {
        return Route.WaypointLeg(wptId, null, minAlt, 220, legActive = true, altRestrActive = true, spdRestrActive = true)
    }

    fun getHoldingCount(): Int {
        return inHoldStack.size
    }

    fun reset() {
        inHoldStack.clear()
        pendingEnterHold.clear()
    }
}