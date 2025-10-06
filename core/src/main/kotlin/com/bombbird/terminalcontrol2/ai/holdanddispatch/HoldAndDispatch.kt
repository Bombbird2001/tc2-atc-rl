package com.bombbird.terminalcontrol2.ai.holdanddispatch

import com.badlogic.gdx.math.MathUtils
import com.bombbird.terminalcontrol2.components.CommandTarget
import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.global.MAG_HDG_DEV
import com.bombbird.terminalcontrol2.navigation.Route
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import ktx.ashley.get
import ktx.collections.GdxArray
import ktx.collections.GdxArrayMap

class HoldAndDispatch(private val gs: GameServer) {
    val holdingStacks: GdxArray<HoldStack> = GdxArray()

    fun init() {
        val targetLocPoint = gs.waypoints[gs.updatedWaypointMapping["MENNU"]]!!.entity[Position.mapper]!!

        holdingStacks.add(HoldStack(
            targetLocPoint.x - MathUtils.sinDeg(53 - MAG_HDG_DEV),
            targetLocPoint.y - MathUtils.cosDeg(53 - MAG_HDG_DEV),
            5000, 53, 5, CommandTarget.TURN_LEFT, "ILS-02L-LEFT-HOLD"
        ))

        holdingStacks.add(HoldStack(
            targetLocPoint.x - MathUtils.sinDeg(353 - MAG_HDG_DEV),
            targetLocPoint.y - MathUtils.cosDeg(353 - MAG_HDG_DEV),
            5000, 353, 5, CommandTarget.TURN_LEFT, "ILS-02L-RIGHT-HOLD"
        ))
    }

    fun update(aircraft: GdxArrayMap<String, Aircraft>) {
        for (i in 0 until aircraft.size) {
            val ac = aircraft.getValueAt(i).entity
            val latestClearance = getLatestClearanceState(ac)!!
            if (latestClearance.route.size >= 2) {
                val lastLegWptId = (latestClearance.route[latestClearance.route.size - 1] as? Route.WaypointLeg)?.wptId?.toInt()
                val secondLastWptId = (latestClearance.route[latestClearance.route.size - 2] as? Route.WaypointLeg)?.wptId?.toInt()
                if (secondLastWptId == 143 && lastLegWptId == 15) {
                    // LEVEX - DOGGO -> replace DOGGO with left hold point
                    val newRoute = Route()
                    newRoute.setToRouteCopy(latestClearance.route)
                    newRoute.removeIndex(newRoute.size - 1)
                    newRoute.add(holdingStacks[0].getWptLeg())
                    newRoute.add(holdingStacks[0].getHoldLeg())
                    val newClearance = latestClearance.copy(route = newRoute)
                } else if (secondLastWptId == 143 && lastLegWptId == 15) {
                    // GINON - SANTA
                    // MUDUP - DOGGO
                    // -> replace DOGGO/SANTA with right hold point
                    val newRoute = Route()
                    newRoute.setToRouteCopy(latestClearance.route)
                    newRoute.removeIndex(newRoute.size - 1)
                    newRoute.add(holdingStacks[1].getWptLeg())
                    newRoute.add(holdingStacks[1].getHoldLeg())
                    val newClearance = latestClearance.copy(route = newRoute)
                } else if (secondLastWptId == 34 && lastLegWptId == 15) {
                    // PUSOB - SANTA -> add right hold point at end
                    val newRoute = Route()
                    newRoute.setToRouteCopy(latestClearance.route)
                    newRoute.add(holdingStacks[1].getWptLeg())
                    newRoute.add(holdingStacks[1].getHoldLeg())
                    val newClearance = latestClearance.copy(route = newRoute)
                }
            }
        }
    }
}