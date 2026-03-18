package com.bombbird.terminalcontrol2.ai.holdanddispatch

import com.badlogic.ashley.core.Entity
import com.badlogic.ashley.utils.ImmutableArray
import com.badlogic.gdx.math.MathUtils
import com.badlogic.gdx.math.Vector2
import com.badlogic.gdx.utils.Queue
import com.bombbird.terminalcontrol2.ai.Agent
import com.bombbird.terminalcontrol2.ai.reward.RewardHandler
import com.bombbird.terminalcontrol2.components.AircraftInfo
import com.bombbird.terminalcontrol2.components.Altitude
import com.bombbird.terminalcontrol2.components.ApproachChildren
import com.bombbird.terminalcontrol2.components.ClearanceAct
import com.bombbird.terminalcontrol2.components.CommandTarget
import com.bombbird.terminalcontrol2.components.Direction
import com.bombbird.terminalcontrol2.components.GroundTrack
import com.bombbird.terminalcontrol2.components.LocalizerCaptured
import com.bombbird.terminalcontrol2.components.Position
import com.bombbird.terminalcontrol2.components.Speed
import com.bombbird.terminalcontrol2.entities.Aircraft
import com.bombbird.terminalcontrol2.global.CHECK_AIRCRAFT_CONFLICT
import com.bombbird.terminalcontrol2.global.CHECK_MVA_CONFLICT
import com.bombbird.terminalcontrol2.global.CHECK_WAKE_CONFLICT
import com.bombbird.terminalcontrol2.global.HALF_TURN_RATE_THRESHOLD_IAS
import com.bombbird.terminalcontrol2.global.MAX_HIGH_SPD_ANGULAR_SPD
import com.bombbird.terminalcontrol2.global.MAX_LOW_SPD_ANGULAR_SPD
import com.bombbird.terminalcontrol2.global.MAX_RL_AIRCRAFT
import com.bombbird.terminalcontrol2.navigation.Approach
import com.bombbird.terminalcontrol2.navigation.ClearanceState
import com.bombbird.terminalcontrol2.navigation.Route
import com.bombbird.terminalcontrol2.networking.GameServer
import com.bombbird.terminalcontrol2.traffic.WakeMatrix
import com.bombbird.terminalcontrol2.traffic.conflict.Conflict
import com.bombbird.terminalcontrol2.traffic.conflict.ConflictManager
import com.bombbird.terminalcontrol2.utilities.CsvWriter
import com.bombbird.terminalcontrol2.utilities.FileLog
import com.bombbird.terminalcontrol2.utilities.addNewClearanceToPendingClearances
import com.bombbird.terminalcontrol2.utilities.calculateDistanceBetweenPoints
import com.bombbird.terminalcontrol2.utilities.calculateIASFromTAS
import com.bombbird.terminalcontrol2.utilities.convertWorldAndRenderDeg
import com.bombbird.terminalcontrol2.utilities.findDeltaHeading
import com.bombbird.terminalcontrol2.utilities.getLatestClearanceState
import com.bombbird.terminalcontrol2.utilities.getRequiredTrack
import com.bombbird.terminalcontrol2.utilities.nmToPx
import ktx.ashley.get
import ktx.ashley.has
import ktx.collections.GdxArray
import ktx.collections.GdxArrayMap
import ktx.collections.set
import ktx.collections.toGdxArray
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

class HoldAndDispatch(
    private val gs: GameServer, private val conflictManager: ConflictManager,
    goalReward: Float, mvaConflictPenalty: Float, aircraftConflictPenalty: Float, wakeConflictPenalty: Float
): Agent {
    companion object {
        var timePassed = 0f

        const val STEP_INTERVAL = 10 * 30
    }

    private var stepCountdown = STEP_INTERVAL
    private var episodeCounter = -1
    private var episodeStepCounter = 0
    private val rewardHandler = RewardHandler(
        conflictManager, true, goalReward, mvaConflictPenalty, aircraftConflictPenalty, wakeConflictPenalty
    )

    val holdingStacks: GdxArray<HoldStack> = GdxArray()
    private val distNmFromFAF = 10
    private val offsetAngle = 30
    private val dispatchSpacingBufferNm = 0.5f
    private val holdAltInterval = 2000
    private var lastDispatchedAircraft: Aircraft? = null
    private lateinit var targetLocPoint: Position
    private lateinit var targetApp: Approach

    enum class AIState {
        PENDING_ENTER_HOLD,
        IN_HOLD,
        EXITED_HOLD
    }

    private var spawnCount = 0
    private val acArray: Array<Entity?> = Array(MAX_RL_AIRCRAFT) { null }
    private val acStates: GdxArrayMap<String, AIState> = GdxArrayMap()
    private val assignedStack: GdxArrayMap<String, HoldStack> = GdxArrayMap()

    private var holdingTimeQueue = Queue<Float>()

    override fun init() {
        targetLocPoint = gs.waypoints[gs.updatedWaypointMapping["MENNU"]]!!.entity[Position.mapper]!!
        targetApp = gs.airports[0].entity[ApproachChildren.mapper]!!.approachMap["ILS 02L"]!!
        val targetAppTrack = convertWorldAndRenderDeg(targetApp.entity[Direction.mapper]!!.trackUnitVector.angleDeg() + 180)

        val distPx = nmToPx(distNmFromFAF)
        holdingStacks.add(HoldStack(
            targetLocPoint.x - MathUtils.sinDeg(targetAppTrack + offsetAngle - 10) * distPx,
            targetLocPoint.y - MathUtils.cosDeg(targetAppTrack + offsetAngle - 10) * distPx,
            5000, 43, 5, CommandTarget.TURN_LEFT, "ILS-02L-LEFT-HOLD", holdAltInterval
        ))

        holdingStacks.add(HoldStack(
            targetLocPoint.x - MathUtils.sinDeg(targetAppTrack - offsetAngle) * distPx,
            targetLocPoint.y - MathUtils.cosDeg(targetAppTrack - offsetAngle) * distPx,
            5000, 353, 5, CommandTarget.TURN_RIGHT, "ILS-02L-RIGHT-HOLD", holdAltInterval
        ))
    }

    override fun getEpisodeSpawnCount(): Int {
        return spawnCount
    }

    override fun incrementSpawnCount() {
        spawnCount++
    }

    override fun despawnAircraft(aircraft: Entity) {
        val acIndex = acArray.indexOf(aircraft)
        if (acIndex == -1) return
        acArray[acIndex] = null
    }

    override fun reset() {
        spawnCount = 1
        episodeCounter++
        episodeStepCounter = 0
        stepCountdown = STEP_INTERVAL
        rewardHandler.rewardReset()
        holdingTimeQueue.clear()
        acStates.clear()
        assignedStack.clear()
        for (stack in holdingStacks) stack.reset()
        for (i in 0 until acArray.size) acArray[i] = null
    }

    override fun update(aircraft: GdxArrayMap<String, Aircraft>, deltaTime: Float, stopServer: () -> Unit, resetEpisode: () -> Unit) {
        if (aircraft.isEmpty || episodeStepCounter >= 600) {
            resetEpisode()
            reset()

            if (episodeCounter % 10 == 0 && episodeCounter > 0) println("Finished episode $episodeCounter")

            if (episodeCounter >= 256) {
                stopServer()
                return
            }
        }

        var holdCountChanged = false

        for (i in 0 until aircraft.size) {
            val ac = aircraft.getValueAt(i)
            val callsign = ac.entity[AircraftInfo.mapper]!!.icaoCallsign

            // New aircraft, no state
            if (!acStates.containsKey(callsign)) {
                // Add to entity array
                acArray[acArray.indexOf(null)] = ac.entity

                val latestClearance = getLatestClearanceState(ac.entity)!!
                if (latestClearance.route.size >= 2) {
                    // Lateral assignment (Hold stack)
                    val lastLegWptId = (latestClearance.route[latestClearance.route.size - 1] as? Route.WaypointLeg)?.wptId?.toInt()
                    val secondLastWptId = (latestClearance.route[latestClearance.route.size - 2] as? Route.WaypointLeg)?.wptId?.toInt()
                    val holdStackId: Int
                    val newClearance: ClearanceState
                    if (secondLastWptId == 143 && lastLegWptId == 15) {
                        // LEVEX - DOGGO -> replace DOGGO with left hold point
                        val newRoute = Route()
                        newRoute.setToRouteCopy(latestClearance.route)
                        newRoute.removeIndex(newRoute.size - 1)
                        newRoute.add(holdingStacks[0].getWptLeg())
                        newRoute.add(holdingStacks[0].getHoldLeg())
                        newClearance = latestClearance.copy(route = newRoute)
                        holdStackId = 0
                    } else if ((secondLastWptId == 146 && lastLegWptId == 15) || (secondLastWptId == 29 && lastLegWptId == 31)) {
                        // GINON - SANTA
                        // MUDUP - DOGGO
                        // -> replace DOGGO/SANTA with right hold point
                        val newRoute = Route()
                        newRoute.setToRouteCopy(latestClearance.route)
                        newRoute.removeIndex(newRoute.size - 1)
                        newRoute.add(holdingStacks[1].getWptLeg())
                        newRoute.add(holdingStacks[1].getHoldLeg())
                        if (lastLegWptId == 31) (newRoute[newRoute.size - 3] as Route.WaypointLeg).spdRestrActive = false
                        newClearance = latestClearance.copy(route = newRoute)
                        holdStackId = 1
                    } else if (secondLastWptId == 34 && lastLegWptId == 31) {
                        // PUSOB - SANTA -> add right hold point at end
                        val newRoute = Route()
                        newRoute.setToRouteCopy(latestClearance.route)
                        newRoute.add(holdingStacks[1].getWptLeg())
                        newRoute.add(holdingStacks[1].getHoldLeg())
                        (newRoute[newRoute.size - 3] as Route.WaypointLeg).spdRestrActive = false
                        newClearance = latestClearance.copy(route = newRoute)
                        holdStackId = 1
                    } else {
                        FileLog.warn("HoldAndDispatch", "Route with no valid hold: ${latestClearance.route}")
                        throw IllegalStateException("No holding stack found for $callsign")
                    }

                    // Vertical assignment - choose the aircraft with highest cleared altitude + 2000 ft
                    val holdStack = holdingStacks[holdStackId]
                    val newAlt = max(holdStack.getHighestEnteringHoldAltitude() + holdAltInterval, holdStack.minAlt)
                    newClearance.clearedAlt = newAlt
                    newClearance.deactivateAllAltRestrictions()
                    addNewClearanceToPendingClearances(ac.entity, newClearance, 0)
                    holdStack.addAircraftToEnteringHold(ac)
                    assignedStack[callsign] = holdStack
                    acStates[callsign] = AIState.PENDING_ENTER_HOLD
                }
            } else if (acStates[callsign] == AIState.PENDING_ENTER_HOLD) {
                val currentClearance = ac.entity[ClearanceAct.mapper]!!.actingClearance.clearanceState
                val latestClearance = getLatestClearanceState(ac.entity)!!
                if (currentClearance.route.size == 1 && currentClearance.route[0] is Route.HoldLeg) {
                    acStates[callsign] = AIState.IN_HOLD
                    val stack = assignedStack[callsign]
                    stack.removeAircraftFromEnteringHold(ac)
                    val idealHoldAlt = min(max(stack.getHighestHoldingAltitude() + holdAltInterval, stack.minAlt), latestClearance.clearedAlt)
                    var currentClearedAlt = latestClearance.clearedAlt
                    while (currentClearedAlt > idealHoldAlt) {
                        val aircraftToSwapAlts = stack.getAircraftEnteringHoldInLowerLayerFarEnough(currentClearedAlt, 15) ?: break
                        val toSwapOriginal = getLatestClearanceState(aircraftToSwapAlts.entity)!!
                        val toSwapClearance = toSwapOriginal.copy(clearedAlt = currentClearedAlt, route = Route().apply { setToRouteCopy(toSwapOriginal.route) })
                        val newClearance = latestClearance.copy(clearedAlt = currentClearedAlt - holdAltInterval, route = Route().apply { setToRouteCopy(latestClearance.route) })
                        addNewClearanceToPendingClearances(aircraftToSwapAlts.entity, toSwapClearance, 0)
                        addNewClearanceToPendingClearances(ac.entity, newClearance, 0)
                        currentClearedAlt -= holdAltInterval
                    }
                    stack.addAircraftToHold(ac, timePassed)
                    holdCountChanged = true
                }
            }
        }

        lastDispatchedAircraft?.let {
            if (it.entity.has(LocalizerCaptured.mapper)) lastDispatchedAircraft = null
        }

        var nextStackToDispatchFrom: HoldStack? = null
        var lowestEntryTime = Float.MAX_VALUE
        for (holdStack in holdingStacks) {
            holdStack.sortHoldAircraftByClearedAlt()
            val firstAc = holdStack.getFirstInHoldAircraft() ?: continue
            val alt = firstAc.aircraft.entity[Altitude.mapper]!!
            if ((alt.altitudeFt - holdStack.minAlt) > 500) continue
            if (firstAc.timeEnteredHold < lowestEntryTime) {
                nextStackToDispatchFrom = holdStack
                lowestEntryTime = firstAc.timeEnteredHold
            }
        }

        if (nextStackToDispatchFrom != null) {
            val ac = nextStackToDispatchFrom.getFirstInHoldAircraft()!!.aircraft
            val dispatch = lastDispatchedAircraft?.let {
                val acPos = ac.entity[Position.mapper]!!
                val altitude = ac.entity[Altitude.mapper]!!.altitudeFt
                val turnRate = if (calculateIASFromTAS(altitude, ac.entity[Speed.mapper]!!.speedKts) > HALF_TURN_RATE_THRESHOLD_IAS) MAX_HIGH_SPD_ANGULAR_SPD else MAX_LOW_SPD_ANGULAR_SPD
                val groundTrack = ac.entity[GroundTrack.mapper]!!.trackVectorPxps
                val estimatedTravelDistPx = calculateDistanceToPointWithTurn(acPos.x, acPos.y, targetLocPoint.x, targetLocPoint.y, groundTrack, turnRate, groundTrack.len())
                val lastDispatchedPos = it.entity[Position.mapper]!!
                // Ensure at least 2nm spacing between this and previously dispatched aircraft before dispatching next
                if (calculateDistanceBetweenPoints(lastDispatchedPos.x, lastDispatchedPos.y, acPos.x, acPos.y) <= nmToPx(2.5f)) return@let false
                // Ensure previous dispatched aircraft has already turned towards the LOC before dispatching next
                if (abs(findDeltaHeading(
                        convertWorldAndRenderDeg(it.entity[GroundTrack.mapper]!!.trackVectorPxps.angleDeg()),
                        getRequiredTrack(lastDispatchedPos.x, lastDispatchedPos.y, targetLocPoint.x, targetLocPoint.y),
                        CommandTarget.TURN_DEFAULT
                )) > 30) return@let false
                val lastDispatchedPx = calculateDistanceBetweenPoints(lastDispatchedPos.x, lastDispatchedPos.y, targetLocPoint.x, targetLocPoint.y)
                val leaderInfo = it.entity[AircraftInfo.mapper]!!
                val followerInfo = ac.entity[AircraftInfo.mapper]!!
                val minimumDispatchDistNmRequired = max(WakeMatrix.getDistanceRequired(
                    leaderInfo.aircraftPerf.wakeCategory, leaderInfo.aircraftPerf.recat,
                    followerInfo.aircraftPerf.wakeCategory, followerInfo.aircraftPerf.recat
                ).toFloat(), 2.5f)
                val additionalSpacingNm = 6.4f * (195 - leaderInfo.aircraftPerf.appSpd) / leaderInfo.aircraftPerf.appSpd +
                        max(0f, minimumDispatchDistNmRequired - 6.4f) * (200 - leaderInfo.aircraftPerf.appSpd) / leaderInfo.aircraftPerf.appSpd +
                        (if (minimumDispatchDistNmRequired < 2.6f) -0.5f else 0f)
                        4.4f * 30 / 190 +
                        dispatchSpacingBufferNm
                (estimatedTravelDistPx - lastDispatchedPx) >= nmToPx(minimumDispatchDistNmRequired + additionalSpacingNm)
            } ?: true

            holdCountChanged = holdCountChanged || dispatch
            if (dispatch) {
                val latestClearance = getLatestClearanceState(ac.entity)!!
                val newClearance = latestClearance.copy(route = Route().apply { setToRouteCopy(targetApp.routeLegs) }, clearedAlt = 3500, clearedApp = "ILS 02L", clearedTrans = "vectors")
                addNewClearanceToPendingClearances(ac.entity, newClearance, 0)
                val holdInfo = nextStackToDispatchFrom.removeFirstInHoldAircraft()
                nextStackToDispatchFrom.clearAllHoldingAircraftDownwards()
                nextStackToDispatchFrom.clearAllPendingEnterAircraftDownwards()
                lastDispatchedAircraft = ac
                acStates[ac.entity[AircraftInfo.mapper]!!.icaoCallsign] = AIState.EXITED_HOLD
                updateHoldingTimeStatistics(timePassed - holdInfo.timeEnteredHold)
            }
        }

        if (holdCountChanged) {
            updateHoldingCountStatistics(holdingStacks.sumOf { it.getHoldingCount() })
        }

        timePassed += deltaTime

        stepCountdown--
        if (stepCountdown < 0) {
            val conflicts = getConflicts()
            var acConflicts = 0
            var mvaConflicts = 0
            var wakeConflicts = 0
            for (conflict in conflicts) {
                if (conflict.entity2 != null) {
                    acConflicts++
                } else if (conflict.reason == Conflict.MVA) {
                    mvaConflicts++
                } else if (conflict.reason == Conflict.WAKE_INFRINGE) {
                    wakeConflicts++
                }
            }

            CsvWriter.writeStepMetrics(
                episodeCounter, episodeStepCounter, aircraft.size, spawnCount, mvaConflicts, acConflicts,
                wakeConflicts,rewardHandler.rewardStep(acArray, gs.aircraft, conflicts)
            )

            stepCountdown = STEP_INTERVAL
            episodeStepCounter++
        }
    }

    private fun updateHoldingTimeStatistics(newHoldingTime: Float) {
        holdingTimeQueue.addLast(newHoldingTime)

        if (holdingTimeQueue.size > 30) holdingTimeQueue.removeFirst()

        val holdingTime = holdingTimeQueue.average().toFloat()

        CsvWriter.writeToAverageHoldingTime(timePassed, holdingTime)
        CsvWriter.writeToIndividualHoldTime(holdingTime)
    }

    private fun updateHoldingCountStatistics(newHoldCount: Int) {
        CsvWriter.writeToHoldingCount(timePassed, newHoldCount.toFloat())
    }

    fun calculateDistanceToPointWithTurn(posX: Float, posY: Float, destX: Float, destY: Float, dir: Vector2, maxTurnRateDegPerS: Float, gsPxps: Float): Float {
        val turnRadiusPx = gsPxps / Math.toRadians(maxTurnRateDegPerS.toDouble()).toFloat()
        val degreeOffset = abs(
            findDeltaHeading(
                convertWorldAndRenderDeg(dir.angleDeg()),
                getRequiredTrack(posX, posY, destX, destY),
                CommandTarget.TURN_DEFAULT
            )
        )
        val horizontalOffset = turnRadiusPx * if (degreeOffset <= 90) {
            MathUtils.sinDeg(degreeOffset)
        } else {
            1 - MathUtils.cosDeg(degreeOffset)
        }

        val verticalOffset = turnRadiusPx * MathUtils.sinDeg(degreeOffset)
        val distFromPoint = calculateDistanceBetweenPoints(posX, posY, destX, destY)

        return sqrt((distFromPoint - verticalOffset).pow(2) + horizontalOffset.pow(2)) + turnRadiusPx * Math.toRadians(degreeOffset.toDouble()).toFloat()
    }

    private fun getConflicts(): GdxArray<Conflict> {
        return if (CHECK_AIRCRAFT_CONFLICT || CHECK_MVA_CONFLICT || CHECK_WAKE_CONFLICT) {
            // Conflict check
            conflictManager.getConflictsRL(ImmutableArray(acArray.filterNotNull().toGdxArray()))
        } else GdxArray()
    }
}