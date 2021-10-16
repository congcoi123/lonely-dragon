package com.congcoi123.lonely.dragon.behavior

import com.congcoi123.lonely.dragon.configuration.ParamLoader
import com.congcoi123.lonely.dragon.constant.Behavior
import com.congcoi123.lonely.dragon.constant.Deceleration
import com.congcoi123.lonely.dragon.constant.SummingMethod
import com.congcoi123.lonely.dragon.entity.Vehicle
import com.congcoi123.lonely.dragon.entity.Wall
import com.tenio.common.utility.MathUtility
import com.tenio.engine.physic2d.common.BaseGameEntity
import com.tenio.engine.physic2d.common.Path
import com.tenio.engine.physic2d.graphic.Paint
import com.tenio.engine.physic2d.graphic.Renderable
import com.tenio.engine.physic2d.math.Vector2
import com.tenio.engine.physic2d.utility.Geometry
import com.tenio.engine.physic2d.utility.Transformation
import java.awt.Color
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * This class is used to encapsulate steering behaviors for a vehicle.
 */
class SteeringBehavior(
    private val vehicle: Vehicle
) : Renderable {
    private val paramLoader = ParamLoader.instance

    // a vertex buffer to contain the feelers for wall avoidance
    private val sensors: MutableList<Vector2>

    // the length of the 'feeler/s' used in wall detection
    private val wallDetectionSensorLength: Float

    // the current position on to wander circle the agent is
    // attempting to steer towards
    private val wanderTarget: Vector2

    // explained above
    private val wanderJitter: Float
    private val wanderRadius: Float
    private val wanderDistance: Float

    // multipliers. These can be adjusted to effect strength of the
    // appropriate behavior. Useful to get flocking the way you require
    // for example.
    private val separationWeight: Float
    private val cohesionWeight: Float
    private val alignmentWeight: Float
    private val weightWander: Float
    private val weightObstacleAvoidance: Float
    private val weightWallAvoidance: Float
    private val weightSeek: Float
    private val weightFlee: Float
    private val weightArrive: Float
    private val weightPursuit: Float
    private val weightOffsetPursuit: Float
    private val weightInterpose: Float
    private val weightHide: Float
    private val weightEvade: Float
    private val weightFollowPath: Float

    // pointer to any current path
    private val path: Path

    // the distance (squared) a vehicle has to be from a path way-point before
    // it starts seeking to the next way-point
    private val waypointSeekDistanceSqr: Float

    // default
    private val deceleration: Deceleration

    /**
     * This function calculates how much of its max steering force the vehicle has
     * left to apply and then applies that amount of the force to add.
     */
    private val vAccumulateForce = Vector2.newInstance()

    // how far the agent can 'see'
    private val agentViewDistance: Float

    // the steering force created by the combined effect of all
    // the selected behaviors
    var force: Vector2 = Vector2.newInstance()

    // these can be used to keep track of friends, pursuers, or prey
    private lateinit var targetAgent1: Vehicle
    private lateinit var targetAgent2: Vehicle

    // length of the 'detection box' utilized in obstacle avoidance
    var detectBoxLength: Float
        private set

    // any offset used for formations or offset pursuit
    private lateinit var offset: Vector2

    // binary flags to indicate whether a behavior should be active
    private var behaviorFlag = 0

    // is cell space partitioning to be used or not?
    var isSpacePartitioning: Boolean
        private set

    // what type of method is used to sum any active behavior
    private var summingMethod: SummingMethod

    // a vertex buffer rqd for drawing the detection box
    private var detectBox: MutableList<Vector2> = ArrayList(4)

    // this function tests if a specific bit of m_iFlags is set
    private fun isBehavior(behavior: Behavior): Boolean {
        return behaviorFlag and behavior.get() == behavior.get()
    }

    private fun accumulateForce(forceToAdd: Vector2) {

        // calculate how much steering force the vehicle has used so far
        val magnitudeSoFar = force.length

        // calculate how much steering force remains to be used by this vehicle
        val magnitudeRemaining = vehicle.maxForce - magnitudeSoFar

        // return false if there is no more force left to use
        if (magnitudeRemaining <= 0) {
            return
        }

        // calculate the magnitude of the force we want to add
        val magnitudeToAdd = forceToAdd.length

        // if the magnitude of the sum of ForceToAdd and the running total
        // does not exceed the maximum force available to this vehicle, just
        // add together. Otherwise, add as much of the ForceToAdd vector is
        // possible without going over the max.
        if (magnitudeToAdd < magnitudeRemaining) {
            force.add(forceToAdd)
        } else {
            // add it to the steering force
            vAccumulateForce.set(forceToAdd).normalize().mul(magnitudeRemaining)
            force.add(vAccumulateForce)
        }
    }

    /**
     * Creates the antenna utilized by WallAvoidance.
     */
    private fun createSensors() {
        sensors.clear()
        // feeler pointing straight in front
        val front = Vector2.valueOf(vehicle.heading).mul(wallDetectionSensorLength)
            .add(vehicle.position)
        sensors.add(front)

        // feeler to left
        val left = Transformation.vec2dRotateAroundOrigin(
            vehicle.heading, MathUtility.HALF_PI * 3.5F
        )
        left.mul(wallDetectionSensorLength / 2.0F).add(vehicle.position)
        sensors.add(left)

        // feeler to right
        val right = Transformation.vec2dRotateAroundOrigin(
            vehicle.heading, MathUtility.HALF_PI * 0.5F
        )
        right.mul(wallDetectionSensorLength / 2.0F).add(vehicle.position)
        sensors.add(right)
    }

    // ---------------------------- START OF BEHAVIORS ----------------------------
    //
    // ----------------------------------------------------------------------------
    /**
     * Given a target, this behavior returns a steering force which will direct the
     * agent towards the target.
     */
    private fun doSeek(targetPos: Vector2): Vector2 {
        val desiredVelocity = Vector2.valueOf(targetPos).sub(vehicle.position).normalize()
            .mul(vehicle.maxSpeed)
        return desiredVelocity.sub(vehicle.velocity)
    }

    /**
     * Does the opposite of Seek.
     */
    private fun doFlee(targetPos: Vector2): Vector2 {
        // only flee if the target is within 'panic distance'. Work in distance squared
        // space.
        val desiredVelocity = Vector2.valueOf(vehicle.position).sub(targetPos).normalize()
            .mul(vehicle.maxSpeed)
        return desiredVelocity.sub(vehicle.velocity)
    }

    /**
     * This behavior is similar to seek, but it attempts to arrive at the target with
     * a zero velocity.
     */
    private fun doArrive(targetPos: Vector2, deceleration: Deceleration): Vector2 {
        val toTarget = Vector2.valueOf(targetPos).sub(vehicle.position)

        // calculate the distance to the target
        val dist = toTarget.length
        if (dist > 0) {
            // because Deceleration is enumerated as an integer, this value is required
            // to provide fine tweaking of the deceleration..
            val decelerationTweaker = 0.3F

            // calculate the speed required to reach the target given the desired
            // deceleration
            var speed = dist / (deceleration.get().toFloat() * decelerationTweaker)

            // make sure the velocity does not exceed the max
            speed = Math.min(speed, vehicle.maxSpeed)

            // from here proceed just like Seek except we don't need to normalize
            // the ToTarget vector because we have already gone to the trouble
            // of calculating its length: dist.
            val desiredVelocity = toTarget.mul(speed / dist)
            return desiredVelocity.sub(vehicle.velocity)
        }
        return toTarget.zero()
    }

    /**
     * This behavior creates a force that steers the agent towards the evader.
     */
    private fun doPursuit(evader: Vehicle?): Vector2 {
        // if the evader is ahead and facing the agent then we can just seek
        // for the evader's current position.
        val toEvader = evader!!.position.sub(vehicle.position)
        val relativeHeading = vehicle.heading.getDotProductValue(evader.heading)
        // acos(0.95) = 18degs
        if (toEvader.getDotProductValue(vehicle.heading) > 0
            && relativeHeading < -0.95F
        ) {
            return doSeek(evader.position)
        }

        // Not considered ahead, so we predict where the evader will be.
        // the lookahead time is proportional to the distance between the evader
        // and the pursuer; and is inversely proportional to the sum of the
        // agent's velocities
        val lookAheadTime = toEvader.length / (vehicle.maxSpeed + evader.speed)

        // now seek to the predicted future position of the evader
        return doSeek(evader.velocity.mul(lookAheadTime).add(evader.position))
    }

    /**
     * Similar to pursuit except the agent Flees from the estimated future position
     * of the pursuer.
     */
    private fun doEvade(pursuer: Vehicle?): Vector2 {
        // Not necessary to include the check for facing direction this time
        val toPursuer = pursuer!!.position.sub(vehicle.position)

        // uncomment the following two lines to have Evade only consider pursuers
        // within a 'threat range'
        val threatRange = 100F
        if (toPursuer.lengthSqr > threatRange * threatRange) {
            return Vector2.newInstance()
        }

        // the lookahead time is proportional to the distance between the pursuer
        // and the pursuer; and is inversely proportional to the sum of the
        // agents' velocities
        val lookAheadTime = toPursuer.length / (vehicle.maxSpeed + pursuer.speed)

        // now flee away from predicted future position of the pursuer
        return doFlee(pursuer.velocity.mul(lookAheadTime).add(pursuer.position))
    }

    /**
     * This behavior makes the agent wander about randomly.
     */
    private fun doWander(): Vector2 {
        // this behavior is dependent on the update rate, so this line must
        // be included when using time independent frame rate.
        val jitterThisTimeSlice = wanderJitter * vehicle.timeElapsed

        // first, add a small random vector to the target's position
        val temp2 = Vector2.newInstance().set(wanderTarget)
            .add(
                MathUtility.randomClamped() * jitterThisTimeSlice,
                MathUtility.randomClamped() * jitterThisTimeSlice
            )

        // re-project this new vector back on to a unit circle
        temp2.normalize()

        // increase the length of the vector to the same as the radius
        // of to wander circle
        temp2.mul(wanderRadius)

        // move the target into a position WanderDist in front of the agent
        val target = Vector2.newInstance().set(wanderDistance, 0F).add(temp2)

        // project the target into world space
        val targetToWorldSpace = Transformation.pointToWorldSpace(
            target,
            vehicle.heading,
            vehicle.side,
            vehicle.position
        )

        // and steer towards it
        return targetToWorldSpace.sub(vehicle.position)
    }

    /**
     * Given a vector of obstacles, this method returns a steering force that will
     * prevent the agent colliding with the closest obstacle.
     */
    private fun doObstacleAvoidance(obstacles: MutableList<BaseGameEntity>): Vector2 {
        // the detection box length is proportional to the agent's velocity
        detectBoxLength =
            (paramLoader?.MIN_DETECTION_BOX_LENGTH?.plus(vehicle.speed / vehicle.maxSpeed * paramLoader.MIN_DETECTION_BOX_LENGTH)!!)

        // tag all obstacles within range of the box for processing
        vehicle.world.tagObstaclesWithinViewRange(vehicle, detectBoxLength)

        // this will keep track of the closest intersecting obstacle (CIB)
        var closestIntersectingObstacle: BaseGameEntity? = null

        // this will be used to track the distance to the CIB
        var distToClosestIP = MathUtility.MAX_FLOAT

        // this will record the transformed local coordinates of the CIB
        var localPosOfClosestObstacle = Vector2.newInstance().zero()
        val iterator = obstacles.listIterator()
        while (iterator.hasNext()) {
            // if the obstacle has been tagged within range proceed
            val obstacle = iterator.next()
            obstacle.let {
                if (it.isTagged) {
                    // calculate this obstacle's position in local space
                    val localPos = Transformation.pointToLocalSpace(
                        it.position,
                        vehicle.heading,
                        vehicle.side,
                        vehicle.position
                    )

                    // if the local position has a negative x value then it must lay
                    // behind the agent. (in which case it can be ignored)
                    if (localPos.x >= 0) {
                        // if the distance from the x-axis to the object's position is less
                        // than its radius + half the width of the detection box then there
                        // is a potential intersection.
                        val expandedRadius = it.boundingRadius + vehicle.boundingRadius
                        if (abs(localPos.y) < expandedRadius) {
                            // now to do a line/circle intersection test. The center of the
                            // circle is represented by (cX, cY). The intersection points are
                            // given by the formula x = cX +/-sqrt(r^2-cY^2) for y=0.
                            // We only need to look at the smallest positive value of x because
                            // that will be the closest point of intersection.
                            val cX = localPos.x
                            val cY = localPos.y

                            // we only need to calculate the sqrt part of the above equation once
                            val sqrtPart = sqrt((expandedRadius * expandedRadius - cY * cY).toDouble()).toFloat()
                            var ip = cX - sqrtPart
                            if (ip <= 0.0) {
                                ip = cX + sqrtPart
                            }

                            // test to see if this is the closest so far. If it is keep a
                            // record of the obstacle and its local coordinates
                            if (ip < distToClosestIP) {
                                distToClosestIP = ip
                                closestIntersectingObstacle = it
                                localPosOfClosestObstacle = localPos
                            }
                        }
                    }
                }
            }
        }

        // if we have found an intersecting obstacle, calculate a steering
        // force away from it
        val steeringForce = Vector2.newInstance().zero()
        closestIntersectingObstacle?.let {
            // the closer the agent is to an object, the stronger the
            // steering force should be
            val multiplier = 1 + (detectBoxLength - localPosOfClosestObstacle.x) /
                    detectBoxLength

            // calculate the lateral force
            steeringForce.y = ((it.boundingRadius - localPosOfClosestObstacle.y)
                    * multiplier)

            // apply a braking force proportional to the obstacles distance from
            // the vehicle.
            val brakingWeight = 0.2F
            steeringForce.x = ((it.boundingRadius - localPosOfClosestObstacle.x)
                    * brakingWeight)
        }

        // finally, convert the steering vector from local to world space
        return Transformation.vectorToWorldSpace(
            steeringForce,
            vehicle.heading,
            vehicle.side
        )
    }

    /**
     * This returns a steering force that will keep the agent away from any walls it
     * may encounter.
     */
    private fun doWallAvoidance(walls: List<Wall>): Vector2 {
        // the feelers are contained in a list, m_Feelers
        createSensors()
        var distToThisIP = 0F
        var distToClosestIP = MathUtility.MAX_FLOAT

        // this will hold an index into the vector of walls
        var closestWall = -1
        var steeringForce = Vector2.newInstance().zero()
        // holds the closest intersection point
        var closestPoint = Vector2.newInstance().zero()

        // examine each feeler in turn
        for (sensor in sensors) {
            // run through each wall checking for any intersection points
            for ((wallIndex, wall) in walls.withIndex()) {
                distToThisIP = Geometry.getDistanceTwoSegmentIntersect(
                    vehicle.position,
                    sensor,
                    wall.from,
                    wall.to
                )
                if (distToThisIP != -1F) {
                    // is this the closest found so far? If so keep a record
                    if (distToThisIP < distToClosestIP) {
                        distToClosestIP = distToThisIP
                        closestWall = wallIndex
                        closestPoint = Geometry.getPointTwoSegmentIntersect(
                            vehicle.position,
                            sensor,
                            wall.from,
                            wall.to
                        )
                    }
                }
            } // next wall

            // if an intersection point has been detected, calculate a force
            // that will direct the agent away
            if (closestWall >= 0) {
                // calculate by what distance the projected position of the agent
                // will overshoot the wall
                val overShoot = sensor.sub(closestPoint)

                // create a force in the direction of the wall normal, with a
                // magnitude of the overshoot
                steeringForce = Vector2.newInstance().set(walls[closestWall].normal)
                    .mul(overShoot.length)
            }
        } // next feeler

        return steeringForce.clone()
    }

    /**
     * This calculates a force re-pelling from the other neighbors.
     */
    private fun doSeparation(neighbors: List<Vehicle>): Vector2 {
        val steeringForce = Vector2.newInstance().zero()
        for (neighbor in neighbors) {
            // make sure this agent isn't included in the calculations and that
            // the agent being examined is close enough. ***also make sure it doesn't
            // include to evade target ***
            if (neighbor != vehicle && neighbor.isTagged && neighbor != targetAgent1) {
                val toAgent = Vector2.newInstance().set(vehicle.position).sub(neighbor.position)

                // scale the force inversely proportional to the agents distance
                // from its neighbor.
                steeringForce.add(toAgent.normalize().div(toAgent.length))
            }
        }

        return steeringForce.clone()
    }

    /**
     * Returns a force that attempts to align this agents heading with that of its
     * neighbors.
     */
    private fun doAlignment(neighbors: List<Vehicle>): Vector2 {
        // used to record the average heading of the neighbors
        val averageHeading = Vector2.newInstance().zero()

        // used to count the number of vehicles in the neighborhood
        var neighborCount = 0

        // iterate through all the tagged vehicles and sum their heading vectors
        for (neighbor in neighbors) {
            // make sure *this* agent isn't included in the calculations and that
            // the agent being examined is close enough ***also make sure it doesn't
            // include any evade target ***
            if (neighbor != vehicle && neighbor.isTagged && neighbor != targetAgent1) {
                averageHeading.add(neighbor.heading)
                ++neighborCount
            }
        }

        // if the neighborhood contained one or more vehicles, average their
        // heading vectors.
        if (neighborCount > 0) {
            averageHeading.div(neighborCount.toFloat())
            averageHeading.sub(vehicle.heading)
        }

        return averageHeading.clone()
    }

    /**
     * Returns a steering force that attempts to move the agent towards the center
     * of mass of the agents in its immediate area.
     */
    private fun doCohesion(neighbors: List<Vehicle>): Vector2 {
        // first find the center of mass of all the agents
        val centerOfMass = Vector2.newInstance().zero()
        var steeringForce = Vector2.newInstance().zero()
        var neighborCount = 0

        // iterate through the neighbors and sum up all the position vectors
        for (neighbor in neighbors) {
            // make sure *this* agent isn't included in the calculations and that
            // the agent being examined is close enough ***also make sure it doesn't
            // include to evade target ***
            if (neighbor != vehicle && neighbor.isTagged && neighbor != targetAgent1) {
                centerOfMass.add(neighbor.position)
                ++neighborCount
            }
        }
        if (neighborCount > 0) {
            // the center of mass is the average of the sum of positions
            centerOfMass.div(neighborCount.toFloat())

            // now seek towards that position
            steeringForce = doSeek(centerOfMass)
        }

        // the magnitude of cohesion is usually much larger than separation or
        // alignment, so it usually helps to normalize it.
        return steeringForce.normalize()
    }

    /*
   * NOTE: the next three behaviors are the same as the above three, except that
   * they use a cell-space partition to find the neighbors
   */

    /**
     * This calculates a force re-pelling from the other neighbors.
     * <br>
     * USES SPACIAL PARTITIONING
     */
    private fun doSeparationPlus(neighbors: List<Vehicle>): Vector2 {
        val steeringForce = Vector2.newInstance().zero()

        // iterate through the neighbors and sum up all the position vectors
        var pV = vehicle.world.cellSpace.frontOfNeighbor
        while (!vehicle.world.cellSpace.isEndOfNeighbors) {

            // make sure this agent isn't included in the calculations and that
            // the agent being examined is close enough
            if (pV != vehicle) {
                val toAgent = Vector2.newInstance().set(vehicle.position).sub(pV.position)

                // scale the force inversely proportional to the agents distance
                // from its neighbor.
                steeringForce.add(toAgent.normalize().div(toAgent.length))
            }
            pV = vehicle.world.cellSpace.nextOfNeighbor
        }

        return steeringForce.clone()
    }

    /**
     * Returns a force that attempts to align this agents heading with that of its
     * neighbors.
     * <br>
     * USES SPACIAL PARTITIONING
     */
    private fun doAlignmentPlus(neighbors: List<Vehicle>): Vector2 {
        // This will record the average heading of the neighbors
        val averageHeading = Vector2.newInstance().zero()

        // This count the number of vehicles in the neighborhood
        var neighborCount = 0F

        // iterate through the neighbors and sum up all the position vectors
        var pV = vehicle.world.cellSpace.frontOfNeighbor
        while (!vehicle.world.cellSpace.isEndOfNeighbors) {

            // make sure *this* agent isn't included in the calculations and that
            // the agent being examined is close enough
            if (pV != vehicle) {
                averageHeading.add(pV.heading)
                ++neighborCount
            }
            pV = vehicle.world.cellSpace.nextOfNeighbor
        }

        // if the neighborhood contained one or more vehicles, average their
        // heading vectors.
        if (neighborCount > 0) {
            averageHeading.div(neighborCount)
            averageHeading.sub(vehicle.heading)
        }
        return averageHeading.clone()
    }

    /**
     * Returns a steering force that attempts to move the agent towards the center
     * of mass of the agents in its immediate area.
     * <br>
     * USES SPACIAL PARTITIONING
     */
    private fun doCohesionPlus(neighbors: List<Vehicle>): Vector2 {
        // first find the center of mass of all the agents
        val centerOfMass = Vector2.newInstance().zero()
        var steeringForce = Vector2.newInstance().zero()
        var neighborCount = 0

        // iterate through the neighbors and sum up all the position vectors
        var pV = vehicle.world.cellSpace.frontOfNeighbor
        while (!vehicle.world.cellSpace.isEndOfNeighbors) {

            // make sure *this* agent isn't included in the calculations and that
            // the agent being examined is close enough
            if (pV != vehicle) {
                centerOfMass.add(pV.position)
                ++neighborCount
            }
            pV = vehicle.world.cellSpace.nextOfNeighbor
        }
        if (neighborCount > 0) {
            // the center of mass is the average of the sum of positions
            centerOfMass.div(neighborCount.toFloat())

            // now seek towards that position
            steeringForce = doSeek(centerOfMass)
        }

        // the magnitude of cohesion is usually much larger than separation or
        // alignment, so it usually helps to normalize it.
        return steeringForce.normalize()
    }

    /**
     * Given two agents, this method returns a force that attempts to position the
     * vehicle between them.
     */
    private fun doInterpose(agentA: Vehicle, agentB: Vehicle): Vector2 {
        // first we need to figure out where the two agents are going to be at
        // time T in the future. This is approximated by determining the time
        // taken to reach the midway point at the current time at max speed.
        var midPoint = Vector2.newInstance().set(agentA.position).add(agentB.position).div(2F)
        val timeToReachMidPoint = vehicle.position.getDistanceValue(midPoint) / vehicle.maxSpeed

        // now we have T, we assume that agent A and agent B will continue on a
        // straight trajectory and extrapolate to get their future positions
        val aPos = Vector2.newInstance().set(agentA.velocity).mul(timeToReachMidPoint)
            .add(agentA.position)
        val bPos = Vector2.newInstance().set(agentB.velocity).mul(timeToReachMidPoint)
            .add(agentB.position)

        // calculate the mid-point of these predicted positions
        midPoint = Vector2.newInstance().set(aPos).add(bPos).div(2F)

        // then steer to Arrive at it
        return doArrive(midPoint, Deceleration.FAST)
    }

    private fun doHide(hunter: Vehicle, obstacles: MutableList<BaseGameEntity>): Vector2 {
        var distToClosest = MathUtility.MAX_FLOAT
        var bestHidingSpot = Vector2.newInstance().zero()
        val iterator = obstacles.listIterator()
        while (iterator.hasNext()) {
            val obstacle = iterator.next()
            // calculate the position of the hiding spot for this obstacle
            val hidingSpot =
                getHidingPosition(obstacle.position, obstacle.boundingRadius, hunter.position)

            // work in distance-squared space to find the closest hiding
            // spot to the agent
            val dist = hidingSpot.getDistanceSqrValue(vehicle.position)
            dist.let {
                if (it < distToClosest) {
                    distToClosest = it
                    bestHidingSpot = hidingSpot
                }
            }
        } // end while

        // if no suitable obstacles found then Evade the hunter
        return if (distToClosest == MathUtility.MAX_FLOAT) {
            doEvade(hunter)
        } else {
            // else use Arrive on the hiding spot
            doArrive(bestHidingSpot, Deceleration.FAST)
        }
    }

    /**
     * Given the position of a hunter, and the position and radius of an obstacle,
     * this method calculates a position DistanceFromBoundary away from its bounding
     * radius and directly opposite the hunter.
     */
    private fun getHidingPosition(posOb: Vector2, radiusOb: Float, posHunter: Vector2): Vector2 {
        // calculate how far away the agent is to be from the chosen obstacle's
        // bounding radius
        val distanceFromBoundary = 30F
        val distAway = radiusOb + distanceFromBoundary

        // calculate the heading toward the object from the hunter
        val toOb = Vector2.newInstance().set(posOb).sub(posHunter).normalize()

        // scale it to size and add to the obstacles position to get
        // the hiding spot.
        return toOb.mul(distAway).add(posOb).clone()
    }

    /**
     * Given a series of Vector2Ds, this method produces a force that will move the
     * agent along the way-points in order. The agent uses the 'Seek' behavior to
     * move to the next way-point - unless it is the last way-point, in which case
     * it 'Arrives'.
     */
    private fun doFollowPath(): Vector2 {
        // move to next target if close enough to current target (working in
        // distance squared space)
        if (path.currentWayPoint.getDistanceSqrValue(vehicle.position) < waypointSeekDistanceSqr) {
            path.setToNextWayPoint()
        }
        return if (!path.isEndOfWayPoints) {
            doSeek(path.currentWayPoint)
        } else {
            doArrive(path.currentWayPoint, Deceleration.NORMAL)
        }
    }

    /**
     * Produces a steering force that keeps a vehicle at a specified offset from a
     * leader vehicle.
     */
    private fun doOffsetPursuit(leader: Vehicle, offset: Vector2): Vector2 {
        // calculate the offset's position in world space
        val worldOffsetPos = Transformation.pointToWorldSpace(
            offset,
            leader.heading,
            leader.side,
            leader.position
        )
        val toOffset = worldOffsetPos.sub(vehicle.position)

        // the lookahead time is proportional to the distance between the leader
        // and the pursuer; and is inversely proportional to the sum of both
        // agent's velocities
        val lookAheadTime = toOffset.length / (vehicle.maxSpeed + leader.speed)

        // now Arrive at the predicted future position of the offset
        return doArrive(
            leader.velocity.mul(lookAheadTime).add(worldOffsetPos),
            Deceleration.FAST
        )
    }

    /**
     * Renders visual aids and info for seeing how each behavior is calculated.
     */
    override fun render(paint: Paint) {
        paint.enableOpaqueText(false)
        paint.setTextColor(Color.GRAY)

        var nextSlot = paint.fontHeight
        val slotSize = 20
        if (vehicle.maxForce < 0) {
            vehicle.maxForce = 0.0f
        }
        if (vehicle.maxSpeed < 0) {
            vehicle.maxSpeed = 0.0f
        }
        if (vehicle.id === "dragon") {
            paint.drawTextAtPosition(5, nextSlot, "MaxForce(Ins/Del):")
            paint.drawTextAtPosition(
                160,
                nextSlot,
                (vehicle.maxForce / (paramLoader?.STEERING_FORCE_TWEAKER ?: 1F)).toString()
            )
            nextSlot += slotSize
        }
        if (vehicle.id === "dragon") {
            paint.drawTextAtPosition(5, nextSlot, "MaxSpeed(Home/End):")
            paint.drawTextAtPosition(160, nextSlot, vehicle.maxSpeed.toString())
            nextSlot += slotSize
        }

        // render the steering force
        if (vehicle.world.isRenderSteeringForce) {
            paint.setPenColor(Color.RED)
            val force = paramLoader?.let {
                Vector2.newInstance().set(force).div(it.STEERING_FORCE_TWEAKER)
                    .mul(paramLoader.VEHICLE_SCALE)
            }
            paint.drawLine(
                vehicle.position,
                Vector2.newInstance().set(vehicle.position.add(force))
            )
        }

        // render wander stuff if relevant
        if (isBehavior(Behavior.WANDER) && vehicle.world.isRenderWanderCircle) {
            if (vehicle.id === "dragon") {
                paint.drawTextAtPosition(5, nextSlot, "Jitter(F/V): ")
                paint.drawTextAtPosition(160, nextSlot, wanderJitter.toString())
                nextSlot += slotSize
            }
            if (vehicle.id === "dragon") {
                paint.drawTextAtPosition(5, nextSlot, "Distance(G/B): ")
                paint.drawTextAtPosition(160, nextSlot, wanderDistance.toString())
                nextSlot += slotSize
            }
            if (vehicle.id === "dragon") {
                paint.drawTextAtPosition(5, nextSlot, "Radius(H/N): ")
                paint.drawTextAtPosition(160, nextSlot, wanderRadius.toString())
                nextSlot += slotSize
            }

            // calculate the center of to wander circle
            val vTCC = Transformation.pointToWorldSpace(
                Vector2.newInstance().set(wanderDistance * vehicle.boundingRadius, 0F),
                vehicle.heading,
                vehicle.side,
                vehicle.position
            )
            // draw to wander circle
            paint.setPenColor(Color.GREEN)
            paint.setBgColor(null)
            paint.drawCircle(vTCC, wanderRadius * vehicle.boundingRadius)

            // draw to wander target
            paint.setPenColor(Color.RED)
            paint.drawCircle(
                Transformation.pointToWorldSpace(
                    Vector2.newInstance().set(wanderTarget).add(wanderDistance, 0F)
                        .mul(vehicle.boundingRadius),
                    vehicle.heading,
                    vehicle.side,
                    vehicle.position
                ), 3F
            )
        }

        // render the detection box if relevant
        if (vehicle.world.isRenderDetectionBox) {
            paint.setPenColor(Color.GRAY)
            val length = ((paramLoader?.MIN_DETECTION_BOX_LENGTH ?: 0F)
                    + vehicle.speed / vehicle.maxSpeed * (paramLoader?.MIN_DETECTION_BOX_LENGTH ?: 1F))

            // vets for the detection box buffer
            detectBox.clear()
            detectBox.add(Vector2.newInstance().set(0f, vehicle.boundingRadius))
            detectBox.add(Vector2.newInstance().set(length, vehicle.boundingRadius))
            detectBox.add(Vector2.newInstance().set(length, -vehicle.boundingRadius))
            detectBox.add(Vector2.newInstance().set(0f, -vehicle.boundingRadius))

            if (!vehicle.isSmoothing) {
                detectBox = Transformation.pointsToWorldSpace(
                    detectBox,
                    vehicle.position,
                    vehicle.heading,
                    vehicle.side
                )
                paint.drawClosedShape(detectBox)
            } else {
                detectBox = Transformation.pointsToWorldSpace(
                    detectBox,
                    vehicle.position,
                    vehicle.getSmoothedHeading(),
                    Vector2.newInstance().set(vehicle.getSmoothedHeading()).perpendicular()
                )
                paint.drawClosedShape(detectBox)
            }

            // the detection box length is proportional to the agent's velocity
            detectBoxLength = ((paramLoader?.MIN_DETECTION_BOX_LENGTH ?: 0F)
                    + vehicle.speed / vehicle.maxSpeed * (paramLoader?.MIN_DETECTION_BOX_LENGTH ?: 1F))

            // tag all obstacles within range of the box for processing
            vehicle.world.tagObstaclesWithinViewRange(vehicle, detectBoxLength)
            val obstacles = vehicle.world.obstacles.listIterator()
            while (obstacles.hasNext()) {
                val obstacle = obstacles.next()
                // if the obstacle has been tagged within range proceed
                obstacle.let {
                    if (it.isTagged) {
                        // calculate this obstacle's position in local space
                        val localPos = Transformation.pointToLocalSpace(
                            it.position ?: Vector2.newInstance(),
                            vehicle.heading,
                            vehicle.side,
                            vehicle.position
                        )

                        // if the local position has a negative x value then it must lay
                        // behind the agent. (in which case it can be ignored)
                        if (localPos.x >= 0) {
                            // if the distance from the x-axis to the object's position is less
                            // than its radius + half the width of the detection box then there
                            // is a potential intersection.
                            if (abs(localPos.y) < it.boundingRadius + vehicle.boundingRadius) {
                                paint.setPenColor(Color.RED)
                                paint.drawClosedShape(detectBox)
                            }
                        }
                    }
                }
            }
        }

        // render the wall avoidance feelers
        if (isBehavior(Behavior.WALL_AVOIDANCE) && vehicle.world.isRenderSensors) {
            paint.setPenColor(Color.ORANGE)
            for (sensor in sensors) {
                paint.drawLine(vehicle.position, sensor)
            }
        }

        // render path info
        if (isBehavior(Behavior.FOLLOW_PATH) && vehicle.world.isRenderPath) {
            path.render(paint)
        }

        if (isBehavior(Behavior.SEPARATION)) {
            if (vehicle.id === "dragon") {
                paint.drawTextAtPosition(5, nextSlot, "Separation(S/X):")
                paint.drawTextAtPosition(
                    160, nextSlot, (separationWeight / (paramLoader?.STEERING_FORCE_TWEAKER ?: 1F)).toString()
                )
                nextSlot += slotSize
            }
        }

        if (isBehavior(Behavior.ALIGNMENT)) {
            if (vehicle.id === "dragon") {
                paint.drawTextAtPosition(5, nextSlot, "Alignment(A/Z):")
                paint.drawTextAtPosition(
                    160, nextSlot, (alignmentWeight / (paramLoader?.STEERING_FORCE_TWEAKER ?: 1F)
                            ).toString()
                )
                nextSlot += slotSize
            }
        }

        if (isBehavior(Behavior.COHESION)) {
            if (vehicle.id === "dragon") {
                paint.drawTextAtPosition(5, nextSlot, "Cohesion(D/C):")
                paint.drawTextAtPosition(
                    160, nextSlot, (cohesionWeight / (paramLoader?.STEERING_FORCE_TWEAKER ?: 1F))
                        .toString()
                )
                nextSlot += slotSize
            }
        }

        if (isBehavior(Behavior.FOLLOW_PATH)) {
            val sd = sqrt(waypointSeekDistanceSqr.toDouble()).toFloat()
            if (vehicle.id === "dragon") {
                paint.drawTextAtPosition(5, nextSlot, "SeekDistance(D/C):")
                paint.drawTextAtPosition(160, nextSlot, sd.toString())
                nextSlot += slotSize
            }
        }
    }

    // ---------------------------- CALCULATE METHODS ----------------------------
    //
    // ---------------------------------------------------------------------------
    /**
     * Calculates the accumulated steering force according to the method set in
     * summingMethod.
     */
    fun calculateAccumulate(): Vector2 {
        // reset the steering force
        force.zero()

        // use space partitioning to calculate the neighbors of this vehicle
        // if switched on. If not, use the standard tagging system
        if (!isSpacePartitioning) {
            // tag neighbors if any of the following 3 group behaviors are switched on
            if (isBehavior(Behavior.SEPARATION) || isBehavior(Behavior.ALIGNMENT) || isBehavior(Behavior.COHESION)) {
                // vehicle.world.tagVehiclesWithinViewRange(vehicle, 10F)
            }
        } else {
            // calculate neighbors in cell-space if any of the following 3 group
            // behaviors are switched on
            if (isBehavior(Behavior.SEPARATION) || isBehavior(Behavior.ALIGNMENT) || isBehavior(Behavior.COHESION)) {
                vehicle.world.cellSpace.calculateNeighbors(vehicle.position, 10F)
            }
        }

        when (summingMethod) {
            SummingMethod.WEIGHTED_AVERAGE -> force = calculateWeightedSum
            SummingMethod.PRIORITIZED -> calculatePrioritized()
            SummingMethod.DITHERED -> force = calculateDithered
        }

        return force
    }

    /**
     * Returns the forward component of the steering force.
     */
    val forwardComponent: Float
        get() = vehicle.heading.getDotProductValue(force)

    /**
     * Returns the side component of the steering force.
     */
    val sideComponent: Float
        get() = vehicle.side.getDotProductValue(force)

    /**
     * This method calls each active steering behavior in order of priority and
     * accumulates their forces until the max steering force magnitude is reached,
     * at which time the function returns the steering force accumulated to that
     * point.
     */
    private fun calculatePrioritized() {
        if (isBehavior(Behavior.WALL_AVOIDANCE)) {
            val force = doWallAvoidance(vehicle.world.walls).mul(weightWallAvoidance)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.OBSTACLE_AVOIDANCE)) {
            val force = doObstacleAvoidance(vehicle.world.obstacles).mul(weightObstacleAvoidance)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.EVADE)) {
            val force = doEvade(targetAgent1).mul(weightEvade)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.FLEE)) {
            val force = doFlee(vehicle.world.crosshair).mul(weightFlee)
            accumulateForce(force)
        }

        // these next three can be combined for flocking behavior (wander is
        // also a good behavior to add into this mix)
        if (!isSpacePartitioning) {
            if (isBehavior(Behavior.SEPARATION)) {
                val force = doSeparation(vehicle.world.agents).mul(separationWeight)
                accumulateForce(force)
            }
            if (isBehavior(Behavior.ALIGNMENT)) {
                val force = doAlignment(vehicle.world.agents).mul(alignmentWeight)
                accumulateForce(force)
            }
            if (isBehavior(Behavior.COHESION)) {
                val force = doCohesion(vehicle.world.agents).mul(cohesionWeight)
                accumulateForce(force)
            }
        } else {
            if (isBehavior(Behavior.SEPARATION)) {
                val force = doSeparationPlus(vehicle.world.agents).mul(separationWeight)
                accumulateForce(force)
            }
            if (isBehavior(Behavior.ALIGNMENT)) {
                val force = doAlignmentPlus(vehicle.world.agents).mul(alignmentWeight)
                accumulateForce(force)
            }
            if (isBehavior(Behavior.COHESION)) {
                val force = doCohesionPlus(vehicle.world.agents).mul(cohesionWeight)
                accumulateForce(force)
            }
        }

        if (isBehavior(Behavior.SEEK)) {
            val force = doSeek(vehicle.world.crosshair).mul(weightSeek)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.ARRIVE)) {
            val force = doArrive(vehicle.world.crosshair, deceleration).mul(weightArrive)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.WANDER)) {
            val force = doWander().mul(weightWander)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.PURSUIT)) {
            val force = doPursuit(targetAgent1).mul(weightPursuit)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.OFFSET_PURSUIT)) {
            val force = doOffsetPursuit(targetAgent1, offset)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.INTERPOSE)) {
            val force = doInterpose(targetAgent1, targetAgent2).mul(weightInterpose)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.HIDE)) {
            val force = doHide(targetAgent1, vehicle.world.obstacles).mul(weightHide)
            accumulateForce(force)
        }

        if (isBehavior(Behavior.FOLLOW_PATH)) {
            val force = doFollowPath().mul(weightFollowPath)
            accumulateForce(force)
        }
    }

    // these next three can be combined for flocking behavior (wander is
    // also a good behavior to add into this mix)

    /**
     * This simply sums up all the active behaviors X their weights and truncates
     * the result to the max available steering force before returning.
     */
    private val calculateWeightedSum: Vector2
        private get() {
            if (isBehavior(Behavior.WALL_AVOIDANCE)) {
                force.add(doWallAvoidance(vehicle.world.walls))
                    .mul(weightWallAvoidance)
            }
            if (isBehavior(Behavior.OBSTACLE_AVOIDANCE)) {
                force.add(doObstacleAvoidance(vehicle.world.obstacles))
                    .mul(weightObstacleAvoidance)
            }
            if (isBehavior(Behavior.EVADE)) {
                force.add(doEvade(targetAgent1)).mul(weightEvade)
            }

            // these next three can be combined for flocking behavior (wander is
            // also a good behavior to add into this mix)
            if (!isSpacePartitioning) {
                if (isBehavior(Behavior.SEPARATION)) {
                    force.add(doSeparation(vehicle.world.agents)).mul(separationWeight)
                }
                if (isBehavior(Behavior.ALIGNMENT)) {
                    force.add(doAlignment(vehicle.world.agents)).mul(alignmentWeight)
                }
                if (isBehavior(Behavior.COHESION)) {
                    force.add(doCohesion(vehicle.world.agents)).mul(cohesionWeight)
                }
            } else {
                if (isBehavior(Behavior.SEPARATION)) {
                    force.add(doSeparationPlus(vehicle.world.agents))
                        .mul(separationWeight)
                }
                if (isBehavior(Behavior.ALIGNMENT)) {
                    force.add(doAlignmentPlus(vehicle.world.agents)).mul(alignmentWeight)
                }
                if (isBehavior(Behavior.COHESION)) {
                    force.add(doCohesionPlus(vehicle.world.agents)).mul(cohesionWeight)
                }
            }
            if (isBehavior(Behavior.WANDER)) {
                force.add(doWander()).mul(weightWander)
            }
            if (isBehavior(Behavior.SEEK)) {
                force.add(doSeek(vehicle.world.crosshair)).mul(weightSeek)
            }
            if (isBehavior(Behavior.FLEE)) {
                force.add(doFlee(vehicle.world.crosshair)).mul(weightFlee)
            }
            if (isBehavior(Behavior.ARRIVE)) {
                force.add(doArrive(vehicle.world.crosshair, deceleration))
                    .mul(weightArrive)
            }
            if (isBehavior(Behavior.PURSUIT)) {
                force.add(doPursuit(targetAgent1)).mul(weightPursuit)
            }
            if (isBehavior(Behavior.OFFSET_PURSUIT)) {
                force.add(doOffsetPursuit(targetAgent1, offset)).mul(weightOffsetPursuit)
            }
            if (isBehavior(Behavior.INTERPOSE)) {
                force.add(doInterpose(targetAgent1, targetAgent2)).mul(weightInterpose)
            }
            if (isBehavior(Behavior.HIDE)) {
                force.add(doHide(targetAgent1, vehicle.world.obstacles))
                    .mul(weightHide)
            }
            if (isBehavior(Behavior.FOLLOW_PATH)) {
                force.add(doFollowPath()).mul(weightFollowPath)
            }
            force.truncate(vehicle.maxForce)

            return force
        }

    /**
     * This method sums up the active behaviors by assigning a probability of being
     * calculated to each behavior. It then tests the first priority to see if it
     * should be calculated this simulation-step. If so, it calculates the steering
     * force resulting from this behavior. If it is more than zero it returns the
     * force. If zero, or if the behavior is skipped it continues onto the next
     * priority, and so on.
     *<br>
     * NOTE: Not all the behaviors have been implemented in this method, just a
     * few, so you get the general idea
     */
    private val calculateDithered: Vector2
        private get() {
            // reset the steering force
            force.zero()
            if (isBehavior(Behavior.WALL_AVOIDANCE) &&
                MathUtility.randFloat() < (paramLoader?.PR_WALL_AVOIDANCE ?: 0F)
            ) {
                force = doWallAvoidance(vehicle.world.walls)
                    .mul(weightWallAvoidance / (paramLoader?.PR_WALL_AVOIDANCE ?: 0F))
                if (!force.isZero) {
                    force.truncate(vehicle.maxForce)
                    return force
                }
            }
            if (isBehavior(Behavior.OBSTACLE_AVOIDANCE)
                && MathUtility.randFloat() < (paramLoader?.PR_OBSTACLE_AVOIDANCE ?: 0F)
            ) {
                force.add(
                    doObstacleAvoidance(vehicle.world.obstacles)
                        .mul(weightObstacleAvoidance / (paramLoader?.PR_OBSTACLE_AVOIDANCE ?: 0F))
                )
                if (!force.isZero) {
                    force.truncate(vehicle.maxForce)
                    return force
                }
            }
            if (!isSpacePartitioning) {
                if (isBehavior(Behavior.SEPARATION) &&
                    MathUtility.randFloat() < (paramLoader?.PR_SEPARATION ?: 0F)
                ) {
                    force.add(
                        doSeparation(vehicle.world.agents)
                            .mul(separationWeight / (paramLoader?.PR_SEPARATION ?: 0F))
                    )
                    if (!force.isZero) {
                        force.truncate(vehicle.maxForce)
                        return force
                    }
                }
            } else {
                if (isBehavior(Behavior.SEPARATION) &&
                    MathUtility.randFloat() < (paramLoader?.PR_SEPARATION ?: 0F)
                ) {
                    force.add(
                        doSeparationPlus(vehicle.world.agents)
                            .mul(separationWeight / (paramLoader?.PR_SEPARATION ?: 0F))
                    )
                    if (!force.isZero) {
                        force.truncate(vehicle.maxForce)
                        return force
                    }
                }
            }
            if (isBehavior(Behavior.FLEE) && MathUtility.randFloat() < (paramLoader?.PR_FLEE ?: 0F)) {
                force.add(doFlee(vehicle.world.crosshair).mul(weightFlee / (paramLoader?.PR_FLEE ?: 0F)))
                if (!force.isZero) {
                    force.truncate(vehicle.maxForce)
                    return force
                }
            }
            if (isBehavior(Behavior.EVADE) && MathUtility.randFloat() < (paramLoader?.PR_EVADE ?: 0F)) {
                force.add(doEvade(targetAgent1).mul(weightEvade / (paramLoader?.PR_EVADE ?: 0F)))
                if (!force.isZero) {
                    force.truncate(vehicle.maxForce)
                    return force
                }
            }
            if (!isSpacePartitioning) {
                if (isBehavior(Behavior.ALIGNMENT) && MathUtility.randFloat() < (paramLoader?.PR_ALIGNMENT ?: 0F)) {
                    force.add(
                        doAlignment(vehicle.world.agents)
                            .mul(alignmentWeight / (paramLoader?.PR_ALIGNMENT ?: 0F))
                    )
                    if (!force.isZero) {
                        force.truncate(vehicle.maxForce)
                        return force
                    }
                }
                if (isBehavior(Behavior.COHESION) && MathUtility.randFloat() < (paramLoader?.PR_COHESION ?: 0F)) {
                    force.add(
                        doCohesion(vehicle.world.agents)
                            .mul(cohesionWeight / (paramLoader?.PR_COHESION ?: 0F))
                    )
                    if (!force.isZero) {
                        force.truncate(vehicle.maxForce)
                        return force
                    }
                }
            } else {
                if (isBehavior(Behavior.ALIGNMENT) && MathUtility.randFloat() < (paramLoader?.PR_ALIGNMENT ?: 0F)) {
                    force.add(
                        doAlignmentPlus(vehicle.world.agents)
                            .mul(alignmentWeight / (paramLoader?.PR_ALIGNMENT ?: 0F))
                    )
                    if (!force.isZero) {
                        force.truncate(vehicle.maxForce)
                        return force
                    }
                }
                if (isBehavior(Behavior.COHESION) && MathUtility.randFloat() < (paramLoader?.PR_COHESION ?: 0F)) {
                    force.add(
                        doCohesionPlus(vehicle.world.agents)
                            .mul(cohesionWeight / (paramLoader?.PR_COHESION ?: 0F))
                    )
                    if (!force.isZero) {
                        force.truncate(vehicle.maxForce)
                        return force
                    }
                }
            }
            if (isBehavior(Behavior.WANDER) && MathUtility.randFloat() < (paramLoader?.PR_WANDER ?: 0F)) {
                force.add(doWander().mul(weightWander / (paramLoader?.PR_WANDER ?: 0F)))
                if (!force.isZero) {
                    force.truncate(vehicle.maxForce)
                    return force
                }
            }
            if (isBehavior(Behavior.SEEK) && MathUtility.randFloat() < (paramLoader?.PR_SEEK ?: 0F)) {
                force
                    .add(doSeek(vehicle.world.crosshair).mul(weightSeek / (paramLoader?.PR_SEEK ?: 0F)))
                if (!force.isZero) {
                    force.truncate(vehicle.maxForce)
                    return force
                }
            }
            if (isBehavior(Behavior.ARRIVE) && MathUtility.randFloat() < (paramLoader?.PR_ARRIVE ?: 0F)) {
                force.add(
                    doArrive(vehicle.world.crosshair, deceleration)
                        .mul(weightArrive / (paramLoader?.PR_ARRIVE ?: 0F))
                )
                if (!force.isZero) {
                    force.truncate(vehicle.maxForce)
                    return force
                }
            }
            return force
        }

    fun createRandomPath(numWaypoints: Int, mx: Int, my: Int, cx: Int, cy: Int) {
        path.createRandomPath(numWaypoints, mx.toFloat(), my.toFloat(), cx.toFloat(), cy.toFloat())
    }

    fun toggleSpacePartitioning() {
        isSpacePartitioning = !isSpacePartitioning
    }

    fun setSummingMethod(sm: SummingMethod) {
        summingMethod = sm
    }

    fun setFleeOn() {
        behaviorFlag = behaviorFlag or Behavior.FLEE.get()
    }

    fun setSeekOn() {
        behaviorFlag = behaviorFlag or Behavior.SEEK.get()
    }

    fun setArriveOn() {
        behaviorFlag = behaviorFlag or Behavior.ARRIVE.get()
    }

    fun setWanderOn() {
        behaviorFlag = behaviorFlag or Behavior.WANDER.get()
    }

    private fun setCohesionOn() {
        behaviorFlag = behaviorFlag or Behavior.COHESION.get()
    }

    private fun setSeparationOn() {
        behaviorFlag = behaviorFlag or Behavior.SEPARATION.get()
    }

    private fun setAlignmentOn() {
        behaviorFlag = behaviorFlag or Behavior.ALIGNMENT.get()
    }

    fun setObstacleAvoidanceOn() {
        behaviorFlag = behaviorFlag or Behavior.OBSTACLE_AVOIDANCE.get()
    }

    fun setWallAvoidanceOn() {
        behaviorFlag = behaviorFlag or Behavior.WALL_AVOIDANCE.get()
    }

    fun setFollowPathOn() {
        behaviorFlag = behaviorFlag or Behavior.FOLLOW_PATH.get()
    }

    fun setInterposeOn(vehicle1: Vehicle, vehicle2: Vehicle) {
        behaviorFlag = behaviorFlag or Behavior.INTERPOSE.get()
        targetAgent1 = vehicle1
        targetAgent2 = vehicle2
    }

    fun setOffsetPursuitOn(vehicle: Vehicle, offset: Vector2) {
        behaviorFlag = behaviorFlag or Behavior.OFFSET_PURSUIT.get()
        this.offset = offset
        targetAgent1 = vehicle
    }

    fun setFlockingOn() {
        setCohesionOn()
        setAlignmentOn()
        setSeparationOn()
        setWanderOn()
    }

    fun setFleeOff() {
        if (isBehavior(Behavior.FLEE)) {
            behaviorFlag = behaviorFlag xor Behavior.FLEE.get()
        }
    }

    fun setSeekOff() {
        if (isBehavior(Behavior.SEEK)) {
            behaviorFlag = behaviorFlag xor Behavior.SEEK.get()
        }
    }

    fun setArriveOff() {
        if (isBehavior(Behavior.ARRIVE)) {
            behaviorFlag = behaviorFlag xor Behavior.ARRIVE.get()
        }
    }

    private fun setWanderOff() {
        if (isBehavior(Behavior.WANDER)) {
            behaviorFlag = behaviorFlag xor Behavior.WANDER.get()
        }
    }

    fun setPursuitOff() {
        if (isBehavior(Behavior.PURSUIT)) {
            behaviorFlag = behaviorFlag xor Behavior.PURSUIT.get()
        }
    }

    fun setEvadeOff() {
        if (isBehavior(Behavior.EVADE)) {
            behaviorFlag = behaviorFlag xor Behavior.EVADE.get()
        }
    }

    private fun setCohesionOff() {
        if (isBehavior(Behavior.COHESION)) {
            behaviorFlag = behaviorFlag xor Behavior.COHESION.get()
        }
    }

    private fun setSeparationOff() {
        if (isBehavior(Behavior.SEPARATION)) {
            behaviorFlag = behaviorFlag xor Behavior.SEPARATION.get()
        }
    }

    private fun setAlignmentOff() {
        if (isBehavior(Behavior.ALIGNMENT)) {
            behaviorFlag = behaviorFlag xor Behavior.ALIGNMENT.get()
        }
    }

    fun setObstacleAvoidanceOff() {
        if (isBehavior(Behavior.OBSTACLE_AVOIDANCE)) {
            behaviorFlag = behaviorFlag xor Behavior.OBSTACLE_AVOIDANCE.get()
        }
    }

    fun setWallAvoidanceOff() {
        if (isBehavior(Behavior.WALL_AVOIDANCE)) {
            behaviorFlag = behaviorFlag xor Behavior.WALL_AVOIDANCE.get()
        }
    }

    fun setFollowPathOff() {
        if (isBehavior(Behavior.FOLLOW_PATH)) {
            behaviorFlag = behaviorFlag xor Behavior.FOLLOW_PATH.get()
        }
    }

    fun setInterposeOff() {
        if (isBehavior(Behavior.INTERPOSE)) {
            behaviorFlag = behaviorFlag xor Behavior.INTERPOSE.get()
        }
    }

    fun setHideOff() {
        if (isBehavior(Behavior.HIDE)) {
            behaviorFlag = behaviorFlag xor Behavior.HIDE.get()
        }
    }

    fun setOffsetPursuitOff() {
        if (isBehavior(Behavior.OFFSET_PURSUIT)) {
            behaviorFlag = behaviorFlag xor Behavior.OFFSET_PURSUIT.get()
        }
    }

    fun setFlockingOff() {
        setCohesionOff()
        setAlignmentOff()
        setSeparationOff()
        setWanderOff()
    }

    val isFleeOn: Boolean
        get() = isBehavior(Behavior.FLEE)
    val isSeekOn: Boolean
        get() = isBehavior(Behavior.SEEK)
    val isArriveOn: Boolean
        get() = isBehavior(Behavior.ARRIVE)
    val isWanderOn: Boolean
        get() = isBehavior(Behavior.WANDER)
    val isPursuitOn: Boolean
        get() = isBehavior(Behavior.PURSUIT)

    fun setPursuitOn(vehicle: Vehicle) {
        behaviorFlag = behaviorFlag or Behavior.PURSUIT.get()
        targetAgent1 = vehicle
    }

    val isEvadeOn: Boolean
        get() = isBehavior(Behavior.EVADE)

    fun setEvadeOn(vehicle: Vehicle) {
        behaviorFlag = behaviorFlag or Behavior.EVADE.get()
        targetAgent1 = vehicle
    }

    val isCohesionOn: Boolean
        get() = isBehavior(Behavior.COHESION)
    val isSeparationOn: Boolean
        get() = isBehavior(Behavior.SEPARATION)
    val isAlignmentOn: Boolean
        get() = isBehavior(Behavior.ALIGNMENT)
    val isObstacleAvoidanceOn: Boolean
        get() = isBehavior(Behavior.OBSTACLE_AVOIDANCE)
    val isWallAvoidanceOn: Boolean
        get() = isBehavior(Behavior.WALL_AVOIDANCE)
    val isFollowPathOn: Boolean
        get() = isBehavior(Behavior.FOLLOW_PATH)
    val isInterposeOn: Boolean
        get() = isBehavior(Behavior.INTERPOSE)
    val isHideOn: Boolean
        get() = isBehavior(Behavior.HIDE)

    fun setHideOn(vehicle: Vehicle) {
        behaviorFlag = behaviorFlag or Behavior.HIDE.get()
        targetAgent1 = vehicle
    }

    val isOffsetPursuitOn: Boolean
        get() = isBehavior(Behavior.OFFSET_PURSUIT)

    companion object {
        // the radius of the constraining circle for to wander behavior
        const val WANDER_RADIUS = 1.2F

        // distance to wander circle is projected in front of the agent
        const val WANDER_DISTANCE = 2F

        // the maximum amount of displacement along the circle each frame
        const val WANDER_JITTER_PER_SECOND = 80F

        // used in path following
        const val WAYPOINT_SEEK_DISTANCE = 20F
    }

    init {
        detectBoxLength = paramLoader?.MIN_DETECTION_BOX_LENGTH ?: 0F
        cohesionWeight = paramLoader?.COHESION_WEIGHT ?: 0F
        alignmentWeight = paramLoader?.ALIGNMENT_WEIGHT ?: 0F
        separationWeight = paramLoader?.SEPARATION_WEIGHT ?: 0F
        weightObstacleAvoidance = paramLoader?.OBSTACLE_AVOIDANCE_WEIGHT ?: 0F
        weightWander = paramLoader?.WANDER_WEIGHT ?: 0F
        weightWallAvoidance = paramLoader?.WALL_AVOIDANCE_WEIGHT ?: 0F
        agentViewDistance = paramLoader?.VIEW_DISTANCE ?: 0F
        wallDetectionSensorLength = paramLoader?.WALL_DETECTION_FEELER_LENGTH ?: 0F
        sensors = ArrayList(3)
        deceleration = Deceleration.NORMAL
        wanderDistance = WANDER_DISTANCE
        wanderJitter = WANDER_JITTER_PER_SECOND
        wanderRadius = WANDER_RADIUS
        waypointSeekDistanceSqr = WAYPOINT_SEEK_DISTANCE * WAYPOINT_SEEK_DISTANCE
        weightSeek = paramLoader?.SEEK_WEIGHT ?: 0F
        weightFlee = paramLoader?.FLEE_WEIGHT ?: 0F
        weightArrive = paramLoader?.ARRIVE_WEIGHT ?: 0F
        weightPursuit = paramLoader?.PURSUIT_WEIGHT ?: 0F
        weightOffsetPursuit = paramLoader?.OFFSET_PURSUIT_WEIGHT ?: 0F
        weightInterpose = paramLoader?.INTERPOSE_WEIGHT ?: 0F
        weightHide = paramLoader?.HIDE_WEIGHT ?: 0F
        weightEvade = paramLoader?.EVADE_WEIGHT ?: 0F
        weightFollowPath = paramLoader?.FOLLOW_PATH_WEIGHT ?: 0F
        isSpacePartitioning = false
        summingMethod = SummingMethod.PRIORITIZED

        // stuff for to wander behavior
        val theta = MathUtility.randFloat() * MathUtility.TWO_PI

        // create a vector to a target position on to wander circle
        wanderTarget = Vector2.valueOf(
            (wanderRadius * cos(theta.toDouble())).toFloat(),
            (wanderRadius * sin(theta.toDouble())).toFloat()
        )

        // create a Path
        path = Path()
        path.enableLoop(true)
    }
}
