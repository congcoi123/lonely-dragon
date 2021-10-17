package com.congcoi123.lonely.dragon.world

import com.congcoi123.lonely.dragon.configuration.ParamLoader.Companion.instance
import com.congcoi123.lonely.dragon.constant.SummingMethod
import com.congcoi123.lonely.dragon.entity.Obstacle
import com.congcoi123.lonely.dragon.entity.Vehicle
import com.congcoi123.lonely.dragon.entity.Wall
import com.tenio.common.utility.MathUtility
import com.tenio.common.worker.WorkerPool
import com.tenio.engine.heartbeat.AbstractHeartBeat
import com.tenio.engine.message.ExtraMessage
import com.tenio.engine.physic2d.common.BaseGameEntity
import com.tenio.engine.physic2d.common.InvertedAabbBox2D
import com.tenio.engine.physic2d.common.Path
import com.tenio.engine.physic2d.graphic.Paint
import com.tenio.engine.physic2d.graphic.window.Windows.P2Point
import com.tenio.engine.physic2d.math.Vector2
import com.tenio.engine.physic2d.utility.CellSpacePartition
import com.tenio.engine.physic2d.utility.EntitiesRelationship
import com.tenio.engine.physic2d.utility.Geometry
import com.tenio.engine.physic2d.utility.Smoother
import java.awt.Color

/**
 * All the environment data and methods for the Steering Behavior projects. This
 * class is the root of the project's update and render calls (excluding main of
 * course).
 */
class World(
    val clientX: Int, // client windows size
    val clientY: Int // client windows size
) : AbstractHeartBeat(clientX, clientY) {

    private val paramLoader = instance

    private val aabb = InvertedAabbBox2D.newInstance()

    // container containing any walls in the environment
    val walls = mutableListOf<Wall>()

    // a container of all the moving entities
    private val dragons = mutableListOf<Vehicle>()

    // any obstacles
    val obstacles = mutableListOf<BaseGameEntity>()
    val cellSpace: CellSpacePartition<Vehicle>

    // flags to turn aids and obstacles etc. on/off
    val isRenderWalls: Boolean = false
    private val isRenderObstacles: Boolean = false
    val isRenderPath: Boolean = false
    val isRenderDetectionBox: Boolean = false
    val isRenderWanderCircle: Boolean = false
    val isRenderSensors: Boolean = false
    val isRenderSteeringForce: Boolean = false
    private val enableShowCellSpaceInfo: Boolean
    private val frameRateSmoother = Smoother(SAMPLE_RATE, .0f)
    private val workerPool: WorkerPool
    var worldListener: WorldListener? = null

    // any path we may create for the vehicles to follow
    private val path: Path

    // set true to pause the motion
    var isPaused = false
        private set

    // the position of the cross-hair
    var crosshair: Vector2 = Vector2.valueOf((clientX / 2).toFloat(), (clientX / 2).toFloat())

    // keeps track of the average FPS
    private var fps: Float
    var isRenderFps: Boolean
        private set
    var isRenderNeighbors: Boolean
        private set
    var isViewKeys: Boolean
        private set
    private var sendingInterval = 0.0F

    fun nonPenetrationConstraint(vehicle: Vehicle) =
        EntitiesRelationship.enforceNonPenetrationConstraint(vehicle, dragons)

    fun tagVehiclesWithinViewRange(vehicle: BaseGameEntity, range: Float) =
        EntitiesRelationship.tagNeighbors(vehicle, dragons, range)

    fun tagObstaclesWithinViewRange(vehicle: BaseGameEntity, range: Float) =
        EntitiesRelationship.tagNeighbors(vehicle, obstacles, range)

    val agents: List<Vehicle>
        get() = dragons

    fun togglePause() {
        isPaused = !isPaused
    }

    /**
     * The user can set the position of the cross-hair by right-clicking the mouse.
     * This method makes sure the click is not inside any enabled Obstacles and sets
     * the position appropriately.
     */
    @Synchronized
    fun setCrossHair(p2Point: P2Point) {
        val proposedPosition = Vector2.valueOf(p2Point.x.toFloat(), p2Point.y.toFloat())

        // make sure it's not inside an obstacle
        val iterator = obstacles.listIterator()
        while (iterator.hasNext()) {
            val obstacle = iterator.next()
            if (Geometry.isPointInCircle(obstacle.position, obstacle.boundingRadius, proposedPosition)) {
                return
            }
        }
        crosshair.x = p2Point.x.toFloat()
        crosshair.y = p2Point.y.toFloat()
    }

    fun toggleShowFps() {
        isRenderFps = !isRenderFps
    }

    fun toggleRenderNeighbors() {
        isRenderNeighbors = !isRenderNeighbors
    }

    fun toggleViewKeys() {
        isViewKeys = !isViewKeys
    }

    /**
     * Creates some walls that form an enclosure for the steering agents. used to
     * demonstrate several of the steering behaviors.
     */
    private fun createWalls() {
        // create the walls
        val borderSize = 20F
        val cornerSize = 0.2F
        val vDist = clientY - 2 * borderSize
        val hDist = clientX - 2 * borderSize
        val walls = arrayOf(
            Vector2.valueOf(hDist * cornerSize + borderSize, borderSize),
            Vector2.valueOf(clientX - borderSize - hDist * cornerSize, borderSize),
            Vector2.valueOf(clientX - borderSize, borderSize + vDist * cornerSize),
            Vector2.valueOf(clientX - borderSize, clientY - borderSize - vDist * cornerSize),
            Vector2.valueOf(clientX - borderSize - hDist * cornerSize, clientY - borderSize),
            Vector2.valueOf(hDist * cornerSize + borderSize, clientY - borderSize),
            Vector2.valueOf(borderSize, clientY - borderSize - vDist * cornerSize),
            Vector2.valueOf(borderSize, borderSize + vDist * cornerSize)
        )
        val numWallVerts = walls.size
        for (w in 0 until numWallVerts - 1) {
            this.walls.add(Wall(walls[w].x, walls[w].y, walls[w + 1].x, walls[w + 1].y))
        }
        this.walls.add(
            Wall(walls[numWallVerts - 1].x, walls[numWallVerts - 1].y, walls[0].x, walls[0].y)
        )
    }

    /**
     * Sets up the vector of obstacles with random positions and sizes. Makes sure
     * the obstacles do not overlap.
     */
    private fun createObstacles() {
        // create a number of randomly sized tiddlywinks
        for (index in 0 until paramLoader!!.NUM_OBSTACLES) {
            var overlapped = true

            // keep creating tiddlywinks until we find one that doesn't overlap
            // any others.Sometimes this can get into an endless loop because the
            // obstacle has nowhere to fit. We test for this case and exit accordingly
            var numTrys = 0
            val numAllowableTrys = 2000
            while (overlapped) {
                numTrys++
                if (numTrys > numAllowableTrys) {
                    return
                }
                val radius = MathUtility.randInt(
                    paramLoader.MIN_OBSTACLE_RADIUS.toInt(),
                    paramLoader.MAX_OBSTACLE_RADIUS.toInt()
                )
                val border = 10
                val minGapBetweenObstacles = 20
                val obstacle = Obstacle(
                    MathUtility.randInt(radius + border, clientX - radius - border).toFloat(),
                    MathUtility.randInt(radius + border, clientY - radius - 30 - border).toFloat(), radius.toFloat()
                )
                if (!EntitiesRelationship.isOverlapped<BaseGameEntity, List<BaseGameEntity>>(
                        obstacle, obstacles,
                        minGapBetweenObstacles.toFloat()
                    )
                ) {
                    // it's not overlapped so we can add it
                    obstacles.add(obstacle)
                    overlapped = false
                }
            }
        }
    }

    // ------------------------------ Render ----------------------------------
    // ------------------------------------------------------------------------
    public override fun onRender(paint: Paint) {
        paint.enableOpaqueText(false)

        // render any walls
        paint.setPenColor(Color.BLACK)
        for (wall in walls) {
            wall.enableRenderNormal(true)
            wall.render(paint) // true flag shows normals
        }

        // render any obstacles
        paint.setPenColor(Color.BLACK)
        for (obstacle in obstacles) {
            paint.drawCircle(obstacle.position, obstacle.boundingRadius)
        }

        // render the agents
        for ((index, dragon) in dragons.withIndex()) {
            dragon.render(paint)

            // render cell partitioning stuff
            if (enableShowCellSpaceInfo && index == 0) {
                paint.setBgColor(null)

                val temp = Vector2.newInstance().set(dragon.position).sub(
                    paramLoader!!.VIEW_DISTANCE,
                    paramLoader.VIEW_DISTANCE
                )
                aabb.left = temp.x
                aabb.top = temp.y
                temp.set(dragon.position).add(
                    paramLoader.VIEW_DISTANCE,
                    paramLoader.VIEW_DISTANCE
                )
                aabb.right = temp.x
                aabb.bottom = temp.y
                aabb.render(paint)

                paint.setPenColor(Color.RED)
                cellSpace.calculateNeighbors(
                    dragon.position,
                    paramLoader.VIEW_DISTANCE
                )

                var frontOfNeighbor = cellSpace.frontOfNeighbor
                while (!cellSpace.isEndOfNeighbors) {
                    paint.drawCircle(frontOfNeighbor.position, frontOfNeighbor.boundingRadius)
                    frontOfNeighbor = cellSpace.nextOfNeighbor
                }
                paint.setPenColor(Color.GREEN)
                paint.drawCircle(dragon.position, paramLoader.VIEW_DISTANCE)
            }
        }
        val enableCrossHair = false
        if (enableCrossHair) {
            // and finally the cross-hair
            paint.setPenColor(Color.RED)
            paint.drawCircle(crosshair, 4F)
            paint.drawLine(crosshair.x - 8, crosshair.y, crosshair.x + 8, crosshair.y)
            paint.drawLine(crosshair.x, crosshair.y - 8, crosshair.x, crosshair.y + 8)
            paint.drawTextAtPosition(5, clientY - 20, "Click to move crosshair")
        }

        paint.setTextColor(Color.GRAY)
        if (isRenderPath) {
            paint.drawTextAtPosition((clientX / 2.0f - 80).toInt(), clientY - 20, "Press 'U' for random path")
            path.render(paint)
        }
        if (isRenderFps) {
            paint.setTextColor(Color.GRAY)
            paint.drawTextAtPosition(5, clientY - 20, (1 / fps).toString())
        }
        if (enableShowCellSpaceInfo) {
            cellSpace.render(paint)
        }
    }

    override fun onCreate() {}

    /**
     * Create a smoother to smooth the frame-rate.
     */
    override fun onUpdate(delta: Float) {
        sendingInterval += delta

        if (sendingInterval >= SEND_PACKETS_RATE) {
            worldListener?.let {
                val currentCCU = it.ccu
                setCcu(currentCCU)

                try {
                    workerPool.execute({
                        val vehicles = mutableListOf<Vehicle>()
                        synchronized(dragons) {
                            dragons.forEach(vehicles::add)
                        }

                        // update the vehicles
                        val iterator = vehicles.iterator()
                        while (iterator.hasNext()) {
                            val vehicle = iterator.next()
                            it.updateVehiclePosition(vehicle)
                        }
                    }, "update-vehicles-position")
                } catch (e: Exception) {
                    error(e)
                }
            }
            sendingInterval = 0F
        }

        fps = frameRateSmoother.update(delta)

        for (dragon in dragons) {
            dragon.update(delta)
        }
    }

    override fun onPause() {}
    override fun onResume() {}

    override fun onDispose() {
        workerPool.waitUntilAllTasksFinished()
        workerPool.stop()
    }

    override fun onAction1() =
        dragons.forEach { it.behavior.setSummingMethod(SummingMethod.PRIORITIZED) }

    override fun onAction2() =
        dragons.forEach { it.behavior.setSummingMethod(SummingMethod.WEIGHTED_AVERAGE) }

    override fun onAction3() =
        dragons.forEach { it.behavior.setSummingMethod(SummingMethod.DITHERED) }

    override fun onMessage(message: ExtraMessage) {
        worldListener?.let {
            val name = message.getContentByKey("id") as String
            val id = name.toInt()
            try {
                workerPool.execute({
                    var entityId = id
                    if (id > paramLoader!!.NUM_AGENTS - 1) {
                        entityId = paramLoader.NUM_AGENTS - 1
                    }
                    val neighbours = getNeighboursOf(entityId)
                    it.responseVehicleNeighbours(name, neighbours, getFps())
                }, name)
            } catch (e: Exception) {
                error(e)
            }
        }
    }

    private fun getNeighboursOf(index: Int): List<Vehicle> {
        val vehicles = mutableListOf<Vehicle>()
        val neighbours = mutableListOf<Vehicle>()

        synchronized(dragons) {
            dragons.forEach(vehicles::add)
        }

        vehicles.forEach {
            if (vehicles[index] != it) {
                if (vehicles[index].position.getDistanceValue(it.position) < 50) {
                    neighbours.add(it)
                }
            }
        }

        return neighbours
    }

    companion object {
        private const val ONE_SECOND_EXPECT_SEND_PACKETS = 10
        private const val SEND_PACKETS_RATE = 1F / ONE_SECOND_EXPECT_SEND_PACKETS.toFloat()
        private const val SAMPLE_RATE = 10
    }

    init {
        isRenderFps = true
        fps = 0F
        isRenderNeighbors = false
        isViewKeys = false
        enableShowCellSpaceInfo = false
        workerPool = WorkerPool("world", 150, 300)

        // set up the spatial subdivision class
        cellSpace = CellSpacePartition(
            clientX.toFloat(), clientY.toFloat(), paramLoader!!.NUM_CELLS_X,
            paramLoader.NUM_CELLS_Y, paramLoader.NUM_AGENTS
        )

        val border = 30f
        path = Path(5, border, border, clientX - border, clientY - border, true)
        for (index in 0 until paramLoader.NUM_AGENTS) {
            // determine a random starting position
            val spawnPosition = Vector2.valueOf(
                clientX / 2 + MathUtility.randomClamped() * clientX / 2,
                clientY / 2 + MathUtility.randomClamped() * clientY / 2
            )
            val dragon = Vehicle(
                world = this,
                position = spawnPosition,
                rotation = MathUtility.randFloat() * MathUtility.TWO_PI,
                velocity = Vector2.newInstance(),
                mass = paramLoader.VEHICLE_MASS,
                maxForce = paramLoader.MAX_STEERING_FORCE,
                maxSpeed = paramLoader.MAX_SPEED,
                maxTurnRate = paramLoader.MAX_TURN_RATE_PER_SECOND,
                scale = paramLoader.VEHICLE_SCALE
            )
            dragon.index = index

            // add to the list
            dragons.add(dragon)

            // add it to the cell subdivision
            cellSpace.addEntity(dragon)
        }

        val enableShoal = true
        if (enableShoal) {
            // predator
            dragons[paramLoader.NUM_AGENTS - 1].behavior.setFlockingOff()
            dragons[paramLoader.NUM_AGENTS - 1].behavior.setWanderOn()
            dragons[paramLoader.NUM_AGENTS - 1].id = "dragon"
            dragons[paramLoader.NUM_AGENTS - 1].scale = Vector2.valueOf(10f, 10f)
            dragons[paramLoader.NUM_AGENTS - 1].maxSpeed = 70f

            // preys
            for (i in 0 until paramLoader.NUM_AGENTS - 1) {
                dragons[i].behavior.setFlockingOn()
                dragons[i].behavior.setEvadeOn(dragons[paramLoader.NUM_AGENTS - 1])
            }
        }
    }
}
