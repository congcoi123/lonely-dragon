package com.congcoi123.lonely.dragon.world

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
import com.congcoi123.lonely.dragon.configuration.ParamLoader.Companion.instance
import com.congcoi123.lonely.dragon.constant.SummingMethod
import com.congcoi123.lonely.dragon.entity.Obstacle
import com.congcoi123.lonely.dragon.entity.Vehicle
import com.congcoi123.lonely.dragon.entity.Wall
import java.awt.Color
import java.util.function.Consumer

/**
 * All the environment data and methods for the Steering Behavior projects. This
 * class is the root of the project's update and render calls (excluding main of
 * course).
 */
class World(// local copy of client window dimensions
        val clientX: Int, val clientY: Int) : AbstractHeartBeat(clientX, clientY) {
    private val aabb = InvertedAabbBox2D.newInstance()

    // container containing any walls in the environment
    val walls = mutableListOf<Wall>()
    private val paramLoader = instance

    // a container of all the moving entities
    private val vehicles: MutableList<Vehicle> = ArrayList(paramLoader!!.NUM_AGENTS)

    // any obstacles
    val obstacles: MutableList<BaseGameEntity?> = ArrayList(paramLoader!!.NUM_OBSTACLES)
    val cellSpace: CellSpacePartition<Vehicle>

    // flags to turn aids and obstacles etc on/off
    val isRenderWalls: Boolean
    val isRenderObstacles: Boolean
    val isRenderPath: Boolean
    val isRenderDetectionBox: Boolean
    val isRenderWanderCircle: Boolean
    val isRenderSensors: Boolean
    val isRenderSteeringForce: Boolean
    private val enableShowCellSpaceInfo: Boolean
    private val frameRateSmoother = Smoother(SAMPLE_RATE, .0f)
    private val workerPool: WorkerPool

    // any path we may create for the vehicles to follow
    private var path: Path?

    // set true to pause the motion
    var isPaused = false
        private set

    // the position of the cross-hair
    var crosshair: Vector2

    // keeps track of the average FPS
    private var fps: Float
    var isRenderFPS: Boolean
        private set
    var isRenderNeighbors: Boolean
        private set
    var isViewKeys: Boolean
        private set
    private var sendingInterval = 0.0f
    private var worldListener: WorldListener? = null
    fun nonPenetrationConstraint(vehicle: Vehicle) {
        EntitiesRelationship.enforceNonPenetrationConstraint<Vehicle, List<Vehicle>>(vehicle, vehicles)
    }

    fun tagVehiclesWithinViewRange(vehicle: BaseGameEntity, range: Float) {
        EntitiesRelationship.tagNeighbors<BaseGameEntity, List<Vehicle>>(vehicle, vehicles, range)
    }

    fun tagObstaclesWithinViewRange(vehicle: BaseGameEntity, range: Float) {
        EntitiesRelationship.tagNeighbors<BaseGameEntity, List<BaseGameEntity?>>(vehicle, obstacles, range)
    }

    val agents: List<Vehicle>
        get() = vehicles

    fun togglePause() {
        isPaused = !isPaused
    }

    /**
     * The user can set the position of the crosshair by right clicking the mouse.
     * This method makes sure the click is not inside any enabled Obstacles and sets
     * the position appropriately.
     */
    @Synchronized
    fun setCrosshair(p: P2Point) {
        val proposedPosition = Vector2.valueOf(p.x.toFloat(), p.y.toFloat())

        // make sure it's not inside an obstacle
        val it: ListIterator<BaseGameEntity?> = obstacles.listIterator()
        while (it.hasNext()) {
            val curOb = it.next()
            if (Geometry.isPointInCircle(curOb!!.position, curOb.boundingRadius,
                            proposedPosition)) {
                return
            }
        }
        crosshair.x = p.x.toFloat()
        crosshair.y = p.y.toFloat()
    }

    fun toggleShowFPS() {
        isRenderFPS = !isRenderFPS
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
        val bordersize = 20f
        val cornerSize = 0.2f
        val vDist = clientY - 2 * bordersize
        val hDist = clientX - 2 * bordersize
        val walls = arrayOf(Vector2.valueOf(hDist * cornerSize + bordersize, bordersize),
                Vector2.valueOf(clientX - bordersize - hDist * cornerSize, bordersize),
                Vector2.valueOf(clientX - bordersize, bordersize + vDist * cornerSize),
                Vector2.valueOf(clientX - bordersize, clientY - bordersize - vDist * cornerSize),
                Vector2.valueOf(clientX - bordersize - hDist * cornerSize, clientY - bordersize),
                Vector2.valueOf(hDist * cornerSize + bordersize, clientY - bordersize),
                Vector2.valueOf(bordersize, clientY - bordersize - vDist * cornerSize),
                Vector2.valueOf(bordersize, bordersize + vDist * cornerSize))
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
     * the obstacles do not overlap
     */
    private fun createObstacles() {
        // create a number of randomly sized tiddlywinks
        for (o in 0 until paramLoader!!.NUM_OBSTACLES) {
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
                val radius = MathUtility.randInt(paramLoader.MIN_OBSTACLE_RADIUS.toInt(),
                        paramLoader.MAX_OBSTACLE_RADIUS.toInt())
                val border = 10
                val minGapBetweenObstacles = 20
                var ob: Obstacle? = Obstacle(MathUtility.randInt(radius + border, clientX - radius - border).toFloat(),
                        MathUtility.randInt(radius + border, clientY - radius - 30 - border).toFloat(), radius.toFloat())
                if (!EntitiesRelationship.isOverlapped<BaseGameEntity?, List<BaseGameEntity?>>(ob, obstacles,
                                minGapBetweenObstacles.toFloat())) {
                    // its not overlapped so we can add it
                    obstacles.add(ob)
                    overlapped = false
                } else {
                    ob = null
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
        for (w in walls.indices) {
            walls[w].enableRenderNormal(true)
            walls[w].render(paint) // true flag shows normals
        }

        // render any obstacles
        paint.setPenColor(Color.BLACK)
        for (ob in obstacles.indices) {
            paint.drawCircle(obstacles[ob]!!.position, obstacles[ob]!!.boundingRadius)
            System.err.println(obstacles[ob]!!.position.toString())
        }

        // render the agents
        for (a in vehicles.indices) {
            vehicles[a].render(paint)

            // render cell partitioning stuff
            if (enableShowCellSpaceInfo && a == 0) {
                paint.setBgColor(null)
                val temp = Vector2.newInstance().set(vehicles[a].position).sub(paramLoader!!.VIEW_DISTANCE,
                        paramLoader.VIEW_DISTANCE)
                aabb.left = temp.x
                aabb.top = temp.y
                temp.set(vehicles[a].position).add(paramLoader.VIEW_DISTANCE,
                        paramLoader.VIEW_DISTANCE)
                aabb.right = temp.x
                aabb.bottom = temp.y
                aabb.render(paint)
                paint.setPenColor(Color.RED)
                cellSpace.calculateNeighbors(vehicles[a].position,
                        paramLoader.VIEW_DISTANCE)
                var pV: BaseGameEntity = cellSpace.frontOfNeighbor
                while (!cellSpace
                                .isEndOfNeighbors) {
                    paint.drawCircle(pV.position, pV.boundingRadius)
                    pV = cellSpace.nextOfNeighbor
                }
                paint.setPenColor(Color.GREEN)
                paint.drawCircle(vehicles[a].position, paramLoader.VIEW_DISTANCE)
            }
        }
        val enableCrosshair = false
        if (enableCrosshair) {
            // and finally the cross-hair
            paint.setPenColor(Color.RED)
            paint.drawCircle(crosshair, 4f)
            paint.drawLine(crosshair.x - 8, crosshair.y, crosshair.x + 8, crosshair.y)
            paint.drawLine(crosshair.x, crosshair.y - 8, crosshair.x, crosshair.y + 8)
            paint.drawTextAtPosition(5, clientY - 20, "Click to move crosshair")
        }
        paint.setTextColor(Color.GRAY)
        if (isRenderPath) {
            paint.drawTextAtPosition((clientX / 2.0f - 80).toInt(), clientY - 20,
                    "Press 'U' for random path")
            path!!.render(paint)
        }
        if (isRenderFPS) {
            paint.setTextColor(Color.GRAY)
            paint.drawTextAtPosition(5, clientY - 20, (1 / fps).toString())
        }
        if (enableShowCellSpaceInfo) {
            cellSpace.render(paint)
        }
    }

    override fun onCreate() {}

    /**
     * Create a smoother to smooth the frame-rate
     */
    override fun onUpdate(delta: Float) {
        sendingInterval += delta
        if (sendingInterval >= SEND_PACKETS_RATE) {
            if (worldListener != null) {
                val currentCCU = worldListener!!.ccu
                setCcu(currentCCU)
                try {
                    workerPool.execute({
                        val vehicles: MutableList<Vehicle> = ArrayList()
                        synchronized(this.vehicles) { this.vehicles.forEach(Consumer { vehicle: Vehicle -> vehicles.add(vehicle) }) }

                        // update the vehicles
                        val iterator: Iterator<Vehicle> = vehicles.iterator()
                        while (iterator.hasNext()) {
                            val vehicle = iterator.next()
                            worldListener!!.updateVehiclePosition(vehicle)
                        }
                    }, "update-vehicles-position")
                } catch (e: Exception) {
                    error(e)
                }
            }
            sendingInterval = 0f
        }
        fps = frameRateSmoother.update(delta)
        for (i in vehicles.indices) {
            vehicles[i].update(delta)
        }
    }

    override fun onPause() {}
    override fun onResume() {}
    override fun onDispose() {
        workerPool.waitUntilAllTasksFinished()
        workerPool.stop()
    }

    override fun onAction1() {
        vehicles.forEach(Consumer { vehicle: Vehicle -> vehicle.behavior.setSummingMethod(SummingMethod.PRIORITIZED) })
    }

    override fun onAction2() {
        vehicles.forEach(Consumer { vehicle: Vehicle -> vehicle.behavior.setSummingMethod(SummingMethod.WEIGHTED_AVERAGE) })
    }

    override fun onAction3() {
        vehicles.forEach(Consumer { vehicle: Vehicle -> vehicle.behavior.setSummingMethod(SummingMethod.DITHERED) })
    }

    override fun onMessage(message: ExtraMessage) {
        // System.out.println("World._onMessage(): " + message.getContent().toString());
        if (worldListener == null) {
            return
        }
        val name = message.getContentByKey("id") as String
        val id = name.toInt()
        try {
            workerPool.execute({
                var entityId = id
                if (id > paramLoader!!.NUM_AGENTS - 1) {
                    entityId = paramLoader.NUM_AGENTS - 1
                }
                val neighbours = getNeighboursOf(entityId)
                worldListener!!.responseVehicleNeighbours(name, neighbours, getFps())
            }, name)
        } catch (e: Exception) {
            error(e)
        }
    }

    private fun getNeighboursOf(index: Int): List<Vehicle> {
        val vehicles: MutableList<Vehicle> = ArrayList()
        val neighbours: MutableList<Vehicle> = ArrayList()
        synchronized(this.vehicles) { this.vehicles.forEach(Consumer { vehicle: Vehicle -> vehicles.add(vehicle) }) }
        vehicles.forEach(Consumer { vehicle: Vehicle ->
            if (vehicles[index] != vehicle) {
                if (vehicles[index].position.getDistanceValue(vehicle.position) < 50) {
                    neighbours.add(vehicle)
                }
            }
        })
        return neighbours
    }

    var listener: WorldListener?
        get() = worldListener
        set(listener) {
            worldListener = listener
        }

    companion object {
        private const val ONE_SECOND_EXPECT_SEND_PACKETS = 10
        private const val SEND_PACKETS_RATE = 1.0f / ONE_SECOND_EXPECT_SEND_PACKETS.toFloat()
        private const val SAMPLE_RATE = 10
    }

    init {
        crosshair = Vector2.valueOf((clientX / 2).toFloat(), (clientX / 2).toFloat())
        isRenderWalls = false
        isRenderObstacles = false
        isRenderPath = false
        isRenderWanderCircle = false
        isRenderSteeringForce = false
        isRenderSensors = false
        isRenderDetectionBox = false
        isRenderFPS = true
        fps = 0f
        path = null
        isRenderNeighbors = false
        isViewKeys = false
        enableShowCellSpaceInfo = false
        workerPool = WorkerPool("world", 150, 300)

        // set up the spatial subdivision class
        cellSpace = CellSpacePartition(clientX.toFloat(), clientY.toFloat(), paramLoader!!.NUM_CELLS_X,
                paramLoader.NUM_CELLS_Y, paramLoader.NUM_AGENTS)
        val border = 30f
        path = Path(5, border, border, clientX - border, clientY - border, true)
        for (a in 0 until paramLoader.NUM_AGENTS) {
            // determine a random starting position
            val SpawnPos = Vector2.valueOf(clientX / 2 + MathUtility.randomClamped() * clientX / 2,
                    clientY / 2 + MathUtility.randomClamped() * clientY / 2)
            val pVehicle = Vehicle(this, SpawnPos,  // initial position
                    MathUtility.randFloat() * MathUtility.TWO_PI,  // start rotation
                    Vector2.newInstance(),  // velocity
                    paramLoader.VEHICLE_MASS,  // mass
                    paramLoader.MAX_STEERING_FORCE,  // max force
                    paramLoader.MAX_SPEED,  // max velocity
                    paramLoader.MAX_TURN_RATE_PER_SECOND,  // max turn rate
                    paramLoader.VEHICLE_SCALE) // scale
            pVehicle.index = a

            // Set the unique id for the big guy
            if (a == 0) {
                pVehicle.id = "dragon"
            }
            pVehicle.behavior.setFlockingOn()
            vehicles.add(pVehicle)

            // add it to the cell subdivision
            cellSpace.addEntity(pVehicle)
        }
        val enableShoal = true
        if (enableShoal) {
            vehicles[paramLoader.NUM_AGENTS - 1].behavior.setFlockingOff()
            vehicles[paramLoader.NUM_AGENTS - 1].scale = Vector2.valueOf(10f, 10f)
            vehicles[paramLoader.NUM_AGENTS - 1].behavior.setWanderOn()
            vehicles[paramLoader.NUM_AGENTS - 1].maxSpeed = 70f
            for (i in 0 until paramLoader.NUM_AGENTS - 1) {
                vehicles[i].behavior.setEvadeOn(vehicles[paramLoader.NUM_AGENTS - 1])
            }
        }
    }
}