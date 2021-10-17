package com.congcoi123.lonely.dragon.entity

import com.congcoi123.lonely.dragon.behavior.SteeringBehavior
import com.congcoi123.lonely.dragon.configuration.ParamLoader.Companion.instance
import com.congcoi123.lonely.dragon.world.World
import com.tenio.engine.fsm.entity.Telegram
import com.tenio.engine.physic2d.common.MoveableEntity
import com.tenio.engine.physic2d.graphic.Paint
import com.tenio.engine.physic2d.graphic.Renderable
import com.tenio.engine.physic2d.math.Vector2
import com.tenio.engine.physic2d.utility.SmootherVector
import com.tenio.engine.physic2d.utility.Transformation
import java.awt.Color
import kotlin.math.cos
import kotlin.math.sin

/**
 * Definition of a simple vehicle that uses steering behaviors.
 */
class Vehicle(
    val world: World,
    position: Vector2?,
    rotation: Float,
    velocity: Vector2?,
    mass: Float,
    maxForce: Float,
    maxSpeed: Float,
    maxTurnRate: Float,
    scale: Float
) : MoveableEntity(
    position,
    scale,
    velocity,
    maxSpeed,
    Vector2.valueOf(
        sin(rotation.toDouble()).toFloat(),
        (-cos(rotation.toDouble())).toFloat()
    ),
    mass,
    Vector2.valueOf(scale, scale),
    maxTurnRate,
    maxForce
), Renderable {

    // the steering behavior class
    val behavior: SteeringBehavior

    // some steering behaviors give jerky looking movement. The
    // following members are used to smooth the vehicle's heading
    private val headingSmoother: SmootherVector<Vector2>
    private val smoothedHeading: Vector2 = Vector2.newInstance()

    // buffer for the vehicle shape
    private val shape = mutableListOf<Vector2>()

    // this vector represents the average of the vehicle's heading
    // vector smoothed over the last few frames
    private var smoothedHeadingX = 0F
    private var smoothedHeadingY = 0F

    // when true, smoothing is active
    var isSmoothing = false
        private set

    // keeps a track of the most recent update time. (some
    // steering behaviors make use of this - see Wander)
    var timeElapsed = 0F
        private set

    // index of vehicle in the list
    var index = 0

    /**
     * Fills the vehicle's shape buffer with its vertices.
     */
    private fun createShape() {
        val numVehicleVerts = 3
        val vehicle = arrayOf(
            Vector2.valueOf(-1.0F, 0.6F),
            Vector2.valueOf(1.0F, 0.0F),
            Vector2.valueOf(-1.0F, -0.6F)
        )

        // set up the vertex buffers and calculate the bounding radius
        for (vtx in 0 until numVehicleVerts) {
            shape.add(vehicle[vtx])
        }
    }

    fun getSmoothedHeading(): Vector2 {
        return smoothedHeading.set(smoothedHeadingX, smoothedHeadingY)
    }

    fun setSmoothedHeading(smoothed: Vector2) {
        setSmoothedHeading(smoothed.x, smoothed.y)
    }

    private fun setSmoothedHeading(x: Float, y: Float) {
        smoothedHeadingX = x
        smoothedHeadingY = y
    }

    fun enableSmoothing(enabled: Boolean) {
        isSmoothing = enabled
    }

    fun toggleSmoothing() {
        isSmoothing = !isSmoothing
    }

    /**
     * Updates the vehicle's position and orientation from a series of steering
     * behaviors.
     */
    override fun update(delta: Float) {
        // update the time elapsed
        timeElapsed = delta

        // keep a record of its old position so we can update its cell later
        // in this method
        val oldPos = position

        // calculate the combined force from each steering behavior in the
        // vehicle's list
        val steeringForce = behavior.calculateAccumulate()
        // Vector2 steeringForce = new Vector2(1, 1);

        // Acceleration = Force/Mass
        val acceleration = steeringForce.div(mass)

        // update velocity
        val velocity = acceleration.mul(delta).add(velocity)

        // make sure vehicle does not exceed maximum velocity
        velocity.truncate(maxSpeed)
        setVelocity(velocity)

        // update the position
        val position = velocity.mul(delta).add(position)
        setPosition(position)

        // update the heading if the vehicle has a non-zero velocity
        if (getVelocity().lengthSqr > 0.00000001) {
            heading = getVelocity().normalize()
        }

        // treat the screen as a endless screen
        val around = Transformation.wrapAround(getPosition(), world.clientX, world.clientY)
        setPosition(around)

        // update the vehicle's current cell if space partitioning is turned on
        if (behavior.isSpacePartitioning) {
            world.cellSpace.updateEntity(this, oldPos)
        }
        if (isSmoothing) {
            setSmoothedHeading(headingSmoother.update(heading))
        }
    }

    override fun render(paint: Paint) {
        // render neighboring vehicles in different colors if requested
        if (world.isRenderNeighbors) {
            if (id === "dragon") {
                paint.setPenColor(Color.RED)
            } else if (isTagged) {
                paint.setPenColor(Color.GREEN)
            } else {
                paint.setPenColor(Color.BLUE)
            }
        } else {
            paint.setPenColor(Color.BLUE)
        }
        if (behavior.isInterposeOn) {
            paint.setPenColor(Color.RED)
        }
        if (behavior.isHideOn) {
            paint.setPenColor(Color.GREEN)
        }

        // a vector to hold the transformed vertices
        val shape: List<Vector2> = if (isSmoothing) {
            Transformation.pointsToWorldSpace(
                this.shape,
                position,
                getSmoothedHeading(),
                getSmoothedHeading().perpendicular(),
                scale
            )
        } else {
            Transformation.pointsToWorldSpace(
                this.shape,
                position,
                heading,
                side,
                scale
            )
        }
        paint.drawClosedShape(shape)

        // render any visual aids / and or user options
        if (world.isViewKeys) {
            behavior.render(paint)
        }
    }

    override fun handleMessage(msg: Telegram): Boolean {
        return false
    }

    init {
        createShape()

        // set up the steering behavior class
        behavior = SteeringBehavior(this)

        // set up the smoother
        headingSmoother = SmootherVector(
            instance!!.NUM_SAMPLES_FOR_SMOOTHING,
            Vector2.newInstance()
        )
    }
}
