package com.congcoi123.lonely.dragon.configuration

import com.tenio.common.utility.MathUtility
import java.io.IOException

/**
 * Class to configuration.
 */
class ParamLoader private constructor() : FileLoaderBase("src/main/resources/params.ini") {
    companion object {
        @JvmStatic
        var instance: ParamLoader? = null
            private set

        init {
            try {
                instance = ParamLoader()
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
    }

    @JvmField
    var NUM_AGENTS: Int = 0

    @JvmField
    var NUM_OBSTACLES: Int = 0

    @JvmField
    var MIN_OBSTACLE_RADIUS: Float = 0F

    @JvmField
    var MAX_OBSTACLE_RADIUS: Float = 0F

    // number of horizontal cells used for spatial partitioning
    @JvmField
    var NUM_CELLS_X: Int = 0

    // number of vertical cells used for spatial partitioning
    @JvmField
    var NUM_CELLS_Y: Int = 0

    // how many samples the smoother will use to average a value
    @JvmField
    var NUM_SAMPLES_FOR_SMOOTHING: Int = 0

    // used to tweak the combined steering force (simply altering the
    // MaxSteeringForce
    // will NOT work! This tweaker affects all the steering force multipliers too)
    var STEERING_FORCE_TWEAKER: Float = 0F

    @JvmField
    var MAX_STEERING_FORCE: Float = 0F

    @JvmField
    var MAX_SPEED: Float = 0F

    @JvmField
    var VEHICLE_MASS: Float = 0F

    @JvmField
    var VEHICLE_SCALE: Float = 0F

    @JvmField
    var MAX_TURN_RATE_PER_SECOND: Float = 0F
    var SEPARATION_WEIGHT: Float = 0F
    var ALIGNMENT_WEIGHT: Float = 0F
    var COHESION_WEIGHT: Float = 0F
    var OBSTACLE_AVOIDANCE_WEIGHT: Float = 0F
    var WALL_AVOIDANCE_WEIGHT: Float = 0F
    var WANDER_WEIGHT: Float = 0F
    var SEEK_WEIGHT: Float = 0F
    var FLEE_WEIGHT: Float = 0F
    var ARRIVE_WEIGHT: Float = 0F
    var PURSUIT_WEIGHT: Float = 0F
    var OFFSET_PURSUIT_WEIGHT: Float = 0F
    var INTERPOSE_WEIGHT: Float = 0F
    var HIDE_WEIGHT: Float = 0F
    var EVADE_WEIGHT: Float = 0F
    var FOLLOW_PATH_WEIGHT: Float = 0F

    // how close a neighbor must be before an agent perceives it (considers it
    // to be within its neighborhood)
    @JvmField
    var VIEW_DISTANCE: Float = 0F

    // used in obstacle avoidance
    var MIN_DETECTION_BOX_LENGTH: Float = 0F

    // used in wall avoidance
    var WALL_DETECTION_FEELER_LENGTH: Float = 0F

    // these are the probabilities that a steering behavior will be used
    // when the prioritized dither calculate method is used
    var PR_WALL_AVOIDANCE: Float = 0F
    var PR_OBSTACLE_AVOIDANCE: Float = 0F
    var PR_SEPARATION: Float = 0F
    var PR_ALIGNMENT: Float = 0F
    var PR_COHESION: Float = 0F
    var PR_WANDER: Float = 0F
    var PR_SEEK: Float = 0F
    var PR_FLEE: Float = 0F
    var PR_EVADE: Float = 0F
    var PR_HIDE: Float = 0F
    var PR_ARRIVE: Float = 0F

    init {
        NUM_AGENTS = nextParameterInt
        NUM_OBSTACLES = nextParameterInt
        MIN_OBSTACLE_RADIUS = nextParameterFloat
        MAX_OBSTACLE_RADIUS = nextParameterFloat
        NUM_CELLS_X = nextParameterInt
        NUM_CELLS_Y = nextParameterInt
        NUM_SAMPLES_FOR_SMOOTHING = nextParameterInt
        STEERING_FORCE_TWEAKER = nextParameterFloat
        MAX_STEERING_FORCE = nextParameterFloat * STEERING_FORCE_TWEAKER
        MAX_SPEED = nextParameterFloat
        VEHICLE_MASS = nextParameterFloat
        VEHICLE_SCALE = nextParameterFloat
        SEPARATION_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        ALIGNMENT_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        COHESION_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        OBSTACLE_AVOIDANCE_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        WALL_AVOIDANCE_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        WANDER_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        SEEK_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        FLEE_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        ARRIVE_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        PURSUIT_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        OFFSET_PURSUIT_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        INTERPOSE_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        HIDE_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        EVADE_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        FOLLOW_PATH_WEIGHT = nextParameterFloat * STEERING_FORCE_TWEAKER
        VIEW_DISTANCE = nextParameterFloat
        MIN_DETECTION_BOX_LENGTH = nextParameterFloat
        WALL_DETECTION_FEELER_LENGTH = nextParameterFloat
        PR_WALL_AVOIDANCE = nextParameterFloat
        PR_OBSTACLE_AVOIDANCE = nextParameterFloat
        PR_SEPARATION = nextParameterFloat
        PR_ALIGNMENT = nextParameterFloat
        PR_COHESION = nextParameterFloat
        PR_WANDER = nextParameterFloat
        PR_SEEK = nextParameterFloat
        PR_FLEE = nextParameterFloat
        PR_EVADE = nextParameterFloat
        PR_HIDE = nextParameterFloat
        PR_ARRIVE = nextParameterFloat
        MAX_TURN_RATE_PER_SECOND = MathUtility.PI
    }
}