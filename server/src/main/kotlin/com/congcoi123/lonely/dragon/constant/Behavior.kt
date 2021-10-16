package com.congcoi123.lonely.dragon.constant

enum class Behavior(private val value: Int) {
    NONE(0x00000), SEEK(0x00002), FLEE(0x00004), ARRIVE(0x00008), WANDER(0x00010), COHESION(0x00020), SEPARATION(0x00040), ALIGNMENT(0x00080), OBSTACLE_AVOIDANCE(0x00100), WALL_AVOIDANCE(0x00200), FOLLOW_PATH(0x00400), PURSUIT(0x00800), EVADE(0x01000), INTERPOSE(0x02000), HIDE(0x04000), FLOCK(0x08000), OFFSET_PURSUIT(0x10000);

    fun get(): Int {
        return value
    }

    override fun toString(): String {
        return name
    }
}