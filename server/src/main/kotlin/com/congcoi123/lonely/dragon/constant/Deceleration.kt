package com.congcoi123.lonely.dragon.constant

enum class Deceleration(private val value: Int) {
    SLOW(3), NORMAL(2), FAST(1);

    fun get(): Int {
        return value
    }

    override fun toString(): String {
        return name
    }
}