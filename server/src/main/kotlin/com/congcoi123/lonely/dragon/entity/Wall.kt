/*
The MIT License

Copyright (c) 2016-2021 kong <congcoi123@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
package com.congcoi123.lonely.dragon.entity

import com.tenio.engine.physic2d.graphic.Paint
import com.tenio.engine.physic2d.graphic.Renderable
import com.tenio.engine.physic2d.math.Vector2

/**
 * This class is used to create and render 2D walls. Defined as the two vectors
 * A - B with a perpendicular normal.
 */
class Wall : Renderable {
    var fromX = 0f
        private set
    var fromY = 0f
        private set
    private val vectorA: Vector2? = null
    var toX = 0f
        private set
    var toY = 0f
        private set
    private val vectorB: Vector2? = null
    var normalX = 0f
        private set
    var normalY = 0f
        private set
    private val vectorN: Vector2? = null
    private var vcx = 0f
    private var vcy = 0f
    private val vectorC: Vector2? = null
    private var renderNormal = false

    constructor() {}
    constructor(aX: Float, aY: Float, bX: Float, bY: Float) {
        fromX = aX
        fromY = aY
        toX = bX
        toY = bY
        calculateNormal()
    }

    constructor(aX: Float, aY: Float, bX: Float, bY: Float, nX: Float, nY: Float) {
        fromX = aX
        fromY = aY
        toX = bX
        toY = bY
        normalX = nX
        normalY = nY
    }

    private fun calculateNormal() {
        val temp = Vector2.newInstance().set(toX, toY).sub(fromX, fromY).normalize()
        normalX = -temp.y
        normalY = temp.x
        temp.set(fromX, fromY).add(toX, toY).div(2f)
        vcx = temp.x
        vcy = temp.y
    }

    val from: Vector2
        get() = vectorA!!.set(fromX, fromY)

    fun setFrom(x: Float, y: Float) {
        fromX = x
        fromY = y
        calculateNormal()
    }

    val to: Vector2
        get() = vectorB!!.set(toX, toY)

    fun setTo(x: Float, y: Float) {
        toX = x
        toY = y
        calculateNormal()
    }

    val normal: Vector2
        get() = vectorN!!.set(normalX, normalY)

    fun setNormal(x: Float, y: Float) {
        normalX = x
        normalY = y
    }

    val center: Vector2
        get() = vectorC!!.set(vcx, vcy)

    fun enableRenderNormal(enabled: Boolean) {
        renderNormal = enabled
    }

    override fun render(paint: Paint) {
        paint.drawLine(fromX, fromY, toX, toY)

        // render the normals
        if (renderNormal) {
            val midX = ((fromX + toX) / 2).toInt()
            val midY = ((fromY + toY) / 2).toInt()
            paint.drawLine(midX, midY, (midX + normalX * 5).toInt(), (midY + normalY * 5).toInt())
        }
    }
}