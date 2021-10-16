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
package com.congcoi123.lonely.dragon

import kotlin.Throws
import kotlin.jvm.JvmStatic
import com.congcoi123.lonely.dragon.world.World
import com.congcoi123.lonely.dragon.constant.Example4Constant
import com.tenio.engine.heartbeat.HeartBeatManagerImpl
import java.lang.Exception

/**
 * Only for testing the movement behavior of vehicles.
 */
object TestMovementMechanism {
    @Throws(Exception::class)
    @JvmStatic
    fun main(args: Array<String>) {
        // Create a world
        val world = World(Example4Constant.DESIGN_WIDTH, Example4Constant.DESIGN_HEIGHT)
        // Enable debugger window
        world.debug("[TenIO] Server Debugger : Movement Simulation")
        val heartBeatManager = HeartBeatManagerImpl()
        heartBeatManager.initialize(1)
        heartBeatManager.create("world", world)
    }
}