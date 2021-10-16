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
package com.congcoi123.lonely.dragon.statistic

import java.util.ArrayList

class NetworkStatistic private constructor() {
    private val latencyRecorder: MutableList<Long>
    private val fpsRecorder: MutableList<Int>
    private val lostPacketRecorder: MutableList<Double>
    val latencyAverage: Double
        get() {
            synchronized(latencyRecorder) {
                var average: Long = 0
                val size = latencyRecorder.size
                for (i in 0 until size) {
                    average += latencyRecorder[i]
                }
                return average.toDouble() / size.toDouble()
            }
        }
    val latencySize: Int
        get() {
            synchronized(latencyRecorder) { return latencyRecorder.size }
        }

    fun addLatency(latency: Long) {
        synchronized(latencyRecorder) { latencyRecorder.add(latency) }
    }

    val fpsAverage: Double
        get() {
            synchronized(fpsRecorder) {
                var average = 0
                val size = fpsRecorder.size
                for (i in 0 until size) {
                    average += fpsRecorder[i]
                }
                return average.toDouble() / size.toDouble()
            }
        }
    val fpsSize: Int
        get() {
            synchronized(fpsRecorder) { return fpsRecorder.size }
        }

    fun addFps(fps: Int) {
        synchronized(fpsRecorder) { fpsRecorder.add(fps) }
    }

    val lostPacketsAverage: Double
        get() {
            synchronized(lostPacketRecorder) {
                var average = 0.0
                val size = lostPacketRecorder.size
                for (i in 0 until size) {
                    average += lostPacketRecorder[i]
                }
                return average / size.toDouble()
            }
        }
    val lostPacketsSize: Int
        get() {
            synchronized(lostPacketRecorder) { return lostPacketRecorder.size }
        }

    fun addLostPackets(packets: Double) {
        synchronized(lostPacketRecorder) { lostPacketRecorder.add(packets) }
    }

    companion object {
        @JvmStatic
        fun newInstance(): NetworkStatistic {
            return NetworkStatistic()
        }
    }

    init {
        latencyRecorder = ArrayList()
        fpsRecorder = ArrayList()
        lostPacketRecorder = ArrayList()
    }
}