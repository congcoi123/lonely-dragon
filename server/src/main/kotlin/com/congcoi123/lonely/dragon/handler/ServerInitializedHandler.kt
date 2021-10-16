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
package com.congcoi123.lonely.dragon.handler

import com.congcoi123.lonely.dragon.constant.LDConstant
import com.congcoi123.lonely.dragon.entity.Vehicle
import com.congcoi123.lonely.dragon.utility.SharedEventKey
import com.congcoi123.lonely.dragon.world.World
import com.congcoi123.lonely.dragon.world.WorldListener
import com.tenio.common.bootstrap.annotation.AutowiredAcceptNull
import com.tenio.common.bootstrap.annotation.Component
import com.tenio.common.configuration.Configuration
import com.tenio.core.extension.AbstractExtension
import com.tenio.core.extension.events.EventServerInitialization
import com.tenio.engine.heartbeat.HeartBeatManager

@Component
class ServerInitializedHandler : AbstractExtension(), EventServerInitialization {

    @AutowiredAcceptNull
    private val heartBeatManager: HeartBeatManager? = null

    override fun handle(serverName: String, configuration: Configuration) {
        val world = World(LDConstant.DESIGN_WIDTH, LDConstant.DESIGN_HEIGHT)
        world.debug("Lonely Dragon")

        world.worldListener = object : WorldListener {
            override val ccu: Int
                get() = api().playerCount

            override fun updateVehiclePosition(vehicle: Vehicle) {
                val players = api().allPlayers
                val data = `object`()
                val array = mutableListOf<Int>()

                array.add(vehicle.index)
                array.add(vehicle.positionX.toInt())
                array.add(vehicle.positionY.toInt())
                array.add(vehicle.rotation.toInt())

                data.putIntegerArray(SharedEventKey.KEY_PLAYER_GET_RESPONSE, array)
                response().setRecipients(players).setContent(data.toBinary()).prioritizedUdp().write()
            }

            override fun responseVehicleNeighbours(playerName: String, neighbours: List<Vehicle>, currentFps: Int) {
                val player = api().getPlayerByName(playerName)
                player?.let {
                    val data = `object`()
                    data.putInteger(SharedEventKey.KEY_PLAYER_REQUEST_NEIGHBOURS, currentFps)
                    response().setRecipient(it).setContent(data.toBinary()).write()
                }
            }
        }

        try {
            heartBeatManager!!.initialize(1)
            heartBeatManager.create("world", world)
        } catch (e: Exception) {
            error(e, "world")
        }
    }
}
