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

import com.tenio.core.extension.AbstractExtension
import com.tenio.core.extension.events.EventReceivedMessageFromPlayer
import com.tenio.common.bootstrap.annotation.AutowiredAcceptNull
import com.tenio.common.bootstrap.annotation.Component
import com.tenio.engine.heartbeat.HeartBeatManager
import com.tenio.core.entity.Player
import com.tenio.core.entity.data.ServerMessage
import com.tenio.common.data.ZeroObject
import com.congcoi123.lonely.dragon.utility.SharedEventKey
import com.congcoi123.lonely.dragon.utility.ExampleMessage
import com.congcoi123.lonely.dragon.constant.ServerEventKey

@Component
class ReceivedMessageFromPlayerHandler : AbstractExtension(), EventReceivedMessageFromPlayer {
    @AutowiredAcceptNull
    private val heartBeatManager: HeartBeatManager? = null
    override fun handle(player: Player, message: ServerMessage) {
        val data = message.data as ZeroObject
        if (data.containsKey(SharedEventKey.KEY_PLAYER_REQUEST_NEIGHBOURS)) {
            val request = ExampleMessage.newInstance()
            request.putContent(ServerEventKey.KEY_PLAYER_NAME, player.name)
            request.putContent(
                ServerEventKey.KEY_PLAYER_REQUEST,
                    data.getString(SharedEventKey.KEY_PLAYER_REQUEST_NEIGHBOURS))
            heartBeatManager!!.sendMessage("world", request)
        }
    }
}