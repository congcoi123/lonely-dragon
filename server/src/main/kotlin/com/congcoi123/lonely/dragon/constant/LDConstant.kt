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
package com.congcoi123.lonely.dragon.constant

class LDConstant private constructor() {
    companion object {
        const val DESIGN_WIDTH = 500
        const val DESIGN_HEIGHT = 500
        const val SOCKET_PORT = 8032
        const val DATAGRAM_PORT = 8034
        const val DELAY_CREATION = 0.1f

        // time in minutes
        const val AVERAGE_LATENCY_MEASUREMENT_INTERVAL = 1

        // time in seconds
        const val SEND_MEASUREMENT_REQUEST_INTERVAL = 20
        const val NUMBER_OF_PLAYERS = 100
        private const val ONE_SECOND_EXPECT_RECEIVE_PACKETS = 60
        const val ONE_MINUTE_EXPECT_RECEIVE_PACKETS = ONE_SECOND_EXPECT_RECEIVE_PACKETS * 60 * 100
    }

    init {
        throw UnsupportedOperationException()
    }
}
