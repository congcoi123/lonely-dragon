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
package com.congcoi123.lonely.dragon.utility

class SharedEventKey private constructor() {
    companion object {
        const val KEY_PLAYER_LOGIN = "u"
        const val KEY_ALLOW_TO_ATTACH = "a"
        const val KEY_CLIENT_SERVER_ECHO = "e"
        const val KEY_INTEGER_ARRAY = "i"
        const val KEY_PLAYER_POSITION = "p"
        const val KEY_PLAYER_REQUEST_NEIGHBOURS = "r"
        const val KEY_PLAYER_GET_RESPONSE = "rr"
    }

    init {
        throw UnsupportedOperationException(
            "This class does not support to create a new "
                    + "instance"
        )
    }
}
