package com.congcoi123.lonely.dragon.utility

import java.lang.UnsupportedOperationException

class UdpEstablishedState private constructor() {
    companion object {
        const val ALLOW_TO_ATTACH: Byte = 0
        const val ATTACHED: Byte = 1
    }

    init {
        throw UnsupportedOperationException()
    }
}