package com.congcoi123.lonely.dragon.utility

import com.tenio.common.configuration.ConfigurationType

/**
 * Create your own configurations.
 */
enum class LDConfigurationType(val value: String) : ConfigurationType {

    CUSTOM_VALUE_1("customValue1"),
    CUSTOM_VALUE_2("customValue2"),
    CUSTOM_VALUE_3("customValue3"),
    CUSTOM_VALUE_4("customValue4");

    companion object {
        // Reverse-lookup map for getting a type from a value
        private val lookup: MutableMap<String, LDConfigurationType> = HashMap()
        fun getByValue(value: String): LDConfigurationType? {
            return lookup[value]
        }

        init {
            for (configurationType in values()) {
                lookup[configurationType.value] = configurationType
            }
        }
    }

    override fun toString(): String {
        return name
    }
}
