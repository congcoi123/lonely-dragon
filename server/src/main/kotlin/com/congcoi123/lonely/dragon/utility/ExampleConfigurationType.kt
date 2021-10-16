package com.congcoi123.lonely.dragon.utility

import com.tenio.common.configuration.ConfigurationType
import java.util.HashMap

/**
 * Create your own configurations.
 *
 * @see ConfigurationType
 */
enum class ExampleConfigurationType(val value: String) : ConfigurationType {
    CUSTOM_VALUE_1("customValue1"), CUSTOM_VALUE_2("customValue2"), CUSTOM_VALUE_3("customValue3"), CUSTOM_VALUE_4("customValue4");

    companion object {
        // Reverse-lookup map for getting a type from a value
        private val lookup: MutableMap<String, ExampleConfigurationType> = HashMap()
        fun getByValue(value: String): ExampleConfigurationType? {
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