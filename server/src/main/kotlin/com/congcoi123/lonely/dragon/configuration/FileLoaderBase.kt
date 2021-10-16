package com.congcoi123.lonely.dragon.configuration

import java.io.BufferedReader
import kotlin.Throws
import java.io.IOException
import java.lang.RuntimeException
import java.io.FileReader
import java.io.FileNotFoundException
import java.util.regex.Pattern

/**
 * This class is used for configuration loader.
 */
abstract class FileLoaderBase(filename: String?) {
    private var file: BufferedReader? = null
    private var line: String? = ""
    var isFileIsGood: Boolean
    private fun getParameterValueAsString(line: String?): String {
        // define some delimiters
        val delims = "[ ;=,]"
        val pattern = Pattern.compile(delims)
        val s = pattern.split(line)
        return if (s.size > 0) {
            s[s.size - 1]
        } else ""
    }
    // this will be the string that holds the next parameter

    // if the line is of zero length, get the next line from
    // the file
    @get:Throws(IOException::class)
    private val nextParameter: String?
        private get() {
            // this will be the string that holds the next parameter
            var line: String? = null
            line = file!!.readLine()
            line = removeCommentingFromLine(line)

            // if the line is of zero length, get the next line from
            // the file
            if (line!!.length == 0) {
                return nextParameter
            }
            line = getParameterValueAsString(line)
            return line
        }// strip the token from the line
    // strip the line of any commenting

    // find beginning of parameter description

    // define some delimiters
    @get:Throws(IOException::class)
    private val nextToken: String
        // find the end of the parameter description
        private get() {
            // strip the line of any commenting
            while (line == "") {
                line = file!!.readLine()
                line = removeCommentingFromLine(line)
            }

            // find beginning of parameter description
            var begIdx = line!!.length
            var endIdx = line!!.length

            // define some delimiters
            val delims = "[ ;=,]+"
            val pattern = Pattern.compile(delims)
            val matcher = pattern.matcher(line)

            // find the end of the parameter description
            if (matcher.find()) {
                begIdx = matcher.end()
                endIdx = if (matcher.find()) {
                    matcher.start()
                } else {
                    line!!.length
                }
            }
            val s = line!!.substring(begIdx, endIdx)
            line = if (endIdx != line!!.length) {
                // strip the token from the line
                line!!.substring(endIdx + 1)
            } else {
                ""
            }
            return s
        }

    // helper methods. They convert the next parameter value found into the
    // relevant type
    @get:Throws(IOException::class)
    val nextParameterDouble: Double
        get() {
            if (isFileIsGood) {
                return java.lang.Double.valueOf(nextParameter)
            }
            throw RuntimeException("bad file")
        }

    @get:Throws(IOException::class)
    val nextParameterFloat: Float
        get() {
            if (isFileIsGood) {
                return java.lang.Float.valueOf(nextParameter)
            }
            throw RuntimeException("bad file")
        }

    @get:Throws(IOException::class)
    val nextParameterInt: Int
        get() {
            if (isFileIsGood) {
                return Integer.valueOf(nextParameter)
            }
            throw RuntimeException("bad file")
        }

    @get:Throws(IOException::class)
    val nextParameterBool: Boolean
        get() {
            if (isFileIsGood) {
                return 0 != Integer.valueOf(nextParameter)
            }
            throw RuntimeException("bad file")
        }

    @get:Throws(IOException::class)
    val nextTokenAsDouble: Double
        get() {
            if (isFileIsGood) {
                return java.lang.Double.valueOf(nextToken)
            }
            throw RuntimeException("bad file")
        }

    @get:Throws(IOException::class)
    val nextTokenAsFloat: Float
        get() {
            if (isFileIsGood) {
                return java.lang.Float.valueOf(nextToken)
            }
            throw RuntimeException("bad file")
        }

    @get:Throws(IOException::class)
    val nextTokenAsInt: Int
        get() {
            if (isFileIsGood) {
                return Integer.valueOf(nextToken)
            }
            throw RuntimeException("bad file")
        }

    @get:Throws(IOException::class)
    val nextTokenAsString: String
        get() {
            if (isFileIsGood) {
                return nextToken
            }
            throw RuntimeException("bad file")
        }

    @get:Throws(IOException::class)
    val isEOF: Boolean
        get() {
            if (isFileIsGood) {
                return !file!!.ready()
            }
            throw RuntimeException("bad file")
        }

    companion object {
        // removes any commenting from a line of text
        fun removeCommentingFromLine(line: String?): String? {
            // search for any comment and remove
            val idx = line!!.indexOf("//")
            return if (idx != -1) {
                // cut out the comment
                line.substring(0, idx)
            } else line
        }
    }

    init {
        line = ""
        isFileIsGood = true
        try {
            file = BufferedReader(FileReader(filename))
        } catch (ex: FileNotFoundException) {
            isFileIsGood = false
        }
    }
}