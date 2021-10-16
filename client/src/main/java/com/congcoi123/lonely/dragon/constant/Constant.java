/*
The MIT License

Copyright (c) 2016-2020 kong <congcoi123@gmail.com>

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
package com.congcoi123.lonely.dragon.constant;

/**
 * All base constants' values for the server are defined here. This class should
 * not be modified.
 */
public final class Constant {

	/**
	 * The size of screen in width, it should be same value as this one in server
	 * side.
	 */
	public static final int GAME_WIDTH = 500;

	/**
	 * The size of screen in height, it should be same value as this one in server
	 * side.
	 */
	public static final int GAME_HEIGHT = 500;

	/**
	 * In TCP, because of the stream transmission, it's necessary to know a data
	 * package's length for extracting the number of bytes of its content (divide
	 * stream data into smaller package data). Therefore, we need a data length
	 * value, which should be attached in the header of each package. All TCP
	 * connections which connect to our server must follow this rule.
	 */
	public static final int HEADER_BYTES = 2;

	/**
	 * The number of elements in a bulk those created for the first time. @see
	 * {@link IElementPool}
	 */
	public static final int BASE_ELEMENT_POOL = 32;

	/**
	 * When the desired number of elements exceeded the first configuration. The new
	 * number of elements will be added. @see {@link IElementPool}
	 */
	public static final int ADD_ELEMENT_POOL = 10;

	public static final int SOCKET_PORT = 8032;
	public static final int DATAGRAM_PORT = 8034;
}
