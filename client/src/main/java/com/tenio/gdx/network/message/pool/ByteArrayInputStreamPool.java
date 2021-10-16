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
package com.tenio.gdx.network.message.pool;

import com.tenio.gdx.constant.Constants;
import com.tenio.gdx.exception.NullElementPoolException;

/**
 * @see {@link IElementPool}
 * 
 * @author kong
 * 
 */
public final class ByteArrayInputStreamPool implements IElementPool<ByteArrayInputStream> {

	private static volatile ByteArrayInputStreamPool __instance;

	// preventing Singleton object instantiation from outside
	// creates multiple instance if two thread access this method simultaneously
	public static ByteArrayInputStreamPool getInstance() {
		if (__instance == null) {
			__instance = new ByteArrayInputStreamPool();
		}
		return __instance;
	}

	private ByteArrayInputStream[] __pool;
	private boolean[] __used;

	public ByteArrayInputStreamPool() {
		__pool = new ByteArrayInputStream[Constants.BASE_ELEMENT_POOL];
		__used = new boolean[Constants.BASE_ELEMENT_POOL];

		for (int i = 0; i < __pool.length; i++) {
			__pool[i] = new ByteArrayInputStream();
			__used[i] = false;
		}
	}

	@Override
	public synchronized ByteArrayInputStream get() {
		for (int i = 0; i < __used.length; i++) {
			if (!__used[i]) {
				__used[i] = true;
				return __pool[i];
			}
		}
		// If we got here, then all the Elements are in use. We will
		// increase the number in our pool by @ADD_ELEMENT_POOL (arbitrary value for
		// illustration purposes).
		boolean[] oldUsed = __used;
		__used = new boolean[oldUsed.length + Constants.ADD_ELEMENT_POOL];
		System.arraycopy(oldUsed, 0, __used, 0, oldUsed.length);

		ByteArrayInputStream[] oldPool = __pool;
		__pool = new ByteArrayInputStream[oldPool.length + Constants.ADD_ELEMENT_POOL];
		System.arraycopy(oldPool, 0, __pool, 0, oldPool.length);

		for (int i = oldPool.length; i < __pool.length; i++) {
			__pool[i] = new ByteArrayInputStream();
			__used[i] = false;
		}

		// and allocate the last old ELement
		__used[oldPool.length - 1] = true;
		return __pool[oldPool.length - 1];
	}

	@Override
	public synchronized void repay(ByteArrayInputStream element) {
		try {
			for (int i = 0; i < __pool.length; i++) {
				if (__pool[i] == element) {
					__used[i] = false;
					return;
				}
			}
			throw new NullElementPoolException();
		} catch (NullElementPoolException e) {
			e.printStackTrace();
		}
	}

}
