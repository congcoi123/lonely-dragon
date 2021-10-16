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
package com.tenio.gdx.network;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import com.tenio.gdx.constant.Constants;
import com.tenio.gdx.network.entity.TObject;
import com.tenio.gdx.network.message.MessagePacker;
import com.tenio.gdx.network.message.MsgPackConverter;

/**
 * Create an object for handling a socket connection. It is used to send
 * messages to a server or receive messages from that one.
 * 
 * @author kong
 * 
 */
public class TCP {

	/**
	 * @see ISocketListener
	 */
	private ISocketListener __listener;
	/**
	 * @see Future
	 */
	private Future<?> __future;
	/**
	 * @see Socket
	 */
	private Socket __socket;
	/**
	 * @see DataOutputStream
	 */
	private DataOutputStream __out;
	/**
	 * @see DataInputStream
	 */
	private DataInputStream __in;
	/**
	 * The size of the received packet
	 */
	private short __dataSize = 0;
	/**
	 * This flag is used to determine how many numbers of received bytes can be used
	 * for one packet's header (that contains the packet's length)
	 */
	private boolean __recvHeader = true;

	/**
	 * Listen in a port on the local machine
	 * 
	 * @param port the desired port
	 */
	public TCP(int port) {
		try {
			__socket = new Socket("localhost", port);
			__out = new DataOutputStream(__socket.getOutputStream());
			__in = new DataInputStream(__socket.getInputStream());
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Send a message to the server
	 * 
	 * @param message the desired message @see {@link TObject}
	 */
	public void send(TObject message) {
		// convert message object to bytes data
		byte[] pack = MsgPackConverter.serialize(message);
		// attach the packet's length to packet's header
		byte[] bytes = MessagePacker.pack(pack);
		try {
			__out.write(bytes);
			__out.flush();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Listen for messages that came from the server
	 * 
	 * @param listener @see {@link ISocketListener}
	 */
	public void receive(ISocketListener listener) {
		__listener = listener;
		ExecutorService executorService = Executors.newSingleThreadExecutor();
		__future = executorService.submit(new Runnable() {
			@Override
			public void run() {
				byte[] buffer = new byte[10240];
				try {
					while (__in.read(buffer) > 0) {
						__onRecvData(buffer);
					}
				} catch (IOException e) {
					e.printStackTrace();
					return;
				}
			}
		});
	}

	private void __onRecvData(byte[] bytes) {
		if (bytes.length <= 0) {
			return;
		}

		if (__recvHeader) {
			__updateRecvHeaderData(bytes);
		} else {
			__updateRecvData(bytes);
		}
	}

	private void __updateRecvHeaderData(byte[] bytes) {
		if (bytes.length >= Constants.HEADER_BYTES) { // header length
			byte[] header = Arrays.copyOfRange(bytes, 0, Constants.HEADER_BYTES);
			__dataSize = MessagePacker.byteToShort(header); // network to host short
			__recvHeader = false;
			// package = |2 bytes header| <content bytes> |
			byte[] data = Arrays.copyOfRange(bytes, Constants.HEADER_BYTES, __dataSize + Constants.HEADER_BYTES);
			__onRecvData(data); // recursion
		}
	}

	private void __updateRecvData(byte[] bytes) {
		if (bytes.length >= __dataSize) {
			__onRecvMessage(bytes);
			__recvHeader = true; // reset header count
		}
	}

	private void __onRecvMessage(byte[] bytes) {
		// convert a received array of bytes to a message
		TObject message = MsgPackConverter.unserialize(bytes);
		__listener.onReceivedTCP(message);
	}

	/**
	 * Close this connection
	 */
	public void close() {
		try {
			__socket.close();
			__future.cancel(true);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}