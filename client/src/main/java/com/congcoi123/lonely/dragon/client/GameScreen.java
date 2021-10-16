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
package com.congcoi123.lonely.dragon.client;

import java.util.ArrayList;
import java.util.List;

import com.badlogic.gdx.graphics.g2d.Animation;
import com.badlogic.gdx.utils.Array;
import com.congcoi123.lonely.dragon.c2engine.asset.Asset;
import com.congcoi123.lonely.dragon.c2engine.asset.AssetManageable;
import com.congcoi123.lonely.dragon.c2engine.asset.FramesGenerator;
import com.congcoi123.lonely.dragon.c2engine.screen.XScreen;
import com.congcoi123.lonely.dragon.c2engine.sprite.SpriteAnimation;
import com.congcoi123.lonely.dragon.constant.Constant;
import com.congcoi123.lonely.dragon.constant.SharedEventKey;
import com.congcoi123.lonely.dragon.constant.UdpEstablishedState;
import com.congcoi123.lonely.dragon.network.DatagramListener;
import com.congcoi123.lonely.dragon.network.SocketListener;
import com.congcoi123.lonely.dragon.network.TCP;
import com.congcoi123.lonely.dragon.network.UDP;
import com.tenio.common.data.ZeroObject;
import com.tenio.common.data.implement.ZeroObjectImpl;
import com.tenio.core.entity.data.ServerMessage;

public class GameScreen extends XScreen implements AssetManageable, SocketListener, DatagramListener {

	private TCP tcp;
	private UDP udp;
	
	private String playerName;
	private int lastPositionX;
	private boolean flip;

	/**
	 * An array of all entities.
	 */
	private List<SpriteAnimation> spriteAnimations = new ArrayList<SpriteAnimation>();

	public GameScreen() {
		// create a new TCP object and listen for this port
		tcp = new TCP(Constant.SOCKET_PORT);
		tcp.receive(this);

		// create a new UDP object and listen for this port
		udp = new UDP(Constant.DATAGRAM_PORT);
		udp.receive(this);
		
		// initialization
		playerName = "kong";
		lastPositionX = 0;
		flip = false;

		// send a login request
		sendLoginRequest();
	}

	// networking
	@Override
	public void onReceivedTCP(ServerMessage message) {

		var data = (ZeroObject) message.getData();
		if (data.containsKey(SharedEventKey.KEY_ALLOW_TO_ATTACH)) {
			switch (data.getByte(SharedEventKey.KEY_ALLOW_TO_ATTACH)) {
			case UdpEstablishedState.ALLOW_TO_ATTACH: {
				// now you can send request for UDP connection request
				var sendData = ZeroObjectImpl.newInstance().putString(SharedEventKey.KEY_PLAYER_LOGIN, playerName);
				var request = ServerMessage.newInstance().setData(sendData);
				udp.send(request);

				System.out.println("Request UDP Connection -> " + request);
			}
				break;

			case UdpEstablishedState.ATTACHED: {
				System.out.println("Conversation ...");

			}
				break;

			}
		}
	}

	// networking
	@Override
	public void onReceivedUDP(ServerMessage message) {
		// System.err.println("Received UDP message -> " + message);
		var data = (ZeroObject) message.getData();

		if (data.containsKey(SharedEventKey.KEY_PLAYER_GET_RESPONSE)) {
			List<Integer> transform = (List<Integer>) data.getIntegerArray(SharedEventKey.KEY_PLAYER_GET_RESPONSE);
			int index = transform.get(0);
			int positionX = transform.get(1);
			int positionY = transform.get(2);

			if (lastPositionX > positionX) {
				if (!flip) {
					spriteAnimations.get(index).setFlipX(true);
					flip = true;
				}
			} else {
				if (flip) {
					spriteAnimations.get(index).setFlipX(false);
					flip = false;
				}
			}

			// a naive synchronous for testing ...
			spriteAnimations.get(index).setCenterXY(positionX, 500 - positionY);

			lastPositionX = positionX;
		}
	}

	// rendering
	@Override
	public void show() {
		initSpriteAnimation();
	}

	private void initSpriteAnimation() {
		for (int i = 0; i < 100; i++) {
			SpriteAnimation spriteAnimationSimple = new SpriteAnimation(
					FramesGenerator.getFramesFromTexture(Assets.TX_TEST_DRAGON, 1, 6));
			if (i == 99) {
				spriteAnimationSimple.resize(0.3f);
				spriteAnimationSimple.start(0.1f, Animation.LOOP);
			} else {
				spriteAnimationSimple.resize(0.05f);
			}
			spriteAnimations.add(spriteAnimationSimple);
		}
	}

	@Override
	public void resize(int width, int height) {

	}

	@Override
	public void hide() {

	}

	@Override
	public void pause() {

	}

	@Override
	public void resume() {

	}

	@Override
	public void dispose() {

	}

	@Override
	public Iterable<Asset> loadNeedAssets() {
		Array<Asset> assets = new Array<Asset>();
		assets.add(Assets.TX_TEST_BACKGROUND);
		assets.addAll(Assets.PACK_TEST_DRAGON);
		return assets;
	}

	@Override
	public Iterable<Asset> unloadAssets() {
		return null;
	}

	@Override
	public void draw(float delta) {
		// bring all to the screen
		batch.begin();
		spriteAnimations.forEach(spriteAnimation -> {
			spriteAnimation.draw(batch);
		});
		batch.end();
	}

	@Override
	public void update(float delta) {
		// just simple updating new position for all entities
		spriteAnimations.forEach(spriteAnimation -> {
			spriteAnimation.update(delta);
		});
	}

	private void sendLoginRequest() {
		var data = ZeroObjectImpl.newInstance();
		data.putString(SharedEventKey.KEY_PLAYER_LOGIN, playerName);
		tcp.send(ServerMessage.newInstance().setData(data));
	}
}
