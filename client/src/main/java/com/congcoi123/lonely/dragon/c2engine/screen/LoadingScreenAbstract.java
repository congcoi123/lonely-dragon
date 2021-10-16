package com.congcoi123.lonely.dragon.c2engine.screen;

import com.badlogic.gdx.Game;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.math.Interpolation;
import com.congcoi123.lonely.dragon.c2engine.asset.Asset;
import com.congcoi123.lonely.dragon.c2engine.asset.AssetManageable;
import com.congcoi123.lonely.dragon.c2engine.asset.ResourceManager;

public abstract class LoadingScreenAbstract implements Screen {

	protected float progress;
	private Screen nextScreen;
	private Game game;

	public void createGame(Game game) {
		this.game = game;
		preload();
		progress = 0;
	}

	public void setNextScreen(Screen screen) {
		// unload last screen resource if need
		if (nextScreen != null && nextScreen instanceof AssetManageable) {
			Iterable<Asset> unloadAssets = ((AssetManageable) nextScreen).unloadAssets();
			if (unloadAssets != null)
				ResourceManager.unloadAssets(unloadAssets);
		}

		// load next screen resource if need
		if (screen instanceof AssetManageable) {
			Iterable<Asset> loadAssets = ((AssetManageable) screen).loadNeedAssets();
			if (loadAssets != null)
				ResourceManager.loadAssets(loadAssets);
		}

		nextScreen = screen;
	}

	public void render(float delta) {
		onRender(delta);

		// update process
		progress = Interpolation.linear.apply(progress, ResourceManager.getProgress(), 0.02f);

		// if unload and load is done, automatically set to next screen && progress >
		// 0.99f
		if (ResourceManager.isLoadDone()) {
			this.game.setScreen(nextScreen);
		}
	}

	public abstract void preload();

	// draw background, animation, anything else let code in below. It is
	// abstract because it's up to ideal of design
	public abstract void onRender(float render);
}
