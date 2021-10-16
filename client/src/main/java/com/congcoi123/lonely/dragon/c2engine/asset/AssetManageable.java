package com.congcoi123.lonely.dragon.c2engine.asset;

/**
 * The object which other can get asset from that.
 */
public interface AssetManageable {

	public Iterable<Asset> loadNeedAssets();

	public Iterable<Asset> unloadAssets();
}
