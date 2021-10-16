package com.congcoi123.lonely.dragon.c2engine.asset;

/**
 * This class contain information about asset
 * 
 * @author mytv
 *
 */
public class Asset {
	public String pathName;
	public Class<?> clazz;

	public Asset(String pathName, Class<?> clazz) {
		this.pathName = pathName;
		this.clazz = clazz;
	}
}
