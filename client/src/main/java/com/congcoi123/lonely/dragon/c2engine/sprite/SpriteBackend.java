package com.congcoi123.lonely.dragon.c2engine.sprite;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;

/**
 * A backend of all sprite make them become easy to manage
 * 
 * @FileName: SpriteBackend.java
 * @CreateOn: Sep 23, 2012 - 2:49:59 PM
 * @Author: TrungNT
 */
public interface SpriteBackend {

	// =================================================
	// setter

	public void setBounds(float x, float y, float width, float height);

	public void setSize(float width, float height);

	public void setPosition(float x, float y);

	public void setX(float x);

	public void setY(float y);

	public void translate(float xAmount, float yAmount);

	public void translateX(float xAmount);

	public void translateY(float yAmount);

	public void setOrigin(float originX, float originY);

	public void setRotation(float degree);

	public void rotate(float degree);

	public void setScale(float scaleXY);

	public void setScale(float scaleX, float scaleY);

	public void scale(float amount);

	public void setColor(float r, float g, float b, float a);

	public void setColor(Color color);

	// =================================================
	// getter

	public float[] getVertices();

	public float getCenterX();

	public float getCenterY();

	public float getX();

	public float getY();

	public float getWidth();

	public float getHeight();

	public float getOriginX();

	public float getOriginY();

	public float getRotation();

	public float getScaleX();

	public float getScaleY();

	public void update(float delta);

	public void draw(SpriteBatch batch);

	public void draw(SpriteBatch batch, float alpha);

}
