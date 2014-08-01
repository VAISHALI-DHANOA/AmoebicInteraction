#pragma once
#include "ofMain.h"

class Plankton
{
public:
	void setup();
	void update();
	void draw();
	void exit();

	float startX;
	float startY;

	float x;
	float y;

	float r;
	ofPoint velocity;

	float randomFactorX;
	float randomFactorY;

	float noiseVal;

	bool toDelete;

	int type;

	bool hited;

	ofColor color;

	float movingRange;
};

