#pragma once
#include "ofMain.h"

class Ball
{
public:
	void setup();
	void update();
	void draw();
	void exit();

	float x;
	float y;
	float r;
	ofPoint velocity;	 
};

