#pragma once
#include "ofMain.h"

class Blob
{
public:
    Blob();
	void setup(unsigned int id, float x, float y);
	void update(float x, float y, float r);
	void draw();
	void exit();

	float x;
	float y;
	float r;
	unsigned int id;
	ofColor color;
	bool isOffscreen;

};

