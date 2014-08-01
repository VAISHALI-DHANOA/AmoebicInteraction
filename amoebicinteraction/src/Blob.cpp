#include "Blob.h"

Blob::Blob(){
    isOffscreen = true;
}

void Blob::setup(unsigned int id, float x, float y){
	this->id = id;

	r = 10;
	this->x = x;
	this->y = y;
    isOffscreen =false;
	//color = ofColor(ofRandom(255),ofRandom(255),ofRandom(255));
}

void Blob::update(float x, float y, float r){
	this->x += (x - this->x ) * 0.1;
	this->y += (y - this->y ) * 0.1;
	this->r += (r - this->r ) * 0.1;
}

void Blob::draw(){
	ofSetColor(color);
	ofFill();
	ofCircle(x, y, r);
}
