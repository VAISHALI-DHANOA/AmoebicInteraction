#include "Plankton.h"

void Plankton::setup(){
	r = ofRandom(3, 7);

	noiseVal = ofRandom(10,100);

	if ((ofRandom(0,1) <= 0.5)) type = 1;
	else type = 2;

	if (type == 1){
		color = ofColor(0,0,255);
	}
	else if (type == 2){
		color = ofColor(0,255,0);
	}

	startX = ofRandom(350, ofGetWidth()-400);
	startY = ofRandom(50, ofGetHeight()-200);

	//startX = ofRandom(ofGetWidth()/2-300, ofGetWidth()+100);
	//startY = ofRandom(ofGetHeight()/2-200, ofGetHeight()+200);

	randomFactorX = ofRandom(0.4, 2);
	randomFactorY = ofRandom(0.4, 2);

	toDelete = false;
	hited = false;

	movingRange = ofRandom(30, 50);
}

void Plankton::update(){
	x = startX + ofMap(ofNoise(noiseVal * randomFactorX), 0, 1, -movingRange, movingRange);
	y = startY + ofMap(ofNoise(noiseVal * randomFactorY), 0, 1, -movingRange, movingRange);

	noiseVal += 0.01;

	if (hited){
		 r++;
		 if (r > 60){
			 toDelete = true;
		 }
	}
}

void Plankton::draw(){
	ofSetColor(255,255,255);

	if (hited) ofNoFill();
	else ofFill();

	ofCircle(x, y, r);

}
