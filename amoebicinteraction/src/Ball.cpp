#include "Ball.h"


void Ball::setup(){
	r = 10;
	x = 0;
	y = 0;
	velocity.x = ofRandom(1,10);
	velocity.y = ofRandom(1,10);	
}   

void Ball::update(){	
	x += velocity.x;
	y += velocity.y;

	if (x < 0 || x > ofGetWidth()) {
        velocity.x = - velocity.x;
    }
    
    if (y < 0 || y > ofGetHeight()) {
        velocity.y = -velocity.y;
    }      
}

void Ball::draw(){
	ofSetColor(255,255,0);
	ofFill();
	ofCircle(x, y, r);
}