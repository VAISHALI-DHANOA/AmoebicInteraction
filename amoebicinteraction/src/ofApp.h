#pragma once

#include "Blob.h"
#include "Plankton.h"
#include "Ball.h"
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxCv.h"
#include "ofxOsc.h"
#include "ofxPostProcessing.h"

// Sending OSC messages or not
#define SEND_OSC true

// The IP and port of the computer playing sound
#define HOST "192.168.1.138"
#define PORT 12345

// The number of balls we want in the simulation
#define NUM_BALLS 10
#define NUM_PLANKTONS 100
#define MAX_NUM_BLOBS 3

#define SCALE_FACTOR 4

// The amount of world units the virtual camera moves by when pressing the 'w', 'a', 's', 'd' keys
#define VIRTUAL_CAMERA_TRANSLATION_STEP 100

// Whether to use a mouse controlled camera (easyCam) or a fixed camera
//#define USE_EASY_CAM 1

// Clipping planes for the real camera
#define NEAR_CLIP_REAL_CAMERA 2000
#define FAR_CLIP_REAL_CAMERA 10000

#define KINECT_ON true

class ofApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();
	void exit();

	void drawPointCloud();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	ofxKinect kinect;
	// Color image from the Kinect
	ofxCvColorImage colorImg;
    // Grayscale/depth image from the Kinect
	ofxCvGrayscaleImage grayImage, bgImage;

	bool bDrawPointCloud;

    ofCamera virtualCam;
    bool isVirtualCamInitialAngleSet;

	ofxCv::ContourFinder contourFinder;
	bool showLabels;

	ofFbo fboSmall, fboFinal;

	ofVboMesh mesh;
    int step;

	// BALLS
	void createBalls();
	void drawBalls();
	void updateBalls();
	std::vector<Ball> balls;

	// OSC
	ofxOscSender sender;
	void sendBlobsPositions();
	void sendCollision(int type);
	void sendEntered(int label);
	void sendExited(int label);

	// PLANKTON
	std::vector<Plankton> planktons;
	void createNewPlankton();
	void createPlanktons();
	void updatePlanktons();
	void drawPlanktons();

	// BLOBS
	std::vector<int> labels;
	std::vector<Blob> blobs;
	void findContours();
	void updateBlobs();
	void drawBlobs();
	void drawBlobs2();

	// COLLISION
	void checkCollisionAndUpdate();
	void checkCollisionAndUpdateWithBalls();
	bool isColliding(ofPoint c1, float r1, ofPoint c2, float r2);

	cv::RotatedRect rectcontour;

	ofColor blobcolor[5];

	ofImage planktonSkin1;
	ofImage planktonSkin2;

	bool captureBG;
	ofMesh triangularmesh;

    ofxPostProcessing post;

};
