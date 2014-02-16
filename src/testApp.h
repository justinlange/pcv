#pragma once

#include "ofMain.h"
#include "ofxOculusRift.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"

typedef struct{
	ofColor color;
	ofVec3f pos;
	ofVec3f floatPos;
	float radius;
    bool bHighlighted;
} DemoSphere;

class testApp : public ofBaseApp
{
  public:
	
	void setup();
	void update();
	void draw();
	
	void drawScene();
	
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofxOculusRift		oculusRift;

	ofLight				light;
	ofEasyCam			cam;
	bool showOverlay;
	bool predictive;
	vector<DemoSphere> demos;
    
    ofVec3f cursor2D;
    ofVec3f cursor3D;
    
    ofVec3f cursorRift;
    ofVec3f demoRift;
    
    //-------kinect-------//
    ofxKinect kinect;
    ofMesh mesh;
    ofEasyCam easyCam;
    
    void drawPointCloud();
    bool loadKinect;
    
    bool bDrawPointCloud;
	
    int nearThreshold;
    int farThreshold;
	
    int angle;
    ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
    int rotateY;
    
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	

};