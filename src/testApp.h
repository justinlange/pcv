#pragma once

#include "ofMain.h"
#include "ofxOculusRift.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxObjLoader.h"

//#define USE_TWO_KINECTS
//#define USE_THREE_KINECTS



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
    void exit();

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
    ofMesh mesh;
    
    void prepPointCloud();
    ofxKinect kinect;
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
    ofMesh mesh2;
#endif
#ifdef USE_THREE_KINECTS
	ofxKinect kinect3;
    ofMesh mesh3;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    //-------OBJ--------//
    
    void loadCow();
    ofMesh objFile;
    void drawCow();
    
    ofShader shader;

	

};
