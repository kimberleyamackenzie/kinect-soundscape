#pragma once

#include "ofMain.h"
#include "ofxOpenNI.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"

#define _USE_LIVE_VIDEO

//class KinectWrapper : public ofxOpenNI, public ofxKinect {
//    
//};

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    // for ofxKinect mesh drawing
    void drawPointCloud();

    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void handEvent(ofxOpenNIGestureEvent & event);

    
    ofSoundPlayer   tuning;
    vector<ofSoundPlayer>   pianoNotes;
    string note;
    
    ofPolyline line;
    
    vector<ofVec2f> mathVectors;

    int k;
    int frame;
    ofVec2f diff;
    ofVec2f position;
    float angle;
    float distance;

    ofxOpenNI kinect;
//    KinectWrapper kinect;

    bool mode;
    
    // for drawing the point cloud life
    
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    ofxCvGrayscaleImage     grayDiff;
    ofxCvGrayscaleImage     grayBg;
    
    ofxCvGrayscaleImage     grayCanny;
    ofxCvGrayscaleImage        debugImage;
    ofxCvGrayscaleImage        depthImg;
    ofxCvColorImage            depthImg2;
    ofxCvGrayscaleImage        cvimg;
    ofxCvGrayscaleImage        edges;
    ofxCvGrayscaleImage        depthOverlay;
//
    vector <ofPoint> contourReg;
    vector <ofPoint> contourSmooth;

    
    ofxCvContourFinder contourFinder;
    
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
    bool bLearnBackground;

    int thresholdValue;
    int nearThreshold;
    int farThreshold;
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
        
    int xPos;

    vector<ofPoint> pointCollection;
    vector<float> radii;
    vector<integer_t> xPositions;
    vector<integer_t> yPositions;

    float radius = 0.1;

//    int n;
    
    ofLight light;

    vector<ofColor>   colors;
    
    ofMesh backgroundMesh;
    ofMesh userMesh;
    ofImage image;

    
    
};

