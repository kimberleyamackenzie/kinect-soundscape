#pragma once

#include "ofMain.h"
#include "ofxOpenNI.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"


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

    bool mode;
    
////     mesh drawing!
//    ofxCvColorImage colorImg;
//
    ofxCvGrayscaleImage grayImage; // grayscale depth image
//    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
//    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
//
//    ofxCvContourFinder contourFinder;
//
//    bool bThreshWithOpenCV;
//    bool bDrawPointCloud;
//
//    int nearThreshold;
//    int farThreshold;
//
//    // int angle;
//
////     used for viewing the point cloud
//    ofEasyCam easyCam;
    
};

