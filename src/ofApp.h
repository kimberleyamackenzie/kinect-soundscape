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
    
    // When a known gesture is recognized
    void handEvent(ofxOpenNIGestureEvent & event);

    // Preset functions in OF app
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

    // Setting up for soundmaking
    ofSoundPlayer tuning;
    vector<ofSoundPlayer> pianoNotes;
    string note;
    
    // Counters
    int k;
    int frame;
    
    // Relating to sound cuing logic
    vector<ofVec2f> mathVectors;
    
    ofVec2f diff;
    ofVec2f position;
    
    float angle;
    float distance;

    // the Kinect seems like an important thing to have
    ofxOpenNI kinect;

    // Boolean for keeping track of skeletal tracking
    bool mode;
    
    // for drawing the mesh based on Kinect depth data
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage grayImage;
    ofxCvGrayscaleImage grayThreshNear;
    ofxCvGrayscaleImage grayThreshFar;
    ofxCvGrayscaleImage grayDiff;
    ofxCvGrayscaleImage grayBg;
    ofxCvGrayscaleImage grayCanny;
    ofxCvGrayscaleImage debugImage;
    ofxCvGrayscaleImage depthImg;
    ofxCvColorImage depthImg2;
    ofxCvGrayscaleImage cvimg;
    ofxCvGrayscaleImage edges;
    ofxCvGrayscaleImage depthOverlay;

    vector <ofPoint> contourReg;
    vector <ofPoint> contourSmooth;

    ofxCvContourFinder contourFinder;
    
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
    bool bLearnBackground;

    int thresholdValue;
    int nearThreshold;
    int farThreshold;
    
    // Boolean for triggered gathering point cloud data from Kinect
    bool bKinectDepthPointCloud;
    
    // Boolean for drawing the 'skeleton'
    bool bDrawnSkeleton;
    
    // Boolean for 'drawing' with your hand
    bool bHandDrawing;
    
    // Boolean for showing background mesh (not user generated)
    bool bBackgroundMesh;
    
    // used for viewing and manipulating meshes
    ofEasyCam easyCam;
    
    // Collection of hand positions used in sound cuing, mesh-making, and visualization generating
    vector<ofPoint> pointCollection;
    
    // Collection of joint positions used in sound cuing, mesh-making, and visualization generating
    vector<ofVec2f> bodyJointPointCollection;
    
    // Relating to non-mesh visualizations
    vector<float> radii;
    vector<integer_t> xPositions;
    vector<integer_t> yPositions;

    float radius = 0.1;
    ofPolyline line;
    int xPos;

    // Relating to visualizations, mesh and otherwise
    vector<ofColor>   colors;
    
    // For the meshes
    ofMesh backgroundMesh;
    ofMesh userMesh;
    ofImage image;
    vector<ofVec3f> offsets;

};

