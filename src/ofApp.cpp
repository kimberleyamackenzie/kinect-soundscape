#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    // Managing the kinect and its many many needs
    kinect.setup();
    kinect.setRegister(true);
    kinect.setRegistration(true);
    kinect.setMirror(true);
    kinect.addDepthGenerator();
    kinect.addUserGenerator();
    
    // Managing the kinect with ofxKinect commands in addition to above ofxOpenNI commands
    kinect.ofxKinect::init();
    kinect.ofxKinect::open();
    kinect.start();
    mode = false;
    
    // Allowing the Kinect to register hands (in addition to joints) and recognize a small, small, small set of basic gestures
    kinect.addHandsGenerator();
    kinect.addAllHandFocusGestures();
    kinect.addGestureGenerator();
    kinect.setMaxNumHands(6);

    // Custom event listener for gestures
    ofAddListener(kinect.gestureEvent,this,&ofApp::handEvent);
    
    
    // Iterate through all the wav files to load them; allow them to play simultaneously to themselves, and push them into our collection for easy retrival later
    for(int i = 1; i<80; i++){
        char note[1024];
        sprintf(note, "note%d.wav", i);
        ofSoundPlayer player;
        player.load(note);
        player.setMultiPlay(true);
        pianoNotes.push_back(player);
    }
    
    
    // Fun with variables!
    k = 0;

    // frame counter
    frame = 0;

    // Set up for drawing a mesh based on Kinect depth data
    colorImg.allocate(kinect.getWidth(), kinect.getHeight());
    grayImage.allocate(kinect.getWidth(), kinect.getHeight());
    grayThreshNear.allocate(kinect.getWidth(), kinect.getHeight());
    grayThreshFar.allocate(kinect.getWidth(), kinect.getHeight());
    cvimg.allocate(kinect.getWidth(), kinect.getHeight());
    edges.allocate(kinect.getWidth(), kinect.getHeight());
    debugImage.allocate(kinect.getWidth(), kinect.getHeight());
    depthOverlay.allocate(kinect.getWidth(), kinect.getHeight());
    depthImg.allocate(kinect.getWidth(), kinect.getHeight());
    depthImg2.allocate(kinect.getWidth(), kinect.getHeight());
    grayBg.allocate(320,240);
    grayDiff.allocate(kinect.getWidth(), kinect.getHeight());
    
    thresholdValue = 40;
    nearThreshold = 230;
    farThreshold  = 70;
    
    bThreshWithOpenCV = true;
    bDrawPointCloud = true;
    bLearnBackground = true;
    
    
    // Set frame rate to ensure visualizations operate on same frame rate as Kinect is updating
    ofSetFrameRate(60);
    
    // Setup for a mesh that would become a background (not user generated)
    // Load the image to be meshed
    image.load("test.png");
    image.resize(200, 200);
    // Set up mesh basics
    backgroundMesh.setMode(OF_PRIMITIVE_LINES);
    backgroundMesh.enableColors();
    backgroundMesh.enableIndices();
    
    // Add all vertices based on lightness of pixels
    float intensityThreshold = 125.0;
    int w = image.getWidth();
    int h = image.getHeight();
    for (int x = 0; x < w; ++x){
        for (int y = 0; y < h; ++y) {
            ofColor c = image.getColor(x, y);
            float intensity = c.getLightness();
            if(intensity >= intensityThreshold) {
                float saturation = c.getSaturation();
                float z = ofMap(saturation, 0, 255, -100, 100);
                cout << "z is " << z << endl;
                ofVec3f pos(x * 4, y * 4, z);
                backgroundMesh.addVertex(pos);
                backgroundMesh.addColor(c);
            }
        }
    }
    // Add indices and connect the vertices
    float connectionDistance = 30;
    int numVerts = backgroundMesh.getNumVertices();
    for (int a=0; a<numVerts; ++a) {
        ofVec3f verta = backgroundMesh.getVertex(a);
        for (int b=a+1; b<numVerts; ++b) {
            ofVec3f vertb = backgroundMesh.getVertex(b);
            float distance = verta.distance(vertb);
            if (distance <= connectionDistance) {
                backgroundMesh.addIndex(a);
                backgroundMesh.addIndex(b);
            }
        }
    }
    
    // Set up the basics for the user generated mesh
    userMesh.setMode(OF_PRIMITIVE_LINES);
    userMesh.enableColors();
    userMesh.enableIndices();

}

//--------------------------------------------------------------
void ofApp::update(){
    
    // During update, the kinect should probably update.  Maybe.
    kinect.update();
    
    // Easy frame counter
    frame += 1;
    
    // Update for offsets to create small movements on user generated mesh
    int numVerts = userMesh.getNumVertices();
    for (int i=0; i<numVerts; ++i) {
        ofVec3f vert = userMesh.getVertex(i);
        
        float time = ofGetElapsedTimef();
        float timeScale = 5.0;
        float displacementScale = 0.75;
        
        ofVec3f timeOffsets = offsets[i];
        
        vert.x += (ofSignedNoise(time*timeScale+timeOffsets.x)) * displacementScale;
        vert.y += (ofSignedNoise(time*timeScale+timeOffsets.y)) * displacementScale;
        vert.z += (ofSignedNoise(time*timeScale+timeOffsets.z)) * displacementScale;
        userMesh.setVertex(i, vert);
    }

    // Creating a mesh/contours/point cloud from the user based on Kinect data
    if(kinect.isFrameNew()) {
        grayImage.setFromPixels(kinect.getDepthPixels());

        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            ofPixels & pix = grayImage.getPixels();
            int numPixels = pix.size();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        grayImage.flagImageChanged();
        // find contours and interior contours
        contourFinder.findContours(grayDiff, 20, 25000, 10, true);
        contourFinder.findContours(grayImage, 10, (kinect.getWidth()*kinect.getHeight())/2, 20, false);
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    
        // Trigger drawing point cloud (creating mesh from Kinect depth data)
        // easyCam.begin();
        // drawPointCloud();
        // easyCam.end();

    
        // Or draw depth (quick fix to generating blobs)
        // kinect.drawDepth(10, 10, ofGetWidth(), ofGetHeight());
        // kinect.draw(420, 10, 400, 300);
        // grayImage.draw(10, 320, 400, 300);
        // contourFinder.draw(10, 320, 400, 300);

    // Create gradient for background
    ofColor centerColor = ofColor(85, 78, 68);
    ofColor edgeColor(0, 0, 0);
    ofBackgroundGradient(centerColor, edgeColor, OF_GRADIENT_CIRCULAR);
    
    // Start easy cam for manipulating static mesh (backgroundMesh) above
    // easyCam.begin();
    // ofPushMatrix();
    // ofTranslate(-ofGetWidth()/2, -ofGetHeight()/2);
    // backgroundMesh.draw();
    // ofPopMatrix();
    // easyCam.end();

    // Could use below syntax to redefine the field of view depending on physical set-up of space
    // ofTranslate(200, 50, - 100);
    
    // As long as there is a least one person being tracked...
    if(kinect.getNumTrackedUsers() > 0 ){
     for(int m = 0; m < kinect.getNumTrackedUsers(); m ++){
         
         // Where to start drawing the skeleton if we're doing that
         // line.draw();
        
        // Make that user the person who's skeleton we're tracking.  Would need to mess around here for logic around tracking multiple bodies.
        ofxOpenNIUser user = kinect.getTrackedUser(m);

        // Syntax for getting a specific joint (rather than interating through all)
        ofxOpenNIJoint trackedJoint = user.getJoint(JOINT_RIGHT_ELBOW);

        // For the total count of limbs that the Kinect identifies on the user
        for(int i = 0; i < user.getNumLimbs(); i ++){
            // Go through the ofxOpenNI collection of limbs
            ofxOpenNILimb limb = user.getLimb((enum Limb) i);
            
            // And if that limb is seen on the user
            if( limb.isFound()){
                
                // Find the positions of the joints for that limb
                float x1 = limb.getStartJoint().getProjectivePosition().x;
                float y1 = limb.getStartJoint().getProjectivePosition().y;
                float x2 = limb.getEndJoint().getProjectivePosition().x;
                float y2 = limb.getEndJoint().getProjectivePosition().y;
                
                // And draw a line so that the limb can be seen - if you're drawing the skeleton
                // ofDrawLine(x1, y1, x2, y2);
            }
        }
        
        // Create the vector where the joint's x/y location will be stored
        ofVec2f pos;
        
        // For the total count of joints that the Kinect identifies on the user
        for( int i = 0 ; i < user.getNumJoints(); i ++){
            // Go through the ofxOpenNI collection of limbs
            ofxOpenNIJoint joint = user.getJoint((enum Joint)i);
            
            // Set up the variables we will need to manipulate as the hand moves through space to trigger sounds
            int j = 0;
            float volume = 0;
            float speed;
            
            // And if that joint is seen on the user
            if(joint.isFound()){
                
                // And then we find the position of that joint, set it to the (image)vector we declared above, to draw a circle of radius 10 at that point
                float x = joint.getProjectivePosition().x;
                float y = joint.getProjectivePosition().y;
                pos.set(x, y);
                
                // Draw a circle at each joint, if you're drawing the skeleton
                // ofDrawCircle(pos.x, pos.y, 10);
            
                

                
                //                for (int xX = 0; xX < 700; xX += 7){
                //
                //                    // This if statement has every possible point has a sound attached to it, and makes total cacophany (fun but annoying to my neighbors)
                //                    //                if(xX < floor(handPosition.x) && floor(handPosition.x) < (xX + 10)) {
                //
                //                    // If the position of the hand in space matches the given value of xX
                //                    if(xX == floor(x)) {
                //
                //                        // Then set the speed of the sound based on the hand's y position. If the speed ends up negative, we can't play the sound, so rather than have pockets of empty positions with no sounds, we find the absolute value of the speed calculation.  We set the speed of the playback (as a workaround for pitch).
                //                        speed = abs((x - y) / x * 3.0);
                //                        pianoNotes[j].setSpeed(speed);
                //
                //                        // And now we set the volume for playback using ofLerp function to create some sound modulation and not be so tinny and singular
                //                        volume = ofLerp(volume, 1, 0.8); // jump quickly to 1
                //
                //                        // And we play it!
                //                        pianoNotes[j].play();
                //
                //                        // There is potentially an artistic future where we may want some logic around looping particular sounds, so here this will stay for awhile.
                //                        //                    if(pianoNotes[j].isPlaying()){
                //                        //                        pianoNotes[i].setLoop(true);
                //                        //                        cout << "the y position is " << handPosition.y << "and the speed is " << speed << endl;
                //                        //                    }
                //
                //                        // And this is the start of some logic to create a super simplistic visualization of the positions at which the tracked hand trigers sound
                //                        //                    if (pianoNotes[i].isPlaying()) {// Random color
                //                        //                        ofSetColor(ofRandom(0, 255), ofRandom(0, 255), ofRandom(0, 255), 20);
                //                        //                        ofDrawCircle(handPosition.x, handPosition.y, 15);}
                //                        //                    ofDisableAlphaBlending();
                //                        //                    }
                //                    }
                //
                //                    // If the current increment of xX doesn't match the current x axis position of the tracked hand, we increment j to move on to the next potential sound file.  Since we have a limited number of files, and want to avoid errors relating to potentially wider fields of view than what exist in testing circumstances, if we end up iterating higher than the number of sound files we have, we reset that iterator to 0.
                //                    if(j < pianoNotes.size()) {
                //                        j += 1;
                //                    }else {
                //                        j = 0;
                //                    }
                //}
                // Otherwise, the skeletal tracking monitor circle is set to false, and is red, signaling that a certain joint cannot be identified
            }else{
                mode = false;
            }
        }
        
        // And now, for the total count of hands the Kinect identifies as being tracked
        for( int i = 0 ; i < kinect.getNumTrackedHands(); i ++){
            
            // Set up the variables we will need to manipulate as the hand moves through space to trigger sounds
            int j = 0;
            float volume = 0;
            float speed;
            
            // Find that hand and assign it to the hand object
            ofxOpenNIHand & hand = kinect.getTrackedHand(i);
            
            // Get that hand's position
            ofPoint & handPosition = hand.getPosition();
            
            // Watch the gestures of that hand as a continuous line being drawn on the screen if that's your thing
            // ofPoint pt;
            // pt.set(handPosition.x, handPosition.y);
            // line.addVertex(handPosition.x, handPosition.y);
            // ofDrawLine(handPosition.x, handPosition.y, 50, 50);
            
            position.set(hand.getPosition().x, hand.getPosition().y);
            mathVectors.push_back(position);
            
            
            if(frame % 60 == 0){
                diff = mathVectors[k] - mathVectors[k - 59];
                angle = mathVectors[k - 59].angle(mathVectors[k]);
                distance = mathVectors[k - 59].distance(mathVectors[k]);
//                cout << distance << endl;
//                cout << angle << endl;

                
                if(angle > 90){
//                    pianoNotes.back().play();
//                    cout << angle << endl;
                }

            }
            
              ofPoint pt;
              pt.set(handPosition.x, handPosition.y);
            
            // More logic around drawing with hand position and ensuring that the line never gets to long (deletes from end)
            // line.addVertex(handPosition.x, handPosition.y);
            // ofDrawLine(handPosition.x, handPosition.y, 50, 50);
            
            // if (line.size() > 100){
                // line.getVertices().erase(line.getVertices().begin());
            // }
            

            
            k += 1;
            
            // In increments of 7 (because 1 was too small and 10 seemed like too much 'empty' space/sound, iterate through the stops of the x axis of the Kinect's field of view
            for (int xX = 0; xX < 700; xX += 7){
                
                // This if statement has every possible point has a sound attached to it, and makes total cacophany (fun but annoying to my neighbors)
                //                if(xX < floor(handPosition.x) && floor(handPosition.x) < (xX + 10)) {
                
                // If the position of the hand in space matches the given value of xX
                if(xX == floor(handPosition.x)) {
                    
                    // Then set the speed of the sound based on the hand's y position or based on the distance of the vectors from current from and 60 frames (1 second) ago.  Quick short movements results in extremely high pitched sounds, long slower movements results in slower, lower sounds.
                    //If the speed ends up negative, we can't play the sound, so rather than have pockets of empty positions with no sounds, we find the absolute value of the speed calculation.  We set the speed of the playback (as a workaround for pitch).
                    //Refactor into case statement later if you don't get too distracted
                    
                    if(distance < 25){
                        speed = 6;
                    }
                    if(distance > 25 && distance < 50){
                        speed = 4;
                    }
                    if(distance > 50 && distance < 200){
                        speed = abs((handPosition.x - handPosition.y) / handPosition.x * 3.0);
                    }
                    if(distance > 200 && distance < 350){
                        speed = .5;
                    }
                    if(distance > 350){
                        speed = .2;
                    }
                    pianoNotes[j].setSpeed(speed);
                    
                    //Set the pan position of the sound (which speaker it's coming from) based on x.  (0, 0) is at the center of the sensor, so if you move signifiantly to the left or to the right, the sound shifts to be coming from either of those sides.
                    if(handPosition.x < -300){
                        pianoNotes[j].setPan(-1.0f);
                    }
                    if(handPosition.x > 300){
                        pianoNotes[j].setPan(-1.0f);
                    }
                    
                    // And now we set the volume for playback using ofLerp function to create some sound modulation and not be so tinny and singular
                    volume = ofLerp(volume, 1, 0.8); // jump quickly to 1
                    
                    // And we play it!
                    pianoNotes[j].play();
                    pointCollection.push_back(pt);

                    
                    
                    // There is potentially an artistic future where we may want some logic around looping particular sounds, so here this will stay for awhile.
                    //                    if(pianoNotes[j].isPlaying()){
                    //                        pianoNotes[i].setLoop(true);
                    //                        cout << "the y position is " << handPosition.y << "and the speed is " << speed << endl;
                    //                    }
                    
                    // And this is the start of some logic to create a super simplistic visualization of the positions at which the tracked hand trigers sound
                    //                    if (pianoNotes[i].isPlaying()) {// Random color
                    //                        ofSetColor(ofRandom(0, 255), ofRandom(0, 255), ofRandom(0, 255), 20);
                    //                        ofDrawCircle(handPosition.x, handPosition.y, 15);}
                    //                    ofDisableAlphaBlending();
                    //                    }
                }
                
                // If the current increment of xX doesn't match the current x axis position of the tracked hand, we increment j to move on to the next potential sound file.  Since we have a limited number of files, and want to avoid errors relating to potentially wider fields of view than what exist in testing circumstances, if we end up iterating higher than the number of sound files we have, we reset that iterator to 0.
                if(j < (pianoNotes.size() - 2)){
                    j += 1;
                }else {
                    j = 0;
                }
            }
        }
         
         // // Visualizations that trigger the creation of circles that start at a randomized size and get gradually larger relative to their starting size as the frames increase
         // // N number of times, as long as we don't equal or surpass how many points of hand positions we have
         // for (int n = 0; n < (pointCollection.size()); n ++){
            // // Create a polyline
            // ofPolyline polyline;
            // // Get a random color, and push it into a vector of colors so the circle will remain the same color throughout the program
            // colors.push_back(ofColor(ofRandom(0,255),ofRandom(0,255),ofRandom(0,255)));
            // // Same as above but with the radius of the circle
            // radii.push_back(ofRandom(0, 1));
            // // Rotating the depth around the center of the OF grad created a neat effect
            // ofRotateZ(45);
            // If we are creating a circle for the first time, give it that random radius, and draw the polyline going from 0 to 360 to make a circle.  Otherwise, save the radius as .1 larger than the last frame and set the color to the correlating n color, and draw it.
                 // if(n == (pointCollection.size() - 1)){
                    // polyline.arc(pointCollection[n].x, pointCollection[n].y, radii[n], radii[n], 0, 360);
                 // } else {
                     // radii[n] += 0.1;
                    // polyline.arc(pointCollection[n].x, pointCollection[n].y, radii[n], radii[n], 0, 360);
                 // }
                 // ofSetColor(colors[n]);
                 // polyline.draw();
         // }
         
         // // Similiar logic as above, but with lines that come from a set position on the top of the screen and go down to the users hand position
            // for (int n = 0; n < (pointCollection.size()); n ++){
                 // ofPolyline polyline;
                 // colors.push_back(ofColor(ofRandom(0,255),ofRandom(0,255),ofRandom(0,255)));
                 // xPositions.push_back(ofRandom(0, pointCollection[n].x));
                 // yPositions.push_back(ofRandom(0, pointCollection[n].y));
                 // ofSetColor(colors[n]);
                 // if(n == (pointCollection.size() - 1)){
                      // ofDrawLine(xPositions[n], yPositions[n], pointCollection[n].x, pointCollection[n].y);
                  // } else {
                       // yPositions[n] += 2;
                       // if(yPositions[n] < pointCollection[n].y){
                       // ofDrawLine(xPositions[n], yPositions[n], pointCollection[n].x, pointCollection[n].y);
                      // }
                  // }
             // }
         
         // Creating the user generated mesh from hand positions during each frame
         // As long as there are previous hand positions to reference
         if(pointCollection.size() > 0){
             // Play with adding depth to user generated mesh
             float z = ofRandom(-50, 50);
             // Create a vector based on the last position the hand was in and (potentially) the randomized depth
             ofVec3f nextVertex(pointCollection.back().x, pointCollection.back().y, z);
             // Make this piece of the mesh a randomized color
             userMesh.addColor(ofColor(ofRandom(0,255),ofRandom(0,255),ofRandom(0,255)));
             // Add this vertex to the user generated mesh
             userMesh.addVertex(nextVertex);
             // Everytime a new vertex is created, so is a random combination of numbers for the offsets in update()
             offsets.push_back(ofVec3f(ofRandom(0,100000), ofRandom(0,100000), ofRandom(0,100000)));
         }
         
         // Now, connect those vertices through referencing and creating indicies, based on the connection distance between vertices
         float connectionDistance = 80;
         int numVerts = userMesh.getNumVertices();
         for (int a=0; a<numVerts; ++a) {
             ofVec3f verta = userMesh.getVertex(a);
             for (int b=a+1; b<numVerts; ++b) {
                 ofVec3f vertb = userMesh.getVertex(b);
                 float distance = verta.distance(vertb);
                 if (distance <= connectionDistance) {
                     // In OF_PRIMITIVE_LINES, every pair of vertices or indices will be
                     // connected to form a line
                     userMesh.addIndex(a);
                     userMesh.addIndex(b);
                 }
             }
         }
         userMesh.draw();
         
         ofPopMatrix();
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------

void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

//--------------------------------------------------------------

void ofApp::handEvent(ofxOpenNIGestureEvent & event){
    if(event.gestureName == "Wave"){
//        kinect.stop();
        kinect.close();
    }
    
}

//--------------------------------------------------------------
void ofApp::drawPointCloud() {

    int w = 640;
    int h = 480;
    ofMesh mesh;

    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;

    for (int y = 0; y < h; y += step) {
        cout << "drawing point cloud" << endl;
        int currentIndex = 0;
        for (int x = 0; x < w; x += step) {
            if (kinect.ofxKinect::getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.ofxKinect::getColorAt(x, y));
                mesh.addVertex(kinect.ofxKinect::getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000);
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}
