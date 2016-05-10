#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    printGrabberDevices();

    cloud.startGrabbing();
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(128);

    cloud.drawImage();

    if(false && cloud.getMesh().getNumVertices() > 0){
        cloud.getMesh().save("test.ply");
        ofExit();
    }

    cam.begin();
    ofSetColor(255);
    cloud.draw();
    cam.end();

    ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate()), 30, 30);

    //if(cloud.getMesh().getNumVertices() > 0)
    //ofExit();
    /*
    cam.begin();
    ofSetColor(255);
    cloud.draw();
    cam.end();
    */
}

void ofApp::exit(){
    cloud.stopGrabbing();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    cloud.savePCD("test.pcd");
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


