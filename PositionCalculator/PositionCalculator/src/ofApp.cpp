#include "ofApp.h"

Robot srRobot;
World world;

//--------------------------------------------------------------
void ofApp::setup() {
	srRobot = Robot(Vec3d(50, 50));
	srRobot.setPosUnknown(400, 400, 0);

	world = World({ 50, 50 }, { ofGetWindowWidth() - 400, ofGetWindowWidth() - 400 }, {});
	// world.populateMarkers(28);
	world.populateMarkers(28);
}

//--------------------------------------------------------------
void ofApp::update() {

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(50);

	world.draw();
	srRobot.draw();

	srRobot.m_thetaUnknown += 0.01;

	std::cout << srRobot.see(world).size() << "\n";
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
