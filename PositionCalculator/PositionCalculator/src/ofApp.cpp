#include "ofApp.h"

Robot srRobot;
World world;
bool moveRobotToMouse = false;

//--------------------------------------------------------------
void ofApp::setup() {
	// Set up the default font
	if (!defaultFont.load(OF_TTF_SANS, 20))
		std::cout << "Error loading font. Any text will (most likely) not be rendered\n";

	srRobot = Robot(Vec3d(50, 50), 72);
	srRobot.setPosUnknown({ 200, 200 }, PI * 8 / 5);

	world = World({ 50, 50 }, { 575, 575 }, {});
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

	std::vector<Marker> visible = srRobot.see(world);
	int64_t xCoord = 100 + 575;
	int64_t yCoord = 50;
	int64_t yOffset = 0;
	for (const auto& marker : visible) {
		std::stringstream stream;
		stream.precision(4);
		stream << "ID: " << marker.id << " | X: " << marker.cartesian.x << "  |  Y: " << marker.cartesian.y;
		defaultFont.drawString(stream.str(), xCoord, yCoord + yOffset);
		yOffset += 30;
	}
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
	if (button == 0) { // Move robot to mouse if left mouse button pressed
		if ((Vec3d(x, y) - srRobot.m_posUnknown).mag2() <= (srRobot.m_size / 2).mag2())
			moveRobotToMouse = true;

		if (moveRobotToMouse)
			srRobot.setPosUnknown({ x, y });
	}
	else {
		moveRobotToMouse = false;
	}

	if (button == 2) { // Rotate robot to point to mouse if right mouse button dragged
		double toMouse = atan((mouseY - srRobot.m_posUnknown.y) / (mouseX - srRobot.m_posUnknown.x));
		if (mouseX < srRobot.m_posUnknown.x && toMouse < PI) toMouse -= PI;
		srRobot.m_thetaUnknown = toMouse;
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	// Same code as in mouseDragged, but works for a single click
	if (button == 2) { // Rotate robot to point to mouse if right mouse button pressed
		double toMouse = atan((mouseY - srRobot.m_posUnknown.y) / (mouseX - srRobot.m_posUnknown.x));
		if (mouseX < srRobot.m_posUnknown.x && toMouse < PI) toMouse -= PI;
		srRobot.m_thetaUnknown = toMouse;
	}
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
