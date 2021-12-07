#include "ofApp.h"

Robot srRobot;
World world;
bool moveRobotToMouse = false;
bool robotPointToMouse = false;

//--------------------------------------------------------------
void ofApp::setup() {
	// Set up the default font
	if (!defaultFont.load(OF_TTF_SANS, 20))
		std::cout << "Error loading font. Any text will (most likely) not be rendered\n";

	world = World({ 100, 100 }, { 575, 575 });
	world.populateMarkers(worldMarkers, markerError);

	srRobot = Robot(&world, Vec3d(50, 50), 72);
	// srRobot.setPosUnknown({ world.m_pos.x + 0.5 * metreToPixel, world.m_pos.y + 0.5 * metreToPixel }, PI / 4);
	srRobot.setPosUnknown({ world.m_pos.x + 0.5 * metreToPixel, world.m_pos.y + 0.5 * metreToPixel }, -PI / 2);
}

//--------------------------------------------------------------
void ofApp::update() {

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(50);

	world.draw();
	srRobot.draw();

	// Note: It is inefficient to run srRobot.see() and srRobot.calculateWorldspacePosition() each draw
	//       call, but it is useful for debugging purposes. The code below can be commented out if needed
	std::vector<Marker> visible = srRobot.see();
	int64_t xCoord = 200 + 575;
	int64_t yCoord = 50;
	int64_t yOffset = 0;
	for (int64_t i = 0; i < (visible.size() > 28 ? 28 : visible.size()); ++i) {
		std::stringstream stream;
		stream.precision(4);
		stream << "ID: " << visible[i].id << " | X: " << visible[i].cartesian.x << "  |  Y: " << visible[i].cartesian.y;
		defaultFont.drawString(stream.str(), xCoord, yCoord + yOffset);
		yOffset += 30;
	}

	auto location = srRobot.calculateWorldspacePosition();
	Vec3d pos = location.first;
	double theta = location.second;
	ofSetColor(255, 38, 20);
	ofDrawCircle(world.m_pos.x + pos.x * metreToPixel, world.m_pos.y + pos.y * metreToPixel, 10);
	ofDrawLine(
		world.m_pos.x + pos.x * metreToPixel,
		world.m_pos.y + pos.y * metreToPixel,
		world.m_pos.x + pos.x * metreToPixel + cos(theta) * 50,
		world.m_pos.y + pos.y * metreToPixel + sin(theta) * 50
	);

	ofSetColor(255);
	double thetaDeg = rad2deg(theta);
	// if (thetaDeg < -180) thetaDeg += 360;
	defaultFont.drawString("Robot Position	: " + pos.str() + "\nRobot Angle		: " + std::to_string(thetaDeg) + "*", 100, ofGetWindowHeight() - 150);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == OF_KEY_CONTROL) robotPointToMouse = true;
	if (key == ' ') drawPositionDebuggingInfo ^= true;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
	if (key == OF_KEY_CONTROL) robotPointToMouse = false;
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
	if (!robotPointToMouse && button == 0) { // Move robot to mouse if left mouse button pressed
		if ((Vec3d(x, y) - srRobot.m_posUnknown).mag2() <= (srRobot.m_size / 2).mag2())
			moveRobotToMouse = true;

		if (moveRobotToMouse)
			srRobot.setPosUnknown({ x, y });
	}
	else {
		moveRobotToMouse = false;
	}

	if (button == 2 || robotPointToMouse) { // Rotate robot to point to mouse if right mouse button dragged
		double toMouse = atan((mouseY - srRobot.m_posUnknown.y) / (mouseX - srRobot.m_posUnknown.x));
		if (mouseX < srRobot.m_posUnknown.x && toMouse < PI) toMouse -= PI;
		srRobot.m_thetaUnknown = toMouse;
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	// Same code as in mouseDragged, but works for a single click
	if (!robotPointToMouse && button == 2) { // Rotate robot to point to mouse if right mouse button pressed
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
