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

	world = World({ 100, 100 }, { 5.75, 5.75 });
	world.populateMarkers(worldMarkers, markerError);

	srRobot = Robot(&world, Vec3d(50, 50), 72);
	// srRobot.setPosUnknown({ world.m_pos.x + 0.5 * metreToPixel, world.m_pos.y + 0.5 * metreToPixel }, PI / 4);
	srRobot.setPosUnknown({ world.m_pos.x + 0.5 * metreToPixel, world.m_pos.y + 0.5 * metreToPixel }, -PI / 2);

	// Add a single can to the world
	std::vector<Vec3d> canPositionsFloor = {
		// WRONG ORIENTATION
{		2.871819690265487,0.047704646017699116		 },
{2.871819690265487,1.1385508849557524				 },
{1.6187776548672568,1.62195796460177					 },
{4.124861725663717,1.62195796460177					 },
{2.474280973451328,1.8732024336283188				 },
{3.2725387168141595,1.8732024336283188				 },
{1.876382743362832,2.4711006637168142				 },
{3.8736172566371683,2.4711006637168142				 },
{0.05088495575221239,2.8781803097345136				 },
{1.1385508849557524,2.8781803097345136				 },
{4.614629424778761,2.8781803097345136				 },
{5.705475663716815,2.8781803097345136				 },
{1.876382743362832,3.2693584070796464				 },
{3.8736172566371683,3.2693584070796464				 },
{2.474280973451328,3.870436946902655					 },
{3.2725387168141595,3.870436946902655				 },
{1.6187776548672568,3.9563053097345136				 },
{4.124861725663717,3.9563053097345136				 },
{2.871819690265487,4.614629424778761					 },
{2.871819690265487,5.702295353982302					 },
	};

	std::vector<Vec3d> canPositionsRaised = {
{2.5792311946902657,2.315265486725664		  },
{3.173949115044248,2.315265486725664			  },
{3.441095132743363,2.572870575221239			  },
{3.441095132743363,3.173949115044248			  },
{3.173949115044248,3.441095132743363			  },
{2.5792311946902657,3.441095132743363		  },
{2.315265486725664,3.173949115044248			  },
{2.315265486725664,2.569690265486726			  }
	};


	for (const auto& can : canPositionsFloor) {
		world.addCan(Can(can, false, 0.067));
	}

	for (const auto& can : canPositionsRaised) {
		world.addCan(Can(can, true, 0.067));
	}
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

	ofSetColor(82, 235, 52);
	double thetaTrue = rad2deg(srRobot.m_thetaUnknown);
	if (thetaTrue < -180) thetaTrue += 360;
	defaultFont.drawString("Robot Position	: " + ((srRobot.m_posUnknown - world.m_pos) * pixelToMetre).str() + "\nRobot Angle		: " + std::to_string(thetaTrue) + "*", 100, ofGetWindowHeight() - 175);

	ofSetColor(235, 217, 52);
	defaultFont.drawString("Robot Position	: " + pos.str() + "\nRobot Angle		: " + std::to_string(rad2deg(theta)) + "*", 100, ofGetWindowHeight() - 75);

	// world.distanceSensor((srRobot.m_posUnknown - world.m_pos) * pixelToMetre, srRobot.m_thetaUnknown);
	// srRobot.projectedIntersection();

	if (srRobot.lookingAtCan()) {
		ofSetColor(113, 252, 43);
		ofDrawCircle(world.m_pos.x + world.m_size.x * metreToPixel / 2, world.m_pos.y + world.m_size.y * metreToPixel / 2, 25);
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == OF_KEY_CONTROL) robotPointToMouse = true;
	if (key == ' ') showPositionDebugInfo ^= true;
	if (key == 'r') showRaycastDebugInfo ^= true;
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
