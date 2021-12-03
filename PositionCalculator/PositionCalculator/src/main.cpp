#include "ofMain.h"
#include "ofApp.h"

#include "vector.hpp"
#include <chrono>

//========================================================================
int main() {
	ofSetupOpenGL(1224, 924, OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

	return 0;
}
