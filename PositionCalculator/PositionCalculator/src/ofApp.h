#pragma once

#include "ofMain.h"

#include <vector>
#include <cstdint>
#include <cmath>
#include <sstream>

#include "vector.hpp"

/**
 * Scale information:
 *
 * Every pixel represents 1cm. The actual box is a 5.75m +- 20cm square,
 * and the rendered board is 575 pixels square
 */

constexpr double pixelToMetre = 0.01;
constexpr double metreToPixel = 100;

static ofTrueTypeFont defaultFont;

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
};

inline double deg2rad(double deg) {
	return deg * (PI / 180);
}

inline double rad2deg(double rad) {
	return rad * (180 / PI);
}

// Linearly interpolate between two values
inline double lerp(double start, double end, double percent) {
	return start + (end - start) * percent;
}

class Marker {
public:
	// NOTE: Some information is not stored. This is to make the program simpler

	Marker() : id(-1), distance(-1), cartesian({ 0, 0 }) {}

	Marker(Vec3d cartesian, int64_t id = -1) : cartesian(cartesian), distance(cartesian.mag()), id(id) {}

	Marker(const Marker& other) : id(other.id), distance(other.distance), cartesian(other.cartesian) {}

	Marker& operator=(const Marker& other) {
		id = other.id;
		distance = other.distance;
		cartesian = other.cartesian;
		return *this;
	}

	int64_t id; // The ID of the marker
	double distance; // Distance from camera to marker
	Vec3d cartesian; // A Vec3d instance describing the absolute position of the marker relative to the camera
};

class World {
public:
	World() = default;

	World(const Vec3d& p1, const Vec3d& size, const std::vector<Marker>& markers)
		: m_pos(p1), m_size(size), m_markers(markers) {}

	World& operator=(const World& other) {
		m_pos = other.m_pos;
		m_size = other.m_size;
		m_markers = other.m_markers;
		return *this;
	};

	void populateMarkers(int64_t numMarkers) {
		m_markers = std::vector<Marker>(numMarkers);
		int64_t markersPerSide = numMarkers / 4;
		double markerDist = m_size.x / (double)(markersPerSide + 1);

		// Top side -- IDs count up -- Increment on X axis
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i] = Marker(Vec3d{ (i + 1) * markerDist, 0 }, i);

		// Right side -- IDs count up -- Increment on Y axis
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i + markersPerSide] = Marker(Vec3d{ m_size.x, (i + 1) * markerDist }, i + markersPerSide);

		// Bottom side -- IDs count *down* due to clock-wise ordering -- Increment on X axis
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i + markersPerSide * 2] = Marker(Vec3d{ (i + 1) * markerDist, m_size.y }, markersPerSide * 3 - i - 1);

		// Left side -- IDs count *down* due to clock-wise ordering -- Increment on X axis
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i + markersPerSide * 3] = Marker(Vec3d{ 0, (i + 1) * markerDist }, markersPerSide * 4 - i - 1);
	}

	void draw() const {
		// Draw scoring zones
		ofSetLineWidth(1);
		ofSetColor(0);
		ofDrawLine(m_pos.x, m_pos.y + 2.5 * metreToPixel, m_pos.x + 2.5 * metreToPixel, m_pos.y);
		ofDrawLine(m_pos.x + m_size.x - 2.5 * metreToPixel, m_pos.y, m_pos.x + m_size.x, m_pos.y + 2.5 * metreToPixel);
		ofDrawLine(m_pos.x + m_size.x, m_pos.y + m_size.y - 2.5 * metreToPixel, m_pos.x + m_size.x - 2.5 * metreToPixel, m_pos.y + m_size.y);
		ofDrawLine(m_pos.x, m_pos.y + m_size.y - 2.5 * metreToPixel, m_pos.x + 2.5 * metreToPixel, m_pos.y + m_size.y);

		// Draw starting zones
		ofSetColor(201, 164, 85);
		ofDrawRectangle(m_pos.x, m_pos.y, 1 * metreToPixel, 1 * metreToPixel);
		ofDrawRectangle(m_pos.x + m_size.x - 1 * metreToPixel, m_pos.y, 1 * metreToPixel, 1 * metreToPixel);
		ofDrawRectangle(m_pos.x + m_size.x - 1 * metreToPixel, m_pos.y + m_size.y - 1 * metreToPixel, 1 * metreToPixel, 1 * metreToPixel);
		ofDrawRectangle(m_pos.x, m_pos.y + m_size.y - 1 * metreToPixel, 1 * metreToPixel, 1 * metreToPixel);

		// Draw bounding box
		ofSetColor(170, 50, 50);
		ofNoFill();
		ofSetLineWidth(5);
		ofDrawRectangle(m_pos.x, m_pos.y, m_size.x, m_size.y);
		ofSetLineWidth(2);
		ofFill();

		// Draw all the markers as green dots
		ofSetColor(50, 170, 50);
		for (const auto& marker : m_markers)
			ofDrawCircle(m_pos.x + marker.cartesian.x, m_pos.y + marker.cartesian.y, 10);
	}

public:
	Vec3d m_pos;
	Vec3d m_size;
	std::vector<Marker> m_markers;
};

class Robot {
public:
	Robot() {}

	Robot(const Vec3d& size, double fov = 72) {
		m_size = size;
		m_relCameraPos = { 0, 0 };
		m_fov = deg2rad(fov);
	}

	/// <summary>
	/// Set the screen-space coordinates of the robot.
	/// These are unknown to the robot itself and are used for drawing.
	/// </summary>
	/// <param name="x">= x-coordinate</param>
	/// <param name="y">= y-coordinate</param>
	/// <param name="theta">= angle in radians</param>
	void setPosUnknown(Vec3d pos, double theta) {
		m_posUnknown = pos;
		m_thetaUnknown = theta;
	}

	void setPosUnknown(Vec3d pos) {
		m_posUnknown = pos;
	}

	/// <summary>
	/// Draw the robot to the screen using its screen-space coordinates.
	/// </summary>
	void draw() const {
		ofSetColor(50, 170, 50);

		// Do some rotation + translation and then draw the robot as a rectangle
		ofPushMatrix();
		ofTranslate(m_posUnknown.x, m_posUnknown.y);
		ofRotateRad(m_thetaUnknown - (PI / 2));
		ofDrawRectRounded(-m_size.x / 2, -m_size.y / 2, m_size.x, m_size.x, 10);
		ofSetColor(0);
		ofDrawCircle(m_size.x * -0.3, m_size.y * 0.3, 5);
		ofDrawCircle(m_size.x * 0.3, m_size.y * 0.3, 5);
		ofPopMatrix();

		// Find screen-space origin and end-point of ray
		Vec3d origin = m_posUnknown;
		double rayAngle = m_fov / 2;
		Vec3d end1, end2;
		end1.x = origin.x + 100 * cos(rayAngle + m_thetaUnknown);
		end1.y = origin.y + 100 * sin(rayAngle + m_thetaUnknown);

		end2.x = origin.x + 100 * cos(-rayAngle + m_thetaUnknown);
		end2.y = origin.y + 100 * sin(-rayAngle + m_thetaUnknown);

		// Draw the lines to show the FOV of the camera
		ofSetColor(255, 0, 0);
		ofDrawLine(origin.x, origin.y, end1.x, end1.y);
		ofDrawLine(origin.x, origin.y, end2.x, end2.y);
	}

	std::vector<Marker> see(const World& world) const {
		// List of markers visible to the robot
		std::vector<Marker> visible;

		// Iterate over all markers
		for (const auto& marker : world.m_markers) {
			// Position offsets from marker to robot
			Vec3d offset(
				world.m_pos.x + marker.cartesian.x - m_posUnknown.x,
				world.m_pos.y + marker.cartesian.y - m_posUnknown.y
			);

			// Calculate the angles for the field of view -- rayTheta is the angle the FOV makes with the x-axis
			double rayAngle = m_fov / 2;
			double rayTheta = atan(sin(rayAngle + m_thetaUnknown) / cos(rayAngle + m_thetaUnknown));

			// theta is the angle made between the line connecting the camera to the marker and the x-axis
			// Offset is the change in y and x, so tan(theta) = offset.y / offset.x -- therefore theta = tan^-1(offset.y / offset.x)
			double theta = atan(offset.y / offset.x);

			// Fix a problem where atan returns the angle to the *negative* x-axis -- we need the angle to the positive x
			if (theta > 0 && rayTheta < 0) rayTheta += PI;

			// Do some checks to see if the marker is within the robots FOV
			// 1. Angle to marker is greater than the smaller FOV angle
			// 2. Angle to marker is smaller than the larger FOV angle
			// 3. Marker is in front of robot -- done by checking if dot-product is greater than 0
			if (theta < rayTheta && theta > rayTheta - rayAngle * 2 && offset.dot(Vec3d(cos(rayAngle + m_thetaUnknown), sin(rayAngle + m_thetaUnknown))) > 0) {
				// Highlight the visible dots
				ofSetColor(170, 170, 255);
				ofDrawCircle(world.m_pos.x + marker.cartesian.x, world.m_pos.y + marker.cartesian.y, 15);

				// Label each marker
				ofSetColor(255);
				defaultFont.drawString(std::to_string(marker.id), world.m_pos.x + marker.cartesian.x - 30, world.m_pos.y + marker.cartesian.y - 30);

				// =============================================================================================================

				// Up to this point, everything has been for the end-user, not the information that will be passed to the robot.
				// For this reason, we must convert the position of the marker into an offset which is based on the direction of the
				// robot -- basically, shift the XY axes so that they align with the robot, then figure out where the marker is on
				// that new coordinate system
				// For some visualizations -- see https://www.desmos.com/calculator/zavfflxon7

				double alpha = theta - m_thetaUnknown;
				if (offset.x < 0) alpha += PI;

				Marker newMarker;
				newMarker.id = marker.id;
				newMarker.distance = offset.mag() * pixelToMetre;
				newMarker.cartesian = Vec3d(newMarker.distance * sin(alpha), newMarker.distance * cos(alpha));
				visible.emplace_back(newMarker);
			}
		}
		return visible;
	}

	void update() {
		while (m_thetaUnknown > TWO_PI) m_thetaUnknown -= TWO_PI;
		while (m_thetaUnknown < 0) m_thetaUnknown += TWO_PI;
	}

public:
	/// <summary>
	/// Width and height of robot
	/// </summary>
	Vec3d m_size;

	// Position of camera within robot from top left corner
	Vec3d m_relCameraPos;

	/// <summary>
	/// Field of view of the camera. Defines how much the camera can
	/// see and therefore the number of fiducial markers visible
	/// </summary>
	double m_fov = 0;

	/// <summary>
	/// Screen-space variables used for drawing the robot to the window
	/// </summary>
	Vec3d m_posUnknown;
	double m_thetaUnknown = 0;
};
