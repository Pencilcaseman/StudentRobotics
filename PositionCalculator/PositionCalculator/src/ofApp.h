#pragma once

#include "ofMain.h"

#include <vector>
#include <cstdint>
#include <cmath>
#include <sstream>
#include <random>

#include "vector.hpp"

// Just some stuff
static bool showPositionDebugInfo = false;
static bool showRaycastDebugInfo = false;

/**
 * Scale information:
 *
 * Every pixel represents 1cm. The actual box is a 5.75m +- 20cm square,
 * and the rendered board is 575 pixels square
 */

constexpr double pixelToMetre = 0.01;
constexpr double metreToPixel = 100;

// constexpr int64_t worldMarkers = 12; // Show a few markers for debugging purposes
// constexpr double markerError = 0.00;

// constexpr int64_t worldMarkers = 12; // Show a few markers with a large error for debugging purposes
// constexpr double markerError = 0.5;

constexpr int64_t worldMarkers = 28; // Actual number of markers +- 2cm (20mm)
constexpr double markerError = 0.02;

// constexpr int64_t worldMarkers = 100; // More markers for debugging purposes
// constexpr double markerError = 0.00;

// constexpr int64_t worldMarkers = 10000; // LOADS of markers to make sure stuff works
// constexpr double markerError = 0.00;

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

inline double random(double min, double max) {
	static std::mt19937_64 generator;
	std::uniform_real_distribution<double> dist(min, max);
	return dist(generator);
}

class Line {
public:
	Line(Vec3d start, Vec3d end) : start(start), end(end) {}

	std::pair<bool, Vec3d> intersects(const Line& other) const {
		/*
			y=m1(x-x1)+y1
			y=m2(x-x2)+y2

			m1(x-x1)+y1 = m2(x-x2)+y2

			m1(x-x1) - m2(x-x2) = y2 - y1

			m1*x - m1*x1 - m2 * x + m2*x2 = y2 - y1

			m1*x - m2*x = y2 - y1 + m1*x1 - m2*x2

			x(m1 - m2) = y2 - y1 + m1*x1 - m2*x2

			=========================================
			x = (y2 - y1 + m1*x1 - m2*x2) / (m1 - m2)
			y = m1(x-x1)+y1
			=========================================
		*/

		double m1 = (end.y - start.y) / (end.x - start.x);
		double m2 = (other.end.y - other.start.y) / (other.end.x - other.start.x);
		double x1 = start.x, y1 = start.y;
		double x2 = other.start.x, y2 = other.start.y;

		double xIntersect, yIntersect;

		// Check for edge cases
		if (abs(end.x - start.x) < 1E-5) {
			// Infinite gradient. X intersect is x1
			xIntersect = x1;
			yIntersect = m2 * (xIntersect - x2) + y2;
		}
		else if (abs(other.end.x - other.start.x) < 1E-5) {
			// Infinite gradient. X intersect is x2
			xIntersect = x2;
			yIntersect = m1 * (xIntersect - x1) + y1;
		}
		else {
			xIntersect = (y2 - y1 + m1 * x1 - m2 * x2) / (m1 - m2);
			yIntersect = m1 * (xIntersect - x1) + y1;
		}

		double epsilon = 1E-8;

		return {
			xIntersect > min(start.x, end.x) - epsilon &&
			xIntersect < max(start.x, end.x) + epsilon &&
			yIntersect > min(start.y, end.y) - epsilon &&
			yIntersect < max(start.y, end.y) + epsilon &&
			xIntersect > min(other.start.x, other.end.x) - epsilon &&
			xIntersect < max(other.start.x, other.end.x) + epsilon &&
			yIntersect > min(other.start.y, other.end.y) - epsilon &&
			yIntersect < max(other.start.y, other.end.y) + epsilon,
			Vec3d{xIntersect, yIntersect} };
	}

public:
	Vec3d start;
	Vec3d end;
};

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

class Can : public Marker {
public:
	Can() : Marker() {}
	Can(Vec3d cartesian, bool upright = true) : Marker(cartesian), upright(upright) {}

	bool upright;
};

class World {
public:
	World() = default;

	World(const Vec3d& p1, const Vec3d& size, const std::vector<Marker>& markers = std::vector<Marker>(0))
		: m_pos(p1), m_size(size), m_markers(markers) {}

	World(const World& other) {
		m_pos = other.m_pos;
		m_size = other.m_size;
		m_markers = other.m_markers;
	}

	World& operator=(const World& other) {
		m_pos = other.m_pos;
		m_size = other.m_size;
		m_markers = other.m_markers;
		return *this;
	};

	/// <summary>
	/// Place numMarkers markers around the edge of the box with a random error
	/// in range +- error (measured in meters)
	/// </summary>
	/// <param name="numMarkers"></param>
	/// <param name="errorM"></param>
	void populateMarkers(int64_t numMarkers, double error = 0) {
		m_markers = std::vector<Marker>(numMarkers);
		int64_t markersPerSide = numMarkers / 4;
		double markerDist = m_size.x / (double)(markersPerSide + 1);

		// Top side -- IDs count up -- Increment on X axis
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i] = Marker(Vec3d{ (i + 1) * markerDist + (metreToPixel * random(-error, error)), 0 }, i);

		// Right side -- IDs count up -- Increment on Y axis
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i + markersPerSide] = Marker(Vec3d{ m_size.x, (i + 1) * markerDist + (metreToPixel * random(-error, error)) }, i + markersPerSide);

		// Bottom side -- IDs count *down* due to clock-wise ordering -- Increment on X axis
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i + markersPerSide * 2] = Marker(Vec3d{ ((markersPerSide - i - 1) + 1) * markerDist + (metreToPixel * random(-error, error)), m_size.y }, i + markersPerSide * 2);

		// Left side -- IDs count *down* due to clock-wise ordering -- Increment on X axis
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i + markersPerSide * 3] = Marker(Vec3d{ 0, ((markersPerSide - i - 1) + 1) * markerDist + (metreToPixel * random(-error, error)) }, i + markersPerSide * 3);
	}

	Marker idealMarkerPosition(int64_t id) const {
		int64_t markersPerSide = m_markers.size() / 4;
		double markerDist = m_size.x / (double)(markersPerSide + 1);

		if (id >= 0 && id < markersPerSide) {
			// Top side of the box
			return Marker(Vec3d{ (id + 1) * markerDist, 0 }, id);
		}

		if (id < markersPerSide * 2) {
			// Right side of the box
			return Marker(Vec3d{ m_size.x, (id - markersPerSide + 1) * markerDist }, id);
		}

		if (id < markersPerSide * 3) {
			// Bottom side of the box
			return Marker(Vec3d{ m_size.x - (id - markersPerSide * 2 + 1) * markerDist, m_size.y }, id);
		}

		if (id < markersPerSide * 4) {
			// Left side of the box
			return Marker(Vec3d{ 0, m_size.y - (id - markersPerSide * 3 + 1) * markerDist }, id);
		}

		std::cout << "Cannot calculate position of marker " + std::to_string(id)
			+ " with " + std::to_string(markersPerSide) + " markers per side\n";
		std::exit(1);
	}

	void addCan(const Can& can) {
		m_cans.emplace_back(can);
	}

	void draw() const {
		// Draw scoring zones
		ofSetLineWidth(1);
		ofSetColor(0);
		ofDrawLine(m_pos.x, m_pos.y + 2.5 * metreToPixel, m_pos.x + 2.5 * metreToPixel, m_pos.y);
		ofDrawLine(m_pos.x + m_size.x - 2.5 * metreToPixel, m_pos.y, m_pos.x + m_size.x, m_pos.y + 2.5 * metreToPixel);
		ofDrawLine(m_pos.x + m_size.x, m_pos.y + m_size.y - 2.5 * metreToPixel, m_pos.x + m_size.x - 2.5 * metreToPixel, m_pos.y + m_size.y);
		ofDrawLine(m_pos.x, m_pos.y + m_size.y - 2.5 * metreToPixel, m_pos.x + 2.5 * metreToPixel, m_pos.y + m_size.y);

		// Draw raised section
		ofSetColor(150, 150, 150);
		ofDrawRectangle(m_pos.x + m_size.x / 2 - 60, m_pos.y + m_size.y / 2 - 60, 120, 120);

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

		// Draw all the cans as purple dots
		for (const auto& can : m_cans) {
			ofSetColor(235, 52, 113);
			ofDrawCircle(m_pos.x + can.cartesian.x, m_pos.y + can.cartesian.y, 10);

			if (can.upright) {
				ofSetColor(235, 86, 52);
				ofNoFill();
				ofDrawCircle(m_pos.x + can.cartesian.x, m_pos.y + can.cartesian.y, 10);
				ofFill();
			}
		}
	}

public:
	Vec3d m_pos;
	Vec3d m_size;
	std::vector<Marker> m_markers;
	std::vector<Can> m_cans;
};

class Robot {
public:
	Robot(World* world = nullptr) : m_world(world) {}

	Robot(World* world, const Vec3d& size, double fov = 72) : m_world(world) {
		m_size = size;
		m_relCameraPos = { 0, 0 };
		m_fov = deg2rad(fov);

		m_worldView = World({ 0, 0 }, { 5.75, 5.75 });
		m_worldView.populateMarkers(worldMarkers, 0.0);
	}

	Robot(const Robot& other) {
		m_size = other.m_size;
		m_relCameraPos = other.m_relCameraPos;
		m_fov = other.m_fov;
		m_world = other.m_world;
		m_worldView = other.m_worldView;
	}

	Robot& operator=(const Robot& other) {
		if (this == &other) return *this;
		m_size = other.m_size;
		m_relCameraPos = other.m_relCameraPos;
		m_fov = other.m_fov;
		m_world = other.m_world;
		m_worldView = other.m_worldView;
		return *this;
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
		// ofRotateRad(m_thetaUnknown - (PI / 2));
		ofRotateRad(m_thetaUnknown);
		ofDrawRectRounded(-m_size.x / 2, -m_size.y / 2, m_size.x, m_size.x, 10);
		ofSetColor(0);
		ofDrawCircle(m_size.x * 0.3, m_size.y * 0.3, 5);
		ofDrawCircle(m_size.x * 0.3, m_size.y * -0.3, 5);
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

	std::vector<Marker> see() const {
		// List of markers visible to the robot
		std::vector<Marker> visible;

		// Iterate over all markers
		for (const auto& marker : m_world->m_markers) {
			// Position offsets from marker to robot
			Vec3d offset(
				m_world->m_pos.x + marker.cartesian.x - m_posUnknown.x,
				m_world->m_pos.y + marker.cartesian.y - m_posUnknown.y
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
				ofDrawCircle(m_world->m_pos.x + marker.cartesian.x, m_world->m_pos.y + marker.cartesian.y, 15);

				// Label each marker
				if (m_world->m_markers.size() / 4 < 20) {
					ofSetColor(255);
					defaultFont.drawString(std::to_string(marker.id), m_world->m_pos.x + marker.cartesian.x - 30, m_world->m_pos.y + marker.cartesian.y - 30);
				}

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

	/// <summary>
	/// Calculate the position of the robot relative to two markers. This can then be run with
	/// different combinations of markers to obtain a more precise measurement of the robot's
	/// position in the box.
	/// </summary>
	/// <param name="marker1">= First visible marker (left to right)</param>
	/// <param name="marker2">= Second visible marker (left to right)</param>
	/// <returns></returns>
	std::pair<Vec3d, double> computeRelativePosition(const Marker& marker1, const Marker& marker2) const {
		// Note: The maths used here was calculated with the "SR-Position-Math.png" on GitHub
		//       Variables have the same names and perform the same function

		Vec3d M1 = marker1.cartesian;
		Vec3d M2 = marker2.cartesian;
		double Md = sqrt((M2.x - M1.x) * (M2.x - M1.x) + (M2.y - M1.y) * (M2.y - M1.y));
		double invMd = 1.0 / Md; // (M2.x + M1.x);
		double D1 = M1.mag();
		double D2 = M2.mag();
		double alpha = abs(M1.x) > 1E-5 ? atan(M1.y / M1.x) : HALF_PI;
		double beta = abs(M2.x) > 1E-5 ? atan(M2.y / M2.x) : HALF_PI;

		if (M1.x < 0) alpha += PI;
		if (M2.x < 0) beta += PI;
		alpha = PI - alpha;

		// if (alpha < 90 && alphaTmp < 0) alpha += HALF_PI;

		double gamma = PI - alpha - beta;
		double theta = asin(D2 * sin(gamma) * invMd);
		double mu = asin(D1 * sin(gamma) * invMd);

		// if (M1.x < 0 && theta < HALF_PI)
		// if (M1.x > 0) theta = PI - theta;
		// if (M2.x > 0 && mu < 0) mu *= -1;

		// defaultFont.drawString("Alpha: " + std::to_string(rad2deg(alpha)) + "�\n" +
		// 	"Beta: " + std::to_string(rad2deg(beta)) + "�\n" +
		// 	"Theta: " + std::to_string(rad2deg(theta)) + "�\n" +
		// 	"Gamma: " + std::to_string(rad2deg(gamma)) + "�\n" +
		// 	"Mu: " + std::to_string(rad2deg(mu)) + "�\n" +
		// 	"sin(theta): " + std::to_string(D2 * sin(gamma) * invMd), 300, 500);

		Marker worldspaceMarker1 = m_worldView.idealMarkerPosition(marker1.id);
		Marker worldspaceMarker2 = m_worldView.idealMarkerPosition(marker2.id);

		// Calculate the angle between the markers to the horizontal
		Vec3d markerDiffWorldSpace = worldspaceMarker2.cartesian - worldspaceMarker1.cartesian;
		Vec3d markerDiffRobotSpace = M2 - M1;
		double tau = atan2(markerDiffWorldSpace.y, markerDiffWorldSpace.x);
		double kappa = atan2(markerDiffRobotSpace.y, markerDiffRobotSpace.x);

		// Relative position in *world space* to the first marker
		Vec3d relPosM1 = { D1 * cos(theta + tau), D1 * sin(theta + tau) };
		Vec3d worldPosM1 = worldspaceMarker1.cartesian + relPosM1;

		// Relative position in *world space* to the second marker
		Vec3d relPosM2 = { D2 * cos(PI - mu + tau), D2 * sin(PI - mu + tau) };
		Vec3d worldPosM2 = worldspaceMarker2.cartesian + relPosM2;

		Vec3d worldPosTrue;

		if ((worldspaceMarker1.cartesian - worldPosM1).mag2() >= (worldspaceMarker2.cartesian - worldPosM2).mag2())
			worldPosTrue = worldPosM1;
		else
			worldPosTrue = worldPosM2;

		if (showPositionDebugInfo) {
			// =================================================================
			// Draw a shit-ton of debugging things
			// =================================================================

			ofDrawRectangle(m_world->m_pos.x + markerDiffRobotSpace.x * metreToPixel, m_world->m_pos.y + markerDiffRobotSpace.y * metreToPixel, 20, 20);

			ofPushMatrix();
			ofTranslate(m_posUnknown.x, m_posUnknown.y);
			ofRotateRad(m_thetaUnknown + HALF_PI);

			// // Draw a line going directly up
			// ofSetColor(255, 0, 0);
			// ofDrawLine(0, 0, 0, -100);
			// // Draw a line going directly right
			// ofSetColor(0, 0, 255);
			// ofDrawLine(0, 0, 100, 0);

			ofSetColor(255, 120, 17);
			ofDrawLine(0, 0, M1.x * metreToPixel, 0);
			ofDrawLine(M1.x * metreToPixel, 0, M1.x * metreToPixel, -M1.y * metreToPixel);
			ofDrawLine(0, 0, M1.x * metreToPixel, -M1.y * metreToPixel);

			ofDrawLine(0, 0, M2.x * metreToPixel, 0);
			ofDrawLine(M2.x * metreToPixel, 0, M2.x * metreToPixel, -M2.y * metreToPixel);
			ofDrawLine(0, 0, M2.x * metreToPixel, -M2.y * metreToPixel);

			ofSetColor(59, 231, 237);
			ofDrawCircle(M1.x * metreToPixel, 0, 10);
			ofDrawCircle(M2.x * metreToPixel, 0, 10);
			ofDrawCircle(M1.x * metreToPixel, -M1.y * metreToPixel, 10);
			ofDrawCircle(M2.x * metreToPixel, -M2.y * metreToPixel, 10);

			ofPopMatrix();

			ofSetColor(226, 38, 255);
			ofDrawLine(m_world->m_pos.x + worldspaceMarker1.cartesian.x * metreToPixel, m_world->m_pos.y + worldspaceMarker1.cartesian.y * metreToPixel,
				m_world->m_pos.x + worldspaceMarker1.cartesian.x * metreToPixel + 100 * cos(theta + tau), m_world->m_pos.y + worldspaceMarker1.cartesian.y * metreToPixel + 100 * sin(theta + tau));
			ofDrawCircle(m_world->m_pos.x + worldPosM1.x * metreToPixel, m_world->m_pos.y + worldPosM1.y * metreToPixel, 10);

			ofSetColor(201, 255, 38);
			ofDrawLine(m_world->m_pos.x + worldspaceMarker2.cartesian.x * metreToPixel, m_world->m_pos.y + worldspaceMarker2.cartesian.y * metreToPixel,
				m_world->m_pos.x + worldspaceMarker2.cartesian.x * metreToPixel + 100 * cos(PI - mu + tau), m_world->m_pos.y + worldspaceMarker2.cartesian.y * metreToPixel + 100 * sin(PI - mu + tau));
			ofDrawCircle(m_world->m_pos.x + worldPosM2.x * metreToPixel, m_world->m_pos.y + worldPosM2.y * metreToPixel, 10);

			ofSetColor(255, 38, 38);
			ofDrawCircle(m_world->m_pos.x + worldPosTrue.x * metreToPixel, m_world->m_pos.y + worldPosTrue.y * metreToPixel, 10);

			ofSetColor(50, 50, 255);
			ofDrawLine(m_posUnknown.x, m_posUnknown.y, m_posUnknown.x + 50 * cos(tau + kappa - HALF_PI), m_posUnknown.y - 50 * sin(tau + kappa - HALF_PI));

			defaultFont.drawString(std::to_string(rad2deg(tau)) + "\n" + std::to_string(rad2deg(tau + kappa)), 500, 500);
		}

		return { worldPosTrue, tau + kappa - HALF_PI };
	}

	/// <summary>
	/// Calculate the position of the Robot within the bounds of the world, based on
	/// information from the visible markers. Also calculate the angle of the robot
	/// relative to the world (in radians)
	/// </summary>
	std::pair<Vec3d, double> calculateWorldspacePosition() const {
		// Note: The maths used here was calculated with the "SR-Position-Math.png" on GitHub
		//       Variables have the same names and perform the same function

		std::vector<Marker> visible = see();
		if (visible.size() == 0) return {};

		Vec3d sumPos;
		double theta = 0;
		for (int64_t i = 0; i < visible.size() - 1; ++i)
		{
			auto triPos = computeRelativePosition(visible[i], visible[i + 1]);
			sumPos += triPos.first;
			// theta += triPos.second;
			theta = triPos.second;
		}

		// return { sumPos / ((double)visible.size() - 1), theta / (double) visible.size() };
		return { sumPos / ((double)visible.size() - 1), theta < -PI ? theta + TWO_PI : theta };
	}

	/// <summary>
	/// Return the distance to the nearest wall or world-space object
	/// </summary>
	/// <returns></returns>
	Vec3d projectedIntersection() const {
		std::pair<Vec3d, double> position = calculateWorldspacePosition();
		Line robotLine({ position.first.x, position.first.y }, 4 * Vec3d{ cos(position.second), sin(position.second) } + Vec3d{ position.first.x, position.first.y });

		std::vector<std::pair<bool, Vec3d>> intersections = {
			robotLine.intersects(boxEdge0),
			robotLine.intersects(boxEdge1),
			robotLine.intersects(boxEdge2),
			robotLine.intersects(boxEdge3),
			robotLine.intersects(raisedEdge0),
			robotLine.intersects(raisedEdge1),
			robotLine.intersects(raisedEdge2),
			robotLine.intersects(raisedEdge3)
		};

		double minMag = 1E100;
		Vec3d intersection;
		for (const auto& hit : intersections) {
			if (hit.first && showRaycastDebugInfo) {
				ofSetColor(122, 120, 35);
				ofDrawCircle(m_world->m_pos.x + hit.second.x * metreToPixel, m_world->m_pos.y + hit.second.y * metreToPixel, 10);
			}

			if (hit.first && (position.first - hit.second).mag2() < minMag) {
				minMag = (position.first - hit.second).mag2();
				intersection = hit.second;
			}
		}

		if (showRaycastDebugInfo) {
			ofSetColor(3, 82, 252);
			for (const auto& edge : { robotLine, boxEdge0, boxEdge1, boxEdge2, boxEdge3, raisedEdge0, raisedEdge1, raisedEdge2, raisedEdge3 }) {
				ofDrawLine(m_world->m_pos.x + edge.start.x * metreToPixel, m_world->m_pos.y + edge.start.y * metreToPixel,
					m_world->m_pos.x + edge.end.x * metreToPixel, m_world->m_pos.y + edge.end.y * metreToPixel);
			}

			ofSetColor(242, 242, 39);
			ofDrawCircle(m_world->m_pos.x + intersection.x * metreToPixel, m_world->m_pos.y + intersection.y * metreToPixel, 10);
		}



		return intersection;
	}

	void update() {
		while (m_thetaUnknown > TWO_PI) m_thetaUnknown -= TWO_PI;
		while (m_thetaUnknown < 0) m_thetaUnknown += TWO_PI;
	}

public:
	// Reference to the World object this Robot is contained in
	World* m_world;

	World m_worldView;

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

	// Hard-coded stuff for performance improvements
	const Line boxEdge0 = Line({ 0, 0 }, { 5.75, 0 }); // Top edge of box
	const Line boxEdge1 = Line({ 5.75, 0 }, { 5.75, 5.75 }); // Right edge of box
	const Line boxEdge2 = Line({ 0, 5.75 }, { 5.75, 5.75 }); // Bottom edge of box
	const Line boxEdge3 = Line({ 0, 0 }, { 0, 5.75 }); // Left edge of box

	const Line raisedEdge0 = Line({ 5.75 / 2 - 0.60, 5.75 / 2 - 0.60 }, { 5.75 / 2 + 0.60, 5.75 / 2 - 0.60 }); // Top edge of raised area
	const Line raisedEdge1 = Line({ 5.75 / 2 + 0.60, 5.75 / 2 - 0.60 }, { 5.75 / 2 + 0.60, 5.75 / 2 + 0.60 }); // Right edge of raised area
	const Line raisedEdge2 = Line({ 5.75 / 2 - 0.60, 5.75 / 2 + 0.60 }, { 5.75 / 2 + 0.60, 5.75 / 2 + 0.60 }); // Bottom edge of raised area
	const Line raisedEdge3 = Line({ 5.75 / 2 - 0.60, 5.75 / 2 - 0.60 }, { 5.75 / 2 - 0.60, 5.75 / 2 + 0.60 }); // Left edge of raised area
};
