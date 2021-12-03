#pragma once

#include "ofMain.h"

#include <vector>
#include <cstdint>
#include <cmath>

#include "vector.hpp"

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

struct Marker {
public:
	Marker(double x = 0, double y = 0, double dist = 0) : x(x), y(y), dist(dist) {}
	double x, y, dist = 0;
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
		double markerDist = m_size.x / (double)markersPerSide;

		// Top side
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i] = Marker((i + 0.5) * markerDist, 0);

		// Right side
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i + markersPerSide] = Marker(m_size.x, (i + 0.5) * markerDist);

		// Bottom side
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i + markersPerSide * 2] = Marker((i + 0.5) * markerDist, m_size.y);

		// Left side
		for (int64_t i = 0; i < markersPerSide; ++i)
			m_markers[i + markersPerSide * 3] = Marker(0, (i + 0.5) * markerDist);
	}

	void draw() const {
		ofSetColor(170, 50, 50);
		ofNoFill();
		ofSetLineWidth(5);
		ofDrawRectangle(m_pos.x, m_pos.y, m_size.x, m_size.y);
		ofSetLineWidth(2);
		ofFill();

		ofSetColor(50, 170, 50);
		for (const auto& marker : m_markers)
			ofDrawCircle(m_pos.x + marker.x, m_pos.y + marker.y, 10);
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
	void setPosUnknown(double x, double y, double theta) {
		m_posUnknown.x = x;
		m_posUnknown.y = y;
		m_thetaUnknown = theta;
	}

	/// <summary>
	/// Draw the robot to the screen using its screen-space coordinates.
	/// </summary>
	void draw() const {
		ofSetColor(50, 170, 50);

		ofPushMatrix();
		ofTranslate(m_posUnknown.x, m_posUnknown.y);
		ofRotateRad(m_thetaUnknown - (PI / 2));
		ofDrawRectRounded(-m_size.x / 2, -m_size.y / 2, m_size.x, m_size.x, 10);
		ofSetColor(0);
		ofDrawCircle(m_size.x * -0.3, m_size.y * 0.3, 5);
		ofDrawCircle(m_size.x * 0.3, m_size.y * 0.3, 5);
		ofPopMatrix();

		// Find world-space origin of ray
		Vec3d origin = m_posUnknown;
		double rayAngle = m_fov / 2;
		Vec3d end1, end2;
		end1.x = origin.x + 100 * cos(rayAngle + m_thetaUnknown);
		end1.y = origin.y + 100 * sin(rayAngle + m_thetaUnknown);

		end2.x = origin.x + 100 * cos(-rayAngle + m_thetaUnknown);
		end2.y = origin.y + 100 * sin(-rayAngle + m_thetaUnknown);

		ofSetColor(255, 0, 0);
		ofDrawLine(origin.x, origin.y, end1.x, end1.y);
		ofDrawLine(origin.x, origin.y, end2.x, end2.y);
	}

	std::vector<Marker> see(const World& world) const {
		std::vector<Marker> visible;
		for (const auto& marker : world.m_markers) {
			// Position offsets from marker to robot
			Vec3d offset(
				world.m_pos.x + marker.x - m_posUnknown.x,
				world.m_pos.y + marker.y - m_posUnknown.y
			);

			double rayAngle = m_fov / 2;
			double rayTheta = atan(sin(rayAngle + m_thetaUnknown) / cos(rayAngle + m_thetaUnknown));

			double theta = atan(offset.y / offset.x);

			if (theta > 0 && rayTheta < 0) rayTheta += PI;

			// if (&marker == &world.m_markers[0])
			// 	std::cout << "DX: " << offset.x << " | DY: " << offset.y << " | " << "Theta: " << theta << " | Ray Theta MIN: " << rayTheta << " | Ray Theta MAX: " << rayTheta - rayAngle * 2 << "\n";

			if (theta < rayTheta && theta > rayTheta - rayAngle * 2 && offset.dot(Vec3d(cos(rayAngle + m_thetaUnknown), sin(rayAngle + m_thetaUnknown))) > 0) {
				ofSetColor(170, 170, 255);
				ofDrawCircle(world.m_pos.x + marker.x, world.m_pos.y + marker.y, 20);
			}

			// break;
		}
		return visible;
	}

	void update() {
		while (m_thetaUnknown > 3.1415926 * 2) m_thetaUnknown -= 3.1415926 * 2;
		while (m_thetaUnknown < 0) m_thetaUnknown += 3.1415926 * 2;
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
