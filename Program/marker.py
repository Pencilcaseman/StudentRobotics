"""
class Marker {
public:
	// NOTE: Some information is not stored. This is to make the program simpler

	Marker() : id(-1), distance(-1), cartesian({ 0, 0 }) {}

	Marker(Vec3d cartesian, int64_t id = -1)
		: cartesian(cartesian), distance(cartesian.mag()), id(id) {}

	Marker(const Marker& other)
		: id(other.id), distance(other.distance), cartesian(other.cartesian) {}

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
"""

import vector

class Marker:
    def __init__(self, cartesian, id = -1):
        self.cartesian = cartesian       # Cartesian coordinate (worldspace OR relative)
        self.id = id                     # Marker ID
        self.distance = cartesian.mag()  # Distance from camera to marker (when using relative coordinate space)
     