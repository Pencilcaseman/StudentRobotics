import vector

def fromSrMarker(srmarker):
	"""
	Create a marker.Marker object from an sr.marker object.

	> srmarker - The sr.marker object to convert.
	"""

	res = Marker(vector.Vec3(srmarker.cartesian.x, srmarker.cartesian.z), None, srmarker.id)
	res.orientation = srmarker.orientation
	res.spherical = srmarker.spherical
	return res

class Marker:
	"""
	Stores information about a marker.
	"""

	def __init__(self, cartesian: vector.Vector, angle: float, id: int = -1):
		"""
		Create a marker using a position and an ID.

		> cartesian: vector.Vector - The position of the marker.

		> angle: float - The angle of the line bisecting the marker from the left edge to the right edge, relative to the world

		> id: int (-1) - The id of the marker.
		"""

		self.cartesian = cartesian       # Cartesian coordinate (worldspace OR relative)
		self.angle = angle               # Normal angle relative to world
		self.id = id                     # Marker ID
		self.distance = cartesian.mag()  # Distance from camera to marker (when using relative coordinate space)

		self.orientation = None
		self.spherical = None
     