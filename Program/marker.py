import vector

def fromSrMarker(srmarker):
	"""
	Create a marker.Marker object from an sr.marker object.

	> srmarker - The sr.marker object to convert.
	"""

	return Marker(vector.Vec3(srmarker.cartesian.x, srmarker.cartesian.z), srmarker.id)

class Marker:
	"""
	Stores information about a marker.
	"""

	def __init__(self, cartesian: vector.Vector, id: int = -1):
		"""
		Create a marker using a position and an ID.

		> cartesian: vector.Vector - The position of the marker.

		> id: int (-1) - The id of the marker.
		"""

		self.cartesian = cartesian       # Cartesian coordinate (worldspace OR relative)
		self.id = id                     # Marker ID
		self.distance = cartesian.mag()  # Distance from camera to marker (when using relative coordinate space)
     