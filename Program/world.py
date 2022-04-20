import random
import marker, vector, can
import math

class World:
	"""
	World object to store data about the world.
	"""
	def __init__(self, size: vector.Vector):
		self.size = size
		self.markers = []
		self.cans = []

	def populateMarkers(self, numMarkers: int, error: float = 0):
		"""
		Add markers into the world.

		> numMarkers (int) - Total number of markers to add into the world.

		> error: float (0) - The maximum random error in each direction of the marker positions.
		"""
		self.markers = [None for _ in range(numMarkers)]
		markersPerSide = numMarkers // 4
		markerDist = self.size.x / (markersPerSide + 1)

		# Top Side -- IDs count up -- Increment on X axis
		for i in range(markersPerSide):
			self.markers[i] = marker.Marker(vector.Vec3((i + 1) * markerDist + random.uniform(-error, error), 0), math.pi / 2 , i)

		# Right side -- IDs count up -- Increment on Y axis
		for i in range(markersPerSide):
			self.markers[i + markersPerSide] = marker.Marker(vector.Vec3(self.size.x, (i + 1) * markerDist + random.uniform(-error, error)), math.pi, i + markersPerSide)

		# Bottom side -- IDs count DOWN due to clock-wise ordering -- Increment on X axis
		for i in range(markersPerSide):
			self.markers[i + markersPerSide * 2] = marker.Marker(vector.Vec3((markersPerSide - i) * markerDist + random.uniform(-error, error), self.size.y), -math.pi / 2, i + markersPerSide * 2)

		# Left side -- IDs count DOWN due to clock-wise ordering -- Increment in Y axis
		for i in range(markersPerSide):
			self.markers[i + markersPerSide * 3] = marker.Marker(vector.Vec3(0, (markersPerSide - i) * markerDist + random.uniform(-error, error)), 0, i + markersPerSide * 3)

	def idealMarkerPosition(self, id: int):
		"""
		Get the ideal marker position for a marker.

		> id: int - The id of the marker to get.

		> return: marker.Marker - The ideal position for a marker.
		"""
		markersPerSide = len(self.markers) // 4
		markerDist = self.size.x / (markersPerSide + 1)

		if 0 <= id < markersPerSide:
			# Top side of the box
			return marker.Marker(vector.Vec3((id + 1) * markerDist, 0), math.pi / 2, id)

		if markersPerSide <= id < markersPerSide * 2:
			# Right side of the box
			return marker.Marker(vector.Vec3(self.size.x, (id - markersPerSide + 1) * markerDist), math.pi, id)

		if markersPerSide * 2 <= id < markersPerSide * 3:
			# Bottm side of the box
			return marker.Marker(vector.Vec3(self.size.x - (id - markersPerSide * 2 + 1) * markerDist, self.size.y), -math.pi / 2, id)

		if markersPerSide * 3 <= id < markersPerSide * 4:
			# Left side of the box
			return marker.Marker(vector.Vec3(0, self.size.y - (id - markersPerSide * 3 + 1) * markerDist), 0, id)

		print("[ ERROR ] Invalid Marker ID")

	def addCan(self, can: can.Can):
		"""
		Add a can into the world.

		> can: can.Can - The can to add.
		"""
		self.cans.append(can)