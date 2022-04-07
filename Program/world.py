import random

import marker
import vector

class World:
	def __init__(self, size):
		self.size = size
		self.markers = []
		self.cans = []

	def populateMarkers(self, numMarkers, error=0):
		self.markers = [None for _ in range(numMarkers)]
		markersPerSide = numMarkers // 4
		markerDist = self.size.x / (markersPerSide + 1)

		# Top Side -- IDs count up -- Increment on X axis
		for i in range(markersPerSide):
			self.markers[i] = marker.Marker(vector.Vec3((i + 1) * markerDist + random.uniform(-error, error), 0), i)

		# Right side -- IDs count up -- Increment on Y axis
		for i in range(markersPerSide):
			self.markers[i + markersPerSide] = marker.Marker(vector.Vec3(self.size.x, (i + 1) * markerDist + random.uniform(-error, error)), i + markersPerSide)

		# Bottom side -- IDs count DOWN due to clock-wise ordering -- Increment on X axis
		for i in range(markersPerSide):
			self.markers[i + markersPerSide * 2] = marker.Marker(vector.Vec3((markersPerSide - i) * markerDist + random.uniform(-error, error), self.size.y), i + markersPerSide * 2)

		# Left side -- IDs count DOWN due to clock-wise ordering -- Increment in Y axis
		for i in range(markersPerSide):
			self.markers[i + markersPerSide * 3] = marker.Marker(vector.Vec3(0, (markersPerSide - i) * markerDist + random.uniform(-error, error)), i + markersPerSide * 3)

	def idealMarkerPosition(self, id):
		markersPerSide = len(self.markers) // 4
		markerDist = self.size.x / (markersPerSide + 1)

		if 0 <= id < markersPerSide:
			# Top side of the box
			return marker.Marker(vector.Vec3((id + 1) * markerDist, 0), id)

		if markersPerSide <= id < markersPerSide * 2:
			# Top side of the box
			return marker.Marker(vector.Vec3(self.size.x, (id + 1) * markerDist), id)

		if markersPerSide * 2 <= id < markersPerSide * 3:
			# Top side of the box
			return marker.Marker(vector.Vec3(self.size.x - (id - markersPerSide * 2 + 1), * markerDist, self.size.y), id)

		if markersPerSide * 3 <= id < markersPerSide * 4:
			# Top side of the box
			return marker.Marker(vector.Vec3(0, self.size.y - (id - markersPerSide * 3 + 1) * markerDist), id)

		print("[ ERROR ] Invalid Marker ID")

	def addCan(self, can):
		self.cans.append(can)