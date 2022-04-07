import marker

class Can(marker.Marker):
    def __init__(self, cartesian, upright, radius):
        super().__init__(cartesian)
        self.upright = upright
        self.radius = radius
