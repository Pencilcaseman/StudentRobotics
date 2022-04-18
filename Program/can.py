import marker, vector

class Can(marker.Marker):
    """
    A class extending marker.Marker, which holds the position of a can.
    """
    def __init__(self, cartesian: vector.Vector, upright: bool, radius: float):
        """
        A constructor to create the can object.

        > cartesian: vector.Vector - The position of the can in the world.

        > upright: bool - Wether the can is upright.

        > radius: float - The radius of the can.
        """
        super().__init__(cartesian, None)
        self.upright = upright
        self.radius = radius
