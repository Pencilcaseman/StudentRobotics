RUGGEDUINO = None

class Screen:
    """
    The screen class is used to control a 7-segment, 4-digit screen connected to the ruggeduino.
    """
    def __init__(self):
        """
        Construct the screen.
        """
        if RUGGEDUINO is None:
            raise ValueError("Screen.RUGGEDUINO has not been set")

    def setNum(self, v: int):
        """
        Sets the number displayed on the screen.

        > v: int - The number to set to: [0, 9999]
        """ 
        if len(str(v)) > 4:
            raise ValueError("Number must contain less than 5 digits.")
        RUGGEDUINO.command(f"#SETNUM {v}#")

    def setDot(self, v: int):
        """
        Sets the position of the dot on the screen.

        > v: int - The position to set to: [0, 4]. Setting it to 4 removes the dot.
        """ 
        if 0 > v or v > 4:
            raise ValueError("Dot must be between 0 and 4 (inclusive).")
        RUGGEDUINO.command(f"#SETDOT {v}#")
