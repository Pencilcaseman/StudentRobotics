from numpy import char


RUGGEDUINO = None

class Screen:
    """
    The screen class is used to control a 8x8 screen connected to the ruggeduino.
    """
    def __init__(self):
        """
        Construct the screen.
        """
        if RUGGEDUINO is None:
            raise ValueError("Screen.RUGGEDUINO has not been set")

    def set(self, c: chr):
        """
        Sets the character displayed on the screen.

        > c: chr - The character to set to: [0, 9999]
        """ 
        RUGGEDUINO.command(f"#SETDISPLAY {c}#")