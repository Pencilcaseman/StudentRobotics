RUGGEDUINO = None
lcdInitialized = False

class Screen:
    """
    The screen class is used to control a 8x8 screen connected to the ruggeduino.
    """
    def __init__(self):
        global lcdInitialized

        """
        Construct the screen.
        """
        if RUGGEDUINO is None:
            raise ValueError("Screen.RUGGEDUINO has not been set")

        if not lcdInitialized:
            RUGGEDUINO.command("#INITLCD#")
            lcdInitialized = True

    def set(self, c: str, l: int):
        """
        Sets the character displayed on the screen.

        > c: str - The string to write to the LCD.
        
        > l: int - The line to write to [0, 1].
        """ 

        if len(c) > 16:
            print("[ WARN ] Cannot have string wiht more than 16 characters. Use another line, dumbass")
            c = c[:16]
        RUGGEDUINO.command(f"#WRITELCD {c}, {l}#")