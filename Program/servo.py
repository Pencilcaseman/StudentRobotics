RUGGEDUINO = None
servoCount = 0

class Servo:
    """
    The Servo object holds data about a servo, and can interface with the RUGGEDUINO.
    """
    def __init__(self, pin: int, angle: int = 0, minDutyCycle: int = 600, maxDutyCycle: int = 2400, minAngle: int = 0, maxAngle: int = 180, active: bool = True):
        """
        Initialize the servo and send the data to the ruggeduino.

        > pin: int - The pin the servo is connected to.

        > angle: int (0) - The angle to default the servo to.

        > minDutyCycle: int (600) - The minimum duty cycle of the servo.

        > maxDutyCycle: int (2400) - The maximum duty cycle of the servo.

        > minAngle: int (0) - The minimum angle the servo can be set to.

        > maxAngle: int (180) - The maximum angle the servo can be set to.

        > active: bool (True) - Wether the servo is enabled by default.
        """
        global servoCount

        if RUGGEDUINO is None:
            raise ValueError("Servo.RUGGEDUINO has not been set")

        self.pin = pin
        self.angle = angle
        self.minDutyCycle = minDutyCycle
        self.maxDutyCycle = maxDutyCycle
        self.minAngle = minAngle
        self.maxAngle = maxAngle
        self.active = active

        self.ID = servoCount
        servoCount += 1

        RUGGEDUINO.command(f"#ADDSERVO {self.ID}, {self.pin}, {self.angle}, {self.minDutyCycle}, {self.maxDutyCycle}, {self.minAngle}, {self.maxAngle}#")
    
    def attach(self):
        """
        Enable the servo.
        """
        RUGGEDUINO.command(f"#ENABLE {self.ID}#")

    def detach(self):
        """
        Disable the servo.
        """
        RUGGEDUINO.command(f"#DISABLE {self.ID}#")

    def getAngle(self):
        """
        Get the desired angle of the servo.

        > return: float - The desired angle of the servo.
        """
        return float(RUGGEDUINO.command(f"#GETANGLE {self.ID}#"))

    def setAngle(self, angle):
        """
        Set the desired angle of the servo.

        > angle: float - The desired angle of the servo.
        """
        RUGGEDUINO.command(f"#SETANGLE {self.ID}, {angle}#")
