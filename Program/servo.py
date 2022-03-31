SERVOBOARD = None
servoCount = 0

class Servo:
    def __init__(self, pin, angle=0, minDutyCycle=600, maxDutyCycle=2400, minAngle = 0, maxAngle=180, active=True):
        global servoCount

        if SERVOBOARD is None:
            raise ValueError("Servo.SERVOBOARD has not been set")

        self.pin = pin
        self.angle = angle
        self.minDutyCycle = minDutyCycle
        self.maxDutyCycle = maxDutyCycle
        self.minAngle = minAngle
        self.maxAngle = maxAngle
        self.active = active

        self.ID = servoCount
        servoCount += 1

        SERVOBOARD.command(f"#ADDSERVO {self.ID}, {self.pin}, {self.angle}, {self.minDutyCycle}, {self.maxDutyCycle}, {self.minAngle}, {self.maxAngle}#")
    
    def attach(self):
        SERVOBOARD.command(f"#ENABLE {self.ID}#")

    def detach(self):
        SERVOBOARD.command(f"#DISABLE {self.ID}#")

    def getAngle(self):
        return float(SERVOBOARD.command(f"#GETANGLE {self.ID}#"))

    def setAngle(self, angle):
        SERVOBOARD.command(f"#SETANGLE {self.ID}, {angle}#")
