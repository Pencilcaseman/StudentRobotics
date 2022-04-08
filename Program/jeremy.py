from sr.robot3 import *
import math, time
import vector, servo, marker, can, world

WHEELS = {
    "fl": ["SR0WAF", 0],
    "fr": ["SR0GFJ", 1],
    "bl": ["SR0WAF", 1],
    "br": ["SR0GFJ", 0]
}


class Jeremy:
    def __init__(self, debug: bool):
        self.R = Robot()

        servo.SERVOBOARD = self.R.ruggeduino

        self.grabberServo = servo.Servo(9, 0, 600, 2400, 0, 180, True)
        self.armServo = servo.Servo(10, 0, 500, 2500, 0, 250, True)

        self.worldView = world.World(vector.Vec3(5750, 5750))
        self.worldView.populateMarkers(28, 0.0)

        self.debug = debug

		# Positions calculated from PositionCalculator program.
		# First output in console when EXE is run.
        self.canPositionsFloor = (
            vector.Vec3(2.875000, 0.033500, 0.000000),
            vector.Vec3(0.033500, 2.875000, 0.000000),
            vector.Vec3(5.716500, 2.875000, 0.000000),
            vector.Vec3(2.875000, 5.716500, 0.000000),
            vector.Vec3(2.875000, 1.135000, 0.000000),
            vector.Vec3(1.135000, 2.875000, 0.000000),
            vector.Vec3(4.615000, 2.875000, 0.000000),
            vector.Vec3(2.875000, 4.615000, 0.000000),
            vector.Vec3(1.625000, 1.625000, 0.000000),
            vector.Vec3(4.125000, 1.625000, 0.000000),
            vector.Vec3(1.625000, 4.125000, 0.000000),
            vector.Vec3(4.125000, 4.125000, 0.000000),
            vector.Vec3(2.475000, 1.875000, 0.000000),
            vector.Vec3(3.275000, 1.875000, 0.000000),
            vector.Vec3(1.875000, 2.475000, 0.000000),
            vector.Vec3(1.875000, 3.275000, 0.000000),
            vector.Vec3(3.875000, 2.475000, 0.000000),
            vector.Vec3(3.875000, 3.275000, 0.000000),
            vector.Vec3(2.475000, 3.875000, 0.000000),
            vector.Vec3(3.275000, 3.875000, 0.000000)
        )

        self.canPositionsRaised = (
            vector.Vec3(2.575000, 2.308500, 0.000000),
            vector.Vec3(3.175000, 2.308500, 0.000000),
            vector.Vec3(2.308500, 2.575000, 0.000000),
            vector.Vec3(2.308500, 3.175000, 0.000000),
            vector.Vec3(3.441500, 2.575000, 0.000000),
            vector.Vec3(3.441500, 3.175000, 0.000000),
            vector.Vec3(2.575000, 3.441500, 0.000000),
            vector.Vec3(3.175000, 3.441500, 0.000000)
        )

        for canPos in self.canPositionsFloor:
            self.worldView.addCan(can.Can(canPos, False, 0.067))

        for canPos in self.canPositionsRaised:
            self.worldView.addCan(can.Can(canPos, True, 0.067))

    def drive_wheel(self, power: float, fb: str, lr: str):
        motor = ""

        if fb in ["front", "f"]:
            motor += "f"
        elif fb in ["back", "b"]:
            motor += "b"
        else:
            self.warn("INVALID WHEEL SIDE '{}'".format(fb))
            return

        if lr in ["left", "l"]:
            motor += "l"
        elif lr in ["right", "r"]:
            motor += "r"
        else:
            self.warn("INVALID WHEEL SIDE '{}'".format(lr))
            return

        try:
            self.R.motor_boards[WHEELS[motor][0]
                                ].motors[WHEELS[motor][1]].power = power
        except Exception as e:
            self.error(f"Error while driving wheel: {str(e)}")

    def drive(self, power: float):
        self.drive_wheel(power, "front", "left")
        self.drive_wheel(power, "front", "right")
        self.drive_wheel(power, "back", "left")
        self.drive_wheel(power, "back", "right")

    def turn(self, power: float):
        self.drive_wheel(power, "front", "left")
        self.drive_wheel(-power, "front", "right")
        self.drive_wheel(power, "back", "left")
        self.drive_wheel(-power, "back", "right")

    def stop(self):
        self.drive(0)

    def see(self):
        return self.R.camera.see()

    def save_image(self, name: str):
        return self.R.camera.save(self.R.usbkey / name)

    def find_markers(self):
        return self.R.camera.see()

    def get_servo(self, servo: str):
        if servo.lower() in ["g", "grab", "grabber", "grabberservo"]:
            return self.grabberServo
        elif servo.lower() in ["a", "arm", "armservo"]:
            return self.armServo
        else:
            self.warn("INVALID SERVO '{}'".format(servo))
            return None

    def set_angle(self, servo: str, angle: float):
        try:
            self.get_servo(servo).setAngle(angle)
        except Exception as e:
            self.error(f"Error while setting angle: {str(e)}")

    def get_angle(self, servo: str):
        try:
            return self.get_servo(servo).getAngle()
        except Exception as e:
            self.error(f"Error while getting angle: {str(e)}")

    def get_rot(self):
        try:
            res = [float(i) for i in self.R.ruggeduino.command("#GETROT#").split(" ")]
            vec = vector.Vec3(res[0], res[1], res[2])
            self.log(f"Recieved vector {str(vec)}")
            return vec
        except Exception as e:
            self.error(f"Error while getting rotation: {str(e)}")

    def attach(self, servo: str):
        self.get_servo(servo).attach()

    def detach(self, servo: str):
        self.get_servo(servo).detach()

    def computeRelativePosition(self, marker1, marker2):
        M1 = marker1.cartesian
        M2 = marker2.cartesian
        invMd = ((M2.x - M1.x) * (M2.x - M1.x) +
                 (M2.y - M1.y) * (M2.y - M1.y)) ** -0.5
        D1 = M1.mag()
        D2 = M2.mag()
        alpha = math.atan(M1.y / M1.x) if abs(M1.x) > 1E-5 else math.pi/2
        beta = math.atan(M2.y / M2.x) if abs(M2.x) > 1E-5 else math.pi/2

        if (M1.x < 0):
            alpha += math.pi
        if (M2.x < 0):
            beta += math.pi

        alpha = math.pi - alpha

        gamma = math.pi - alpha - beta
        theta = math.asin(D2 * math.sin(gamma) * invMd)
        mu = math.asin(D1 * math.sin(gamma) * invMd)

        # TODO: Create worldview
        worldspaceMarker1 = self.worldView.idealMarkerPosition(marker1.id)
        worldspaceMarker2 = self.worldView.idealMarkerPosition(marker2.id)

        markerDiffWorldSpace = worldspaceMarker2.cartesian - worldspaceMarker1.cartesian
        markerDiffRobotSpace = M2 - M1
        tau = math.atan2(markerDiffWorldSpace.y, markerDiffWorldSpace.x)
        kappa = math.atan2(markerDiffRobotSpace.y, markerDiffRobotSpace.x)

        relPosM1 = vector.Vec3(D1 * math.cos(theta + tau),
                               D1 * math.sin(theta + tau))
        worldPosM1 = worldspaceMarker1.cartesian + relPosM1

        relPosM2 = vector.Vec3(
            D2 * math.cos(math.pi - mu + tau), D2 * math.sin(math.pi - mu + tau))
        worldPosM2 = worldspaceMarker2.cartesian + relPosM2

        if (worldspaceMarker1.cartesian - worldPosM1).mag2() >= (worldspaceMarker2.cartesian - worldPosM2).mag2():
            worldPosTrue = worldPosM1
        else:
            worldPosTrue = worldPosM2

        return worldPosTrue, tau + kappa - (math.pi / 2)

    def calculateWorldspacePosition(self):
        markers = self.see()
        for seen in markers:
            self.log(f"[ VISIBLE ] Cartesian: {seen.cartesian} | ID: {seen.id}")

        if len(markers) < 2:
            return None, None

        sumPos = vector.Vec3(0, 0, 0)
        theta = 0

        for i in range(len(markers) - 1):
            marker1 = marker.fromSrMarker(markers[i])
            marker2 = marker.fromSrMarker(markers[i + 1])
            pos, angle = self.computeRelativePosition(marker1, marker2)
            sumPos += pos
            theta = angle

        truePos = sumPos / (len(markers) - 1)
        trueAngle = theta + (math.pi * 2 if theta < -math.pi else 0)
        return truePos, trueAngle

    def sleep(self, t: float):
        time.sleep(t)

    def log(self, msg: str):
        if (self.debug):
            print("[ INFO ] " + str(msg))

    def warn(self, msg: str):
        if (self.debug):
            print("[ WARN ] " + str(msg))

    def error(self, msg: str):
        print("[ ERROR ] " + str(msg))

