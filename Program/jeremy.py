from sr.robot3 import *
import math

import servo
import marker
import vector

WHEELS = {
	"fl" : ["SR0WAF", 0],
	"fr" : ["SR0GFJ", 1],
	"bl" : ["SR0WAF", 1],
	"br" : ["SR0GFJ", 0]
}

class Jeremy:
	def __init__(self):
		self.R = Robot()

		servo.SERVOBOARD = self.R.ruggeduino

		self.grabberServo = servo.Servo(9, 0, 600, 2400, 0, 180, True)
		self.armServo = servo.Servo(10, 0, 500, 2500, 0, 250, True)

	def drive_wheel(self, power: float, fb: str, lr: str):
		motor = ""

		if fb in ["front", "f"]:
			motor += "f"
		elif fb in ["back", "b"]:
			motor += "b"
		else:
			print("INVALID WHEEL SIDE '{}'".format(fb))
			return

		if lr in ["left", "l"]:
			motor += "l"
		elif lr in ["right", "r"]:
			motor += "r"
		else:
			print("INVALID WHEEL SIDE '{}'".format(lr))
			return

		try:
			self.R.motor_boards[WHEELS[motor][0]].motors[WHEELS[motor][1]].power = power
		except Exception as error:
			print("DRIVE WHEEL ERROR:", error)
	
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
			print("INVALID SERVO '{}'".format(servo))
			return None

	def set_angle(self, servo: str, angle: float):
		self.get_servo(servo).setAngle(angle)
	
	def get_angle(self, servo: str):
		return self.get_servo(servo).getAngle()

	def attach(self, servo: str):
		self.get_servo(servo).attach()

	def detach(self, servo: str):
		self.get_servo(servo).detach()
		
	def computeRelativePosition(self, marker1, marker2):
		M1 = marker1.cartesian
		M2 = marker2.cartesian
		Md = math.sqrt((M2.x - M1.x) * (M2.x - M1.x) + (M2.y - M1.y) * (M2.y - M1.y))
		invMd = 1.0 / Md
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
		theta = math.asin(D2 * math.sin(gamma)) * invMd
		mu = math.asin(D1 * math.sin(gamma)) * invMd
		
		# TODO: Create worldview
		worldspaceMarker1 = self.worldView.idealMarkerPosition(marker1.id)
		worldspaceMarker2 = self.worldView.idealMarkerPosition(marker2.id)
		
		markerDiffWorldSpace = worldspaceMarker2.cartesian - worldspaceMarker1.cartesian
		markerDiffRobotSpace = M2 - M1
		tau = math.atan2(markerDiffWorldSpace.y, markerDiffWorldSpace.x)
		kappa = math.atan2(markerDiffRobotSpace.y, markerDiffRobotSpace.x)
		
		relPosM1 = vector.Vec3(D1 * math.cos(theta + tau), D1 * math.sin(theta + tau))
		worldPosM1 = worldspaceMarker1.cartesian + relPosM1
		
		relPosM2 =  vector.Vec3(D2 * math.cos(math.pi - mu + tau), D2 * math.sin(math.pi - mu + tau))
		worldPosM2 = worldspaceMarker2.cartesian + relPosM2
		
		if (worldspaceMarker1.cartesian - worldPosM1).mag2() >= (worldspaceMarker2.cartesian - worldPosM2).mag2():
			worldPosTrue = worldPosM1
		else:
			worldPosTrue = worldPosM2
		
		return worldPosTrue, tau + kappa - (math.pi / 2)
