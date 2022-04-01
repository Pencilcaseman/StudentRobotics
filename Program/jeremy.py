from sr.robot3 import *
import servo
import math
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
		M1 = Vec3d.marker1.cartesian ##no idea how the vec3d fits in so ive plonked it in there
		M2 = Vec3d.marker2.cartesian
		Md = float(sqrt((M2.x - M1.x) * (M2.x - M1.x) + (M2.y - M1.y) * (M2.y - M1.y)))
		invMd = float(1.0 / Md)
		D1 = float(M1.mag()) ## dont know what mag is and where to find it so ive just left it in
		D2 = float(M2.mag())
		alpha = float(math.atan(M1.y / M1.x) if abs(M1.x) > (1 * (10 **- 5)) else math.pi/2)
		beta = float(maths.atan(M2.y / M2.x) if abs(M2.x) > (1 * (10 **- 5)) else math.pi/2)
		
		if (M1.x < 0):
			alpha += math.pi
		if (M2.x < 0):
			beta += math.pi
		alpha = math.pi - alpha
		
		gamma = float(math.pi - alpha - beta)
		theta = float(math.asin(D2 * math.sin(gamma)) * invMd)
		mu = float(math.asin(D1 * math.sin(gamma)) * invMd)
		
		worldspaceMarker1 = Marker.m_worldView.idealMarkerPosition(marker1.id) ## dunno if this is where Marker goes
		worldspaceMarker2 = Marker.m_worldView.idealMarkerPosition(marker2.id)
		
		markerDiffWorldSpace = Vec3d(worldspaceMarker2.cartesian - worldspaceMarker1.cartesian) ##lol wtf is Vec3d, i have put it in how i feel like it, vibes n that
		markerDiffRobotSpace = Vec3d(M2 - M1)
		tau = float(math.atan2(markerDiffWorldSpace.y, markerDiffWorldSpace.x))
		kappa = float(math.atan2(markerDiffRobotSpace.y, markerDiffRobotSpace.x))
		
		relPosM1 = Vec3d( D1 * cos(theta + tau), D1 * sin(theta + tau) )
		worldPosM1 = Vec3d(worldspaceMarker1.cartesian + relPosM1)
		
		relPosM2 = float( D2 * cos(PI - mu + tau), D2 * sin(PI - mu + tau) )
		worldPosM2 = float(worldspaceMarker2.cartesian + relPosM2)
		
		# "Vec3d worldPosTrue" didnt know how to put this in, its an initiation right?
		
		if ((worldspaceMarker1.cartesian - worldPosM1).mag2() >= (worldspaceMarker2.cartesian - worldPosM2).mag2())
			worldPosTrue = worldPosM1;
		else
			worldPosTrue = worldPosM2;
		
			##ive gone through this whole bit not knowing hpw to asign variables with custom data types(?)
			## start from line 757 in other program
