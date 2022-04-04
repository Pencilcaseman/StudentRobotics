from multiprocessing.connection import answer_challenge
from sr.robot3 import *
import math

import vector
import servo
import marker
import can
import world

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

		self.worldView = world.World(vector.Vec3(5750, 5750))
		self.worldView.populateMarkers(28, 0.0)

		self.canPositionsFloor = (
			vector.Vec3(2.871819690265487, 0.0477046460176991),
			vector.Vec3(2.871819690265487, 1.1385508849557524),
			vector.Vec3(1.6187776548672568, 1.62195796460177),
			vector.Vec3(4.124861725663717, 1.62195796460177),
			vector.Vec3(2.474280973451328, 1.8732024336283188),
			vector.Vec3(3.2725387168141595, 1.8732024336283188),
			vector.Vec3(1.876382743362832, 2.4711006637168142),
			vector.Vec3(3.8736172566371683, 2.4711006637168142),
			vector.Vec3(0.05088495575221239, 2.8781803097345136),
			vector.Vec3(1.1385508849557524, 2.8781803097345136),
			vector.Vec3(4.614629424778761, 2.8781803097345136),
			vector.Vec3(5.705475663716815, 2.8781803097345136),
			vector.Vec3(1.876382743362832, 3.2693584070796464),
			vector.Vec3(3.8736172566371683, 3.2693584070796464),
			vector.Vec3(2.474280973451328, 3.870436946902655),
			vector.Vec3(3.2725387168141595, 3.870436946902655),
			vector.Vec3(1.6187776548672568, 3.9563053097345136),
			vector.Vec3(4.124861725663717, 3.9563053097345136),
			vector.Vec3(2.871819690265487, 4.614629424778761),
			vector.Vec3(2.871819690265487, 5.702295353982302),
		)

		self.canPositionsRaised = (
			vector.Vec3(2.5792311946902657, 2.315265486725664),
			vector.Vec3(3.173949115044248, 2.315265486725664),
			vector.Vec3(3.441095132743363, 2.572870575221239),
			vector.Vec3(3.441095132743363, 3.173949115044248),
			vector.Vec3(3.173949115044248, 3.441095132743363),
			vector.Vec3(2.5792311946902657, 3.441095132743363),
			vector.Vec3(2.315265486725664, 3.173949115044248),
			vector.Vec3(2.315265486725664, 2.569690265486726),
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
			print("INVALID SERVO '{}'".format(servo))
			return None

	def set_angle(self, servo: str, angle: float):
		try:
			self.get_servo(servo).setAngle(angle)
		except:
			print("[ ERROR ] hlep")
	
	def get_angle(self, servo: str):
		try:
			return self.get_servo(servo).getAngle()
		except:
			print("[ ERROR ] hleep")

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
		theta = math.asin(D2 * math.sin(gamma) * invMd)
		mu = math.asin(D1 * math.sin(gamma) * invMd)
		
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

	def calculateWorldspacePosition(self):
		visible = self.see()

		self.save_image("i_see_u.png")
		for seen in visible:
			print(f"[ VISIBLE ] Cartesian: {seen.cartesian} | ID: {seen.id}")

		if len(visible) < 2: return None, None

		sumPos = vector.Vec3(0, 0, 0)
		theta = 0

		for i in range(len(visible) - 1):
			coord1 = visible[i].cartesian
			coord2 = visible[i + 1].cartesian
			marker1 = marker.Marker(vector.Vec3(coord1.x, coord1.z), visible[i].id)
			marker2 = marker.Marker(vector.Vec3(coord2.x, coord2.z), visible[i + 1].id)
			pos, angle = self.computeRelativePosition(marker1, marker2)
			sumPos += pos
			theta = angle

		truePos = sumPos / (len(visible) - 1)
		trueAngle = theta + (math.pi * 2 if theta < -math.pi else 0)
		return truePos, trueAngle
