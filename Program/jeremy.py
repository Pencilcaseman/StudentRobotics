"""
Main Arm Servo:
180 deg = Front
0 deg = Back

Grabber Arm Servo:
0 deg = Grab can
20 deg = Open
"""

import sr.robot3 as sr
import math, time
from multiprocessing import Pool
import vector, servo, marker, can, world, screen

WHEELS = {
	"fl": ["SR0WAF", 0],
	"fr": ["SR0GFJ", 1],
	"bl": ["SR0WAF", 1],
	"br": ["SR0GFJ", 0]
}

class Jeremy:
	"""
	The main controller for the robot! This is the only thing robot.py should import.
	""" 

	def __init__(self, debug: bool = True):
		"""
		Constructor for Jeremy.

		> debug: bool (True) - Enable / disable debug mode for more log content.
		""" 

		# Arm config variables
		self.ARM_FRONT = 177
		self.ARM_MIDDLE = 90
		self.ARM_BACK = 0

		self.last_update_time = 0
		self.last_known_position = None
		self.last_known_angle = None

		# Create variables
		self.R = sr.Robot()
		self.debug = debug
		screen.RUGGEDUINO = self.R.ruggeduino
		servo.RUGGEDUINO = self.R.ruggeduino

		# Biases
		# Callibration Settings:
		# Hard flooring: 1.075
		# fl, fr, bl, br
		self.wheel_bias_default = vector.Vector(1.075, 1, 1.075, 1)
		self.wheel_bias_default /= max(self.wheel_bias_default)

		self.wheel_bias_can = vector.Vector(1.1, 1, 1.1, 1)
		self.wheel_bias_can /= max(self.wheel_bias_can)

		# Initialize Display
		self.display = screen.Screen()

		# Initialize Servo
		self.grabberServo = servo.Servo(10, 55, 600, 2400, 0, 180, True)
		self.armServo = servo.Servo(9, 177, 500, 2500, 0, 250, True)
		self.grabberServo.setAngle(40)
		self.armServo.setAngle(180)

		# Populate markers
		self.worldView = world.World(vector.Vec3(5750, 5750))
		self.worldView.populateMarkers(28, 0.0)

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

		# Populate cans
		for canPos in self.canPositionsFloor:
			self.worldView.addCan(can.Can(canPos, False, 0.067))

		for canPos in self.canPositionsRaised:
			self.worldView.addCan(can.Can(canPos, True, 0.067))

	def async_position(self):
		return self.calculateWorldspacePosition()

	def drive_wheel(self, power: float, fb: str, lr: str):
		"""
		Drive a wheel with a power setting.

		> power: float - Power with which to drive. Sign specifies direction.

		> fb: str - Determines wheel to use: Front - ["front", "f"], Back -  ["back", "b"]

		> lr: str - Determines wheel to use: Left - ["left", "l"], Right -  ["right", "r"]
		""" 

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
			
		# Multiply the power by the corresponding bias
		hasCan = self.has_can()

		if motor == "fl": power *= self.wheel_bias_default.x if not hasCan else self.wheel_bias_can.x
		if motor == "fr": power *= self.wheel_bias_default.y if not hasCan else self.wheel_bias_can.y
		if motor == "bl": power *= self.wheel_bias_default.z if not hasCan else self.wheel_bias_can.z
		if motor == "br": power *= self.wheel_bias_default.w if not hasCan else self.wheel_bias_can.w

		self.log(f"Motor Power (Adjusted): {motor}, {power}, {self.wheel_bias}")

		try:
			self.R.motor_boards[WHEELS[motor][0]].motors[WHEELS[motor][1]].power = power
		except Exception as e:
			self.error(f"Error while driving wheel: {str(e)}")

	def drive(self, power: float):
		"""
		Drive all 4 wheels with the same power.

		> power: float - Power with which to drive. Sign specifies direction.
		""" 

		self.drive_wheel(power, "f", "l")
		self.drive_wheel(power, "f", "r")
		self.drive_wheel(power, "b", "l")
		self.drive_wheel(power, "b", "r")

	def turn(self, power: float):
		"""
		Drive all 4 wheels with opposite powers, in order to turn.

		> power: float - Power with which to drive. Sign specifies direction.
		""" 
		self.drive_wheel(power, "f", "l")
		self.drive_wheel(-power, "f", "r")
		self.drive_wheel(power, "b", "l")
		self.drive_wheel(-power, "b", "r")

	def stop(self):
		"""
		Stop driving, set all motor power to 0.
		""" 
		self.drive(0)

	def see(self):
		"""
		Find all markers visible.

		> return: sr.marker[] - a list of markers visible.
		""" 
		return self.R.camera.see()

	def see_ids(self):
		"""
		Find all marker ids visible.

		> return: int[] - a list of markers' ids visible.
		""" 
		return self.R.camera.see_ids()

	def save_image(self, name: str):
		"""
		Find all marker ids visible, and save the image.

		> name: str - the file name to save image as.

		> return: int[] - a list of markers' ids visible.
		""" 
		return self.R.camera.save(self.R.usbkey / name)

	def get_servo(self, servo: str):
		"""
		Get a servo object given a servo name.

		> servo: str - The servo to get: Grabber - ["g", "grab", "grabber", "grabberservo"], Arm - ["a", "arm", "armservo"]

		> return: servo - The servo object asked for.
		""" 
		if servo.lower() in ["g", "grab", "grabber", "grabberservo"]:
			return self.grabberServo
		elif servo.lower() in ["a", "arm", "armservo"]:
			return self.armServo
		else:
			self.warn("INVALID SERVO '{}'".format(servo))
			return None

	def set_angle(self, servo: str, angle: float):
		"""
		Sets the angle of a servo.

		> servo: str - The servo to set: Grabber - ["g", "grab", "grabber", "grabberservo"], Arm - ["a", "arm", "armservo"]

		> angle: float - The angle to set the servo to.
		""" 
		try:
			self.get_servo(servo).setAngle(angle)
		except Exception as e:
			self.error(f"Error while setting angle: {str(e)}")

	def get_angle(self, servo: str):
		"""
		Gets the angle of a servo.

		> servo: str - The servo to get: Grabber - ["g", "grab", "grabber", "grabberservo"], Arm - ["a", "arm", "armservo"]

		> return: float - The angle the servo is set to.
		""" 
		try:
			return self.get_servo(servo).getAngle()
		except Exception as e:
			self.error(f"Error while getting angle: {str(e)}")

	def attach(self, servo: str):
		"""
		Attaches the servo specified.

		> servo: str - The servo to attach: Grabber - ["g", "grab", "grabber", "grabberservo"], Arm - ["a", "arm", "armservo"]
		""" 
		self.get_servo(servo).attach()

	def detach(self, servo: str):
		"""
		Detaches the servo specified.

		> servo: str - The servo to detach: Grabber - ["g", "grab", "grabber", "grabberservo"], Arm - ["a", "arm", "armservo"]
		""" 
		self.get_servo(servo).detach()

	def set_display(self, c: str, l: int):
		"""
		Sets the character displayed on the screen.

		> c: str - The string to write to the LCD.
		
		> l: int - The line to write to [0, 1].
		""" 
		self.display.set(c, l)

	def has_can(self):
		"""
		Returns true if the sensors in the grabber form a complete circuit.
		"""
		ret = self.R.ruggeduino.command("#HASCAN#")
		return ret == "1"

	def set_grabber(self, o: bool):
		"""
		Opens the grabber arm.

		> o: bool - Open?
		"""
		self.grabberServo.setAngle(55 if o else 105)

	def set_arm(self, o: int):
		"""
		Moves the main arm.

		> o: int - Arm state.
		"""
		self.armServo.setAngle(o)

	def computeRelativePosition(self, marker1: marker.Marker, marker2: marker.Marker):
		"""
		Calculate's Jeremy's position based on two markers.

		> marker1: marker.Marker - The first marker to use.

		> marker2: marker.Marker - The second marker to use.

		> return: (vector.Vector, float) - The position and angle calculated.
		""" 
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
		"""
		Calculate's Jeremy's position with camera vision.

		> return: (vector.Vector, float) - The true position and angle calculated.
		""" 
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
		"""
		Jeremy does nothing now.
		
		> t: float - Jeremy does nothing for t seconds.
		""" 
		time.sleep(t)

	def log(self, msg: str):
		"""
		Outputs a message to the console if debug mode is enabled.
		
		> msg: str - Message to output.
		""" 
		if (self.debug):
			print("[ INFO ] " + str(msg))

	def warn(self, msg: str):
		"""
		Outputs a warning to the console if debug mode is enabled.
		
		> msg: str - Warning to output.
		""" 
		if (self.debug):
			print("[ WARN ] " + str(msg))

	def error(self, msg: str):
		"""
		Outputs an error to the console.
		
		> msg: str - Error to output.
		""" 
		print("[ ERROR ] " + str(msg))

