"""
Main Arm Servo:
177 deg = Front
0 deg = Back

Grabber Arm Servo:
55 deg = Grab can
105 deg = Open

Starting Top Left:
	Can Order:
		[5, 8, 4, 12, 14, 15, 13]
	Buffer Point:
		(1900, 1900)

Starting Top Right:
	Can Order:
		[6, 9, 4, 13, 16, 17, 12]
	Buffer Point:
		(3850, 1900)

Starting Bottom Left:
	Can Order:
		[7, 10, 5, 15, 18, 19, 14]
	Buffer Point:
		(1900, 3850)

Starting Bottom Right:
	Can Order:
		[6, 11, 7, 19, 17, 16, 18]
	Buffer Point:
		Buffer Point(3850, 3850)

"""

import sr.robot3 as sr
import math, time
import vector, servo, marker, can, world, screen

RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180

TOP_LEFT_CORNER = 0
TOP_RIGHT_CORNER = 1
BOTTOM_LEFT_CORNER = 3
BOTTOM_RIGHT_CORNER = 2

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

	def __init__(self, debug: bool = True, bs: int = 100, dp: float = 0.4, tp: float = 0.3):
		"""
		Constructor for Jeremy.
, 
		> debug: bool (True) - Enable / disable debug mode for more log content.
		""" 

		# Arm config variables
		self.ARM_FRONT = 177
		self.ARM_MIDDLE = 90
		self.ARM_BACK = 0

		# Direction variables
		self.FORWARD = 0
		self.BACKWARD = 1
		self.RIGHT = 2
		self.LEFT = 3
		self.UNKNOWN = -1

		self.direction = None
		self.corner = None
		self.drive_power = dp # Straignt line power for Jeremy (must remain constant?)
		self.turn_power = tp # Turning power for Jeremy (must remain constant?)
		self.last_update_time = 0 # Time since last camera update
		self.last_known_position = None # Last *KNOWN* position
		self.last_known_angle = None # Last *KNOWN* angle
		self.estimated_velocity = None # Estimated velocity at given power level
		self.estimated_angular_velocity = None # Estimated ang. vel. at given power level
		self.estimated_position = None # Estimated position of jeremy based on known and appriximate movements
		self.estimated_angle = None # Estimated angle of jeremy based on known and appriximate rotations

		self.position_buffer = []
		self.angle_buffer = []
		self.buffer_size = bs

		# Create variables
		self.R = sr.Robot()
		self.debug = debug
		screen.RUGGEDUINO = self.R.ruggeduino
		servo.RUGGEDUINO = self.R.ruggeduino

		# Biases
		# Callibration Settings:
		# Hard flooring: 1.075
		# LEH Carpet: 1.148
		# fl, fr, bl, br
		self.wheel_bias_default = vector.Vector(1.1, 1, 1.1, 1) # vector.Vector(1.148, 1, 1.148, 1) # vector.Vector(1.075, 1, 1.075, 1)
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
			vector.Vec3(2875.000, 33.500, 0000.000), #  0
			vector.Vec3(33.500, 2875.000, 0000.000), #  1
			vector.Vec3(5716.500, 2875.000, 0000.000), #  2
			vector.Vec3(2875.000, 5716.500, 0000.000), #  3
			vector.Vec3(2875.000, 1135.000, 0000.000), #  4
			vector.Vec3(1135.000, 2875.000, 0000.000), #  5
			vector.Vec3(4615.000, 2875.000, 0000.000), #  6
			vector.Vec3(2875.000, 4615.000, 0000.000), #  7
			vector.Vec3(1625.000, 1625.000, 0000.000), #  8
			vector.Vec3(4125.000, 1625.000, 0000.000), #  9
			vector.Vec3(1625.000, 4125.000, 0000.000), # 10
			vector.Vec3(4125.000, 4125.000, 0000.000), # 11
			vector.Vec3(2475.000, 1875.000, 0000.000), # 12
			vector.Vec3(3275.000, 1875.000, 0000.000), # 13
			vector.Vec3(1875.000, 2475.000, 0000.000), # 14
			vector.Vec3(1875.000, 3275.000, 0000.000), # 15
			vector.Vec3(3875.000, 2475.000, 0000.000), # 16
			vector.Vec3(3875.000, 3275.000, 0000.000), # 17
			vector.Vec3(2475.000, 3875.000, 0000.000), # 18
			vector.Vec3(3275.000, 3875.000, 0000.000)  # 19
		)

		# TODO: Change these?
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

	def lrCalibration__TEST(self):
		self.set_grabber(True)
		self.sleep(1)
		self.set_arm(self.ARM_FRONT)
		self.sleep(1)
		self.set_display("Jeremy says hi", 0)
		self.drive(0.4)
		self.sleep(1)
		self.stop()
		self.sleep(2)
		self.set_display("Yummy can...", 1)
		self.set_grabber(False)
		self.sleep(0.85)
		self.set_arm(self.ARM_MIDDLE)
		for val in [1.1475, 1.148, 1.49, 1.15]:
			wb = vector.Vec4(val, 1, val, 1)
			wb /= max(wb)
			self.wheel_bias_default = wb
			self.sleep(10)
			self.set_display(f"Offset: {val}", 0)
			self.set_display(f"{wb.x:.1f},{wb.y:.1f},{wb.z:.1f},{wb.w:.1f}", 1)
			self.drive(self.drive_power)
			self.sleep(10)
			self.drive(0)

	def drive_wheel(self, power: float, fb: str, lr: str):
		"""
		Drive a wheel with a power setting.

		> power: float - Power with which to drive. Sign specifies direction.

		> fb: str - Determines wheel to use: Front - ["front", "f"], Back -  ["back", "b"]

		> lr: str - Determines wheel to use: Left - ["left", "l"], Right -  ["right", "r"]
		""" 

		self.direction = self.UNKNOWN

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

		if power == 0: self.direction = None
		if power > 0: self.direction = self.FORWARD
		if power < 0: self.direction = self.BACKWARD

	def turn(self, power: float):
		"""
		Drive all 4 wheels with opposite powers, in order to turn.

		> power: float - Power with which to drive. Sign specifies direction.
		"""

		self.drive_wheel(power, "f", "l")
		self.drive_wheel(-power, "f", "r")
		self.drive_wheel(power, "b", "l")
		self.drive_wheel(-power, "b", "r")

		if power == 0: self.direction = None
		if power > 0: self.direction = self.RIGHT
		if power < 0: self.direction = self.LEFT

	def stop(self):
		"""
		Stop driving, set all motor power to 0.
		"""

		if self.direction is None or self.direction == self.UNKNOWN:
			self.drive(0)
		elif self.direction == self.FORWARD:
			self.drive(-0.2)
		elif self.direction == self.BACKWARD:
			self.drive(0.2)
		elif self.direction == self.RIGHT:
			self.turn(-0.2)
		elif self.direction == self.LEFT:
			self.turn(0.2)

		self.sleep(0.02)
		self.drive(0)
		self.sleep(0.1)
		self.direction = None

		return 

	def see(self):
		"""
		Find all markers visible.

		> return: sr.marker[] - a list of markers visible.
		""" 

		self.stop()
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

	def set_display(self, c: str, l: int = 0):
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

		# Offset the marker coordinates
		# (markers are 200mm squares)
		M1.x += 100 * math.cos(marker1.orientation.pitch)
		M1.y += 100 * math.sin(marker1.orientation.pitch)
		M2.x += 100 * math.cos(marker2.orientation.pitch)
		M2.y += 100 * math.sin(marker2.orientation.pitch)

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

	def calculateWorldspacePosition(self, reattempt: bool = False, retNone: bool = False, updatePos: bool = True):
		"""
		Calculate's Jeremy's position with camera vision.

		> return: (vector.Vector, float) - The true position and angle (deg) calculated.
		""" 

		a = self.time()
		markers = sorted(self.see(), key= lambda s: s.id, reverse=False)

		# self.save_image(str(self.time()) + ".png")

		b = self.time()
		self.log(f"Time taken: {str(b - a)}")
		for seen in markers:
			self.log(f"[ VISIBLE ] Cartesian: {seen.cartesian} | ID: {seen.id}")

		if len(markers) < 2:
			# self.save_image("{}.png".format(round(time.time(), 3)))
			
			if reattempt:
				self.log("Saw fewer than 2 markers. Attempting to take another photo")
				self.sleep(0.5)
				return self.calculateWorldspacePosition(False, retNone, updatePos)
			
			if not retNone:
				self.log("Saw fewer than 2 markers. Falling back to estimated position")
				return self.estimated_position, self.estimated_angle
			
			self.log("Saw fewer than 2 markers. Returning None")
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

		subAngle = math.pi - theta
		truePos.x += 110 * math.sin(subAngle) + 160 * math.cos(subAngle)
		truePos.y -= 110 * math.cos(subAngle) + 160 * math.sin(subAngle)

		if updatePos:
			self.estimated_position = truePos
			self.estimated_angle = trueAngle

		return truePos, trueAngle

	def initial_calibration(self):
		"""
		Calibrate the estimated velocity and angular velocity of Jeremy
		at a particular power level.

		Requirements:
		 - Positive power represents a clockwise turn
		 - Power level remains constant
		 - 0 deg => Facing right
			- -90 deg => Facing up
		 - Robot does not rotate more than 360 degrees
		"""

		# ===========
		#    TURN
		# ===========

		dt = 0.5 # Sleep for longer???

		_, theta0 = self.calculateWorldspacePosition(True)
		self.turn(self.turn_power)
		self.sleep(dt)
		self.stop()
		s1, theta1 = self.calculateWorldspacePosition(True)

		dtheta = theta1 - theta0
		if dtheta < 0: dtheta += 2 * math.pi # Ensure 0 < dtheta < 360

		self.estimated_angular_velocity = dtheta / (dt - 0.1)
		self.angle_buffer.append((dtheta, (dt - 0.1)))

		# Determine which corner we are in
		if s1.x < 5.75 / 2 and s1.y < 5.75 / 2: self.corner = TOP_LEFT_CORNER
		if s1.x < 5.75 / 2 and s1.y > 5.75 / 2: self.corner = BOTTOM_LEFT_CORNER
		if s1.x > 5.75 / 2 and s1.y < 5.75 / 2: self.corner = TOP_RIGHT_CORNER
		if s1.x > 5.75 / 2 and s1.y > 5.75 / 2: self.corner = BOTTOM_RIGHT_CORNER

		# Make sure to look directly into the arena
		# self.turnTo(vector.Vec2(5750 / 2, 5750 / 2))

		# if self.zone() == TOP_LEFT_CORNER: self.setApproximateAngle((7 / 18) * math.pi) # 70 degrees
		# if self.zone() == TOP_RIGHT_CORNER: self.setApproximateAngle((8 / 9) * math.pi) # 160 degrees
		# if self.zone() == BOTTOM_RIGHT_CORNER: self.setApproximateAngle((-11 / 18) * math.pi) # -110 degrees
		# if self.zone() == BOTTOM_LEFT_CORNER: self.setApproximateAngle((-1 / 9) * math.pi) # -20 degrees

		# ===========
		#    MOVE
		# ===========

		dt = 1.5

		self.drive(self.drive_power)
		self.sleep(dt)
		self.stop()
		s2, theta2 = self.calculateWorldspacePosition(True)

		# Update records
		self.last_update_time = self.time()
		self.last_known_position = s2
		self.last_known_angle = theta2

		self.estimated_position = s2
		self.estimated_angle = theta2

		ds = s2 - s1
		self.estimated_velocity = ds.mag() / (dt - 0.1)
		self.position_buffer.append((ds.mag(), (dt - 0.1)))

		self.log("CALIBRATED: Velocity: {}   Ang Velocity: {}".format(self.estimated_velocity, self.estimated_angular_velocity))

		if self.estimated_velocity == 0:
			self.log("Estimated velocity was zero. Setting default value")
			self.estimated_angular_velocity = 400
		if self.estimated_angular_velocity == 0:
			self.log("Estimated angular velocity was zero. Resetting to defualt")
			self.estimated_angular_velocity = 0.8

	def doCalibrateThing(self):
		"""
		Tom. Again. Sex please
		"""

		# Always make sure we have $self.buffer_size elements
		while len(self.position_buffer) > self.buffer_size:
			self.position_buffer.pop(0)

		while len(self.angle_buffer) > self.buffer_size:
			self.angle_buffer.pop(0)

		vel = []
		rot = []

		for i in range(len(self.position_buffer)):
			t = self.position_buffer[i][1]
			s = self.position_buffer[i][0]
			if (t < 1e-5): continue
			vel.append(abs(s) / t)

		for i in range(len(self.angle_buffer)):
			t = self.angle_buffer[i][1]
			a = self.angle_buffer[i][0]
			if (t < 1e-5): continue
			rot.append(abs(a) / t)

		velAdjusted = []
		rotAdjusted = []
		mult = 1.5
		for val in vel:
			if abs(val) < abs((sum(vel) / len(vel)) * mult):
				velAdjusted.append(val)

		for val in rot:
			if abs(val) < abs((sum(rot) / len(rot)) * mult):
				rotAdjusted.append(val)

		if len(velAdjusted) > 0: self.estimated_velocity = sum(velAdjusted) / len(velAdjusted)
		if len(rotAdjusted) > 0: self.estimated_angular_velocity = sum(rotAdjusted) / len(rotAdjusted)

		self.log(f"Estimated Velocity: {self.estimated_velocity}")
		self.log(f"Estimated Ang Vel.: {self.estimated_angular_velocity}")
		self.log(f"Choo Chooo Fuckers: {self.position_buffer}")
		self.log(f"Choooo Chooo Cunts: {self.angle_buffer}")

	def setApproximateAngle(self, angle: float):
		"""
		Set the approximate angle of the robot based on estimated
		angular velcity calculations and the previous known angle
		"""

		while angle < -math.pi: angle += math.pi * 2
		while angle > math.pi: angle -= math.pi * 2

		dtheta = angle - self.estimated_angle
		while dtheta < -math.pi: dtheta += math.pi * 2
		while dtheta > math.pi: dtheta -= math.pi * 2

		if dtheta > 0: power = self.turn_power
		else: power = -self.turn_power
		t = abs(dtheta) / self.estimated_angular_velocity

		self.turn(power)
		self.sleep(t)
		self.stop()

		# Continue calibrating if needed
		truePos, trueAngle = self.calculateWorldspacePosition(True, True, False)
		if truePos is None:
			self.estimated_angle = angle
			return

		trueDTheta = self.estimated_angle - trueAngle
		while trueDTheta < -math.pi: trueDTheta += math.pi * 2
		while trueDTheta > math.pi: trueDTheta -= math.pi * 2

		self.estimated_angle = trueAngle
		self.estimated_position = truePos

		# If too small an angle, don't include it
		if trueDTheta > 5 * DEG_TO_RAD:
			self.angle_buffer.append((trueDTheta, (t - 0.1)))
		
		self.doCalibrateThing()

	def driveApproximateDist(self, dist: float):
		"""
		Set the ... tom do this...
		"""

		t = abs(dist) / self.estimated_velocity

		self.drive(self.drive_power * (1 if dist > 0 else -1))
		self.sleep(t)
		self.stop()

		a = self.time()
		# Continue calibrating if needed
		truePos, trueAngle = self.calculateWorldspacePosition(True, True, False)
		if truePos is None:
			self.estimated_position.x += dist * math.cos(self.estimated_angle)
			self.estimated_position.y += dist * math.sin(self.estimated_angle)
			return

		moved = (self.estimated_position - truePos).mag()
		if moved > 250:
			self.position_buffer.append((moved, (t - 0.1)))

		self.estimated_angle = trueAngle
		self.estimated_position = truePos

		self.doCalibrateThing()
		b = self.time()
		self.log(f"MEGA Time taken: {str(b - a)}")

	def turnTo(self, coord: vector.Vector):
		"""
		Tom do things
		"""

		delta = coord - self.estimated_position
		theta = math.atan2(delta.y, delta.x) # math.atan(delta.y / delta.x)
		
		self.setApproximateAngle(theta)

	def driveTo(self, coord: vector.Vector, steps: int = 1):
		"""
		Tom help.
		"""

		self.log("Driving to {}".format(coord))

		for i in range(steps):
			delta = coord - self.estimated_position
			self.turnTo(coord)
			self.driveApproximateDist(delta.mag() * ((i + 1) / steps))

	def pickUp(self, attempts: int = 1, drive: float = 200):
		"""
		Picks up a can.

		> attempts: int (1) - the number of attempts to do for a can.

		> drive: float (300)- the amount to drive forward inbetween each attempt.

		> return: boolean - if the can has been picked up.
		"""

		self.set_angle("g", 55)
		self.sleep(0.2)
		self.set_angle("a", 177)
		self.sleep(0.3)
		self.set_angle("g", 105)
		self.sleep(0.2)

		good = self.has_can()
		if not good:
			self.sleep(0.1)
		good = self.has_can()

		if good:
			self.set_angle("a", 90)
			return True
		else:
			self.set_angle("g", 55)
			self.sleep(0.2)
			if attempts <= 1:
				self.driveApproximateDist(-drive * 3)
				return False
			self.driveApproximateDist(drive)
			return self.pickUp(attempts - 1, drive)

	def drop(self):
		if self.has_can():
			self.setApproximateAngle(self.estimated_angle + math.pi)
			self.set_angle("a", 0)
			self.sleep(0.5)
			self.set_angle("g", 55)
			self.sleep(0.5)
			self.set_angle("a", 177)
			self.sleep(0.5)
			self.driveApproximateDist(250)

	def sleep(self, t: float):
		"""
		Jeremy does nothing now.

		Note: This uses a spin timer so is highly inefficient from a
		processor point of view
		
		> t: float - Jeremy does nothing for t seconds.
		""" 

		start = time.perf_counter()
		while time.perf_counter() - start < t:
			pass

	def time(self):
		"""
		Return a high-precision time in seconds -- more precise than time.time()
		"""

		return time.perf_counter()

	def zone(self):
		if self.R.mode == "dev":
			return 0
		return self.R.zone

	def canPosition(self, id: int):
		return self.worldView.cans[id].cartesian

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
