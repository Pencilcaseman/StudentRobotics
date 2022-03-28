from re import M
from sr.robot3 import *
from j5.backends.hardware.sr.v4 import servo_board
import time

WHEELS = {
	"fl" : ["SR0WAF", 0],
	"fr" : ["SR0GFJ", 1],
	"bl" : ["SR0WAF", 1],
	"br" : ["SR0GFJ", 0]
}

class Jeremy:
	def __init__(self):
		self.R = Robot()

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

	def servo(self, index: int, angle: float):
		char = chr(round(angle) / 2)
		self.R.ruggeduino.command(char)

	"""
	def direct_servo(self, index: int, val: int):
		servo = self.R.servo_boards["sr0RY2A"].servos[index]
		# servo._backend._positions[servo._identifier] = angle
		servo._backend._write(servo_board.CMD_WRITE_SET_SERVO[servo._identifier], val)

	def servo(self, index: int, angle: float):
		# This is the safe way of doing things, but it doesn't work :)
		# self.R.servo_boards["sr0RY2A"].servos[index].position = angle

		# So instead we fuck with Python, Student Robotics and everyone
		# else in order to set the servo to something other than -1 to 1
		# haehjfehehaekfehfhehefehfahdflkfalkhfhhehahrehfeahfheahdfljfat
		
		servo = self.R.servo_boards["sr0RY2A"].servos[index]
		# servo._backend.set_servo_position(servo._identifier, angle)
		servo._backend._positions[servo._identifier] = angle
		# value = round(angle * 100)
		value = round(angle * 2)
		servo._backend._write(servo_board.CMD_WRITE_SET_SERVO[servo._identifier], value)
	"""
