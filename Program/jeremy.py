from sr.robot3 import *
import servo

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
