from jeremy import Jeremy
import math
import vector

jezza = Jeremy()
jezza.buffer_size = 5

# jezza.set_grabber(True)
# jezza.sleep(1)
# jezza.set_arm(jezza.ARM_FRONT)
# jezza.sleep(1)
# jezza.set_display("Jeremy says hi", 0)
# jezza.drive(0.4)
# jezza.sleep(1)
# jezza.stop()
# jezza.sleep(2)
# jezza.set_display("Deepthroating...", 1)
# jezza.set_grabber(False)
# jezza.sleep(0.85)
# jezza.set_arm(jezza.ARM_MIDDLE)
# for val in [1.01, 1.02, 1.03, 1.04, 1.05, 1.06, 1.07, 1.08, 1.09, 1.10]:
# 	wb = vector.Vec4(val, 1, val, 1)
# 	wb /= max(wb)
# 	jezza.wheel_bias = wb
# 	jezza.sleep(10)
# 	jezza.set_display(f"Offset: {val}", 0)
# 	jezza.set_display(f"{wb.x:.1f},{wb.y:.1f},{wb.z:.1f},{wb.w:.1f}", 1)
# 	jezza.drive(0.4)
# 	jezza.sleep(10)
# 	jezza.drive(0)

"""
jezza.set_grabber(True)
jezza.sleep(1)
jezza.set_arm(jezza.ARM_FRONT)
jezza.sleep(1)
jezza.set_display("Jeremy says hi", 0)
jezza.drive(0.4)
jezza.sleep(1)
jezza.stop()
jezza.sleep(2)
jezza.set_display("Deepthroating...", 1)
jezza.set_grabber(False)
jezza.sleep(0.85)
jezza.set_arm(jezza.ARM_MIDDLE)
jezza.drive(0.4)
jezza.sleep(5)
jezza.stop()
jezza.set_display("Shitting...", 1)
jezza.set_arm(jezza.ARM_FRONT)
jezza.sleep(0.5)
jezza.set_grabber(True)
jezza.sleep(0.5)
jezza.drive(-0.4)
jezza.set_arm(jezza.ARM_BACK)
jezza.sleep(0.5)
jezza.stop()
"""

"""
while True:
	pos, angle = jezza.calculateWorldspacePosition2()
	if pos is None or angle is None: continue

	posStr = f"{pos.x:.2f} {pos.y:.2f}"
	angleStr = angle # f"{angle:.2f}"
	jezza.set_display(posStr, 0)
	jezza.set_display(angleStr, 1)
	jezza.sleep(1)
"""

"""
while True:
	markers = jezza.see()
	if len(markers) == 0: continue

	idStr = f"ID: {markers[0].id}"
	angleStr = f"{markers[0].orientation.pitch * (180 / 3.1415926):.2f}"
	# angleStr = f"{markers[0].spherical.rot_x * (180 / 3.1415926):.2f} {markers[0].spherical.rot_y * (180 / 3.1415926):.2f}"
	jezza.set_display(idStr, 0)
	jezza.set_display(angleStr, 1)
	jezza.sleep(1)
"""


jezza.initial_calibration()
jezza.sleep(2)
jezza.setApproximateAngle(0)
jezza.sleep(2)
jezza.setApproximateAngle(math.pi / 2)
jezza.sleep(2)
jezza.driveTo(vector.Vec2(1800, 1140), 4)
# jezza.lookAt(vector.Vec2(500, 500))
# jezza.sleep(2)


"""
while True:
	pos, angle = jezza.calculateWorldspacePosition()
	if pos is None or angle is None: continue

	posStr = f"{pos.x:.2f} {pos.y:.2f}"
	angleStr = f"{angle:.2f}"
	jezza.set_display(posStr, 0)
	jezza.set_display(angleStr, 1)
	jezza.sleep(1)
"""
