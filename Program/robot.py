from jeremy import Jeremy
jezza = Jeremy()
import vector

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
jezza.set_display("mmm", 1)
"""

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

jezza.initial_calibration()
jezza.sleep(2)
jezza.setApproximateAngle(0)

