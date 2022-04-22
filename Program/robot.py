from jeremy import Jeremy
from vector import Vec2

# JEZZASPAWN
jezza = Jeremy()

# jezza.lrCalibration__TEST()

# Create Variables
cans = [5, 8, 4, 12, 14, 15, 13]
buffer_point = Vec2(1900, 1900)
scoring_zone = (Vec2(0, 0), Vec2(0, 0))

# Calibration
jezza.initial_calibration()

jezza.driveTo(Vec2(1000, 1000), 4)
jezza.pickUp(3, 300)
jezza.driveTo(Vec2(3000, 3000), 3)
jezza.drop()

jezza.driveTo(Vec2(1000, 4750), 4)
jezza.pickUp(3, 300)
jezza.driveTo(Vec2(3000, 3000), 3)
jezza.drop()

"""
# Create game loop
drive_to_buffer = [False, False, False, False, True, False, True]
zone = jezza.zone()
if zone == 0:
	cans = [5, 8, 4, 12, 14, 15, 13]
	buffer_point = Vec2(1900, 1900)
	scoring_zone = (Vec2(260, 2500 - 500), Vec2(2500 - 500, 260))
elif zone == 1:
	cans = [4, 9, 6, 16, 13, 12, 17]
	buffer_point = Vec2(3850, 1900)
	scoring_zone = (Vec2(5750 - 2500 + 500, 260), Vec2(5750 - 260, 2500 - 500))
elif zone == 2:
	cans = [6, 11, 7, 19, 17, 16, 18]
	buffer_point = Vec2(3850, 3850)
	scoring_zone = (Vec2(5750 - 260, 5750 - 2500 + 500), Vec2(5750 - 2500 + 260, 5750 - 250))
elif zone == 3:
	cans = [7, 10, 5, 15, 18, 19, 14]
	buffer_point = Vec2(1900, 3850)
	scoring_zone = (Vec2(2500 - 500, 260), Vec2(260, 5750 - 2500 + 500))
lerp_ratio = (scoring_zone[1] - scoring_zone[0]) / len(cans)

# Execute gameloop
for i in range(len(cans)):
	jezza.driveTo(jezza.canPosition(cans[i]), 4)
	if jezza.pickUp(3, 250):
		jezza.driveTo(scoring_zone[0] + i * lerp_ratio, 3)
		jezza.sleep(0.5)
		jezza.drop()
	else:
		if drive_to_buffer[i]:
			jezza.driveTo(buffer_point, 3)
			jezza.sleep(0.5)
"""

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