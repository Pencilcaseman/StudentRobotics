from jeremy import Jeremy
import math
import time

jezza = Jeremy()

# jezza.save_image("porn.png")

# for marker in jezza.find_markers():
# 	print(marker, " | [ CARTESIAN ]", marker.cartesian)

jezza.drive_wheel(0.1, "front", "left")
time.sleep(1)
jezza.drive_wheel(0.1, "front", "right")
time.sleep(1)
jezza.drive_wheel(0.1, "back", "left")
time.sleep(1)
jezza.drive_wheel(0.1, "back", "right")
time.sleep(1)


start = time.time()
while time.time() - start < 30:

	angle = ((math.sin(time.time()) + 1) / 2) * 180
	jezza.set_angle("grabber", angle)
	jezza.set_angle("arm", angle)


"""

marker_ids = jezza.save_image("marker_null.png")
time.sleep(3)
jezza.drive(0.3)
start = time.time()
tick = 0
while time.time() - start < 5:
	marker_ids = jezza.save_image("movie_{}.png".format(tick))
	tick += 1
	# time.sleep(0.05)

"""
