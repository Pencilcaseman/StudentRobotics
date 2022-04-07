from jeremy import Jeremy
import math
import time

jezza = Jeremy()

# jezza.save_image("porn.png")

# for marker in jezza.find_markers():
# 	print(marker, " | [ CARTESIAN ]", marker.cartesian)

start = time.time()
while time.time() - start < 10:
	jezza.drive(0)
	print(jezza.calculateWorldspacePosition())
	jezza.drive(0.3)
	time.sleep(2)


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
