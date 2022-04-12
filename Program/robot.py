from jeremy import Jeremy
import math, time

jezza = Jeremy(True) # Set to false during actual competition.

# Arm Test
# jezza.set_angle("a", 180)
# jezza.set_angle("g", 40)
# jezza.sleep(5)
# jezza.set_angle("g", 0)
# jezza.sleep(2)
# jezza.set_angle("a", 0)
# jezza.sleep(5)
# jezza.set_angle("g", 40)

# Rotation Test
# for i in range(3):
# 	start = time.time()
# 	jezza.log(jezza.get_rot())
# 	end = time.time()
# 	jezza.log(end - start)
# 	jezza.sleep(2)

# # Rotation Test 2
# jezza.get_rot()
# jezza.sleep(1)
# jezza.get_rot()
# jezza.drive(0.3)
# jezza.sleep(1)
# jezza.get_rot()
# jezza.drive(0)
# jezza.sleep(1)
# jezza.get_rot()

# for i in range(10):
# 	jezza.turn_angle(90, 0.2)
# 	jezza.sleep(2)

while True:
	jezza.drive(0.3)
	jezza.sleep(3)
	jezza.drive(0.0)
	jezza.sleep(3)
