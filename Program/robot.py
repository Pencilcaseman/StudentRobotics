from jeremy import Jeremy
import time

jezza = Jeremy(True) # Set to false during actual competition.

for i in range(3):
	start = time.time()
	jezza.log(jezza.get_rot())
	end = time.time()
	jezza.log(end - start)
	jezza.sleep(2)
