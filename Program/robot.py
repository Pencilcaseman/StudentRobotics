from jeremy import Jeremy
import time

jezza = Jeremy(True) # Set to false during actual competition.

for i in range(3):
	jezza.drive(0)
	start = time.time()
	print(jezza.calculateWorldspacePosition())
	end = time.time()
	print(end - start)
	time.sleep(2)
