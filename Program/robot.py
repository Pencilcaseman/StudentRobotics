from jeremy import Jeremy

jezza = Jeremy()

jezza.drive(0.4)
jezza.sleep(8)

"""
jezza.set_angle("g", 105)
jezza.sleep(1)
jezza.set_angle("a", 177)
jezza.sleep(1)

" ""
while True:
	jezza.set_angle("g", 55)
	jezza.sleep(5)
	jezza.set_angle("g", 105)
	jezza.sleep(0.5)
	jezza.set_display("I HAVE A CAN" if jezza.has_can() else "FUCK", 0)
	jezza.sleep(1)
" ""

jezza.set_display("Jeremy says hi", 0)
jezza.drive(0.4)
jezza.sleep(1)
jezza.stop()
jezza.set_display("Put can in mouth", 1)
jezza.sleep(1)

jezza.set_angle("g", 55)
jezza.sleep(5)
jezza.set_angle("g", 105)
jezza.sleep(0.5)
jezza.set_display("I HAVE A CAN" if jezza.has_can() else "FUCK", 0)
jezza.sleep(1)

jezza.set_display("Deepthroating...", 1)
jezza.set_angle("g", 105)
jezza.sleep(0.85)
jezza.set_angle("a", 90)
jezza.drive(-0.4)
jezza.sleep(3)
jezza.stop()
jezza.set_display("Shitting...", 1)
jezza.set_angle("a", 0)
jezza.sleep(0.5)
jezza.set_angle("g", 55)
jezza.sleep(0.5)
jezza.drive(0.4)
jezza.set_angle("a", 177)
jezza.sleep(0.5)
jezza.stop()
jezza.set_display("mmm", 1)
"""
