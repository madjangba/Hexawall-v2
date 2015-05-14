import RPi.GPIO as GPIO
import time
import math

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)
freqHz = 50
pwm=GPIO.PWM(7,freqHz)



def gaussian(x,mu,sig,a):
	return a*(math.exp(-math.pow((x-mu),2)/2*math.pow(sig,2)))



msPerCycle = 1000/freqHz

#for i in range(3):
#x = -7
x = -5
#while x <= 7:
while x <= 5:
	#g = gaussian(x,0,.2,2)
	#g = gaussian(x,0,.3,2)
	g = gaussian(x,0,.2,1.5)	
	dutyCyclePercentage = g * 100 / msPerCycle
	print "Position: " + str(g)
	print "Duty Cycle: " + str(dutyCyclePercentage) + "%"
	print ""
	pwm.start(dutyCyclePercentage)
	time.sleep(.0006)
	#time.sleep(0.002)
	x+=.06
	#x+=0.09
pwm.stop()
GPIO.cleanup()
