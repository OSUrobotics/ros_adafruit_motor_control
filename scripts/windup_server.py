#!/usr/bin/python

#Setup
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_StepperMotor, Adafruit_DCMotor
import time
import RPi.GPIO as GPIO
import atexit
import sys
import rospy
from std_srvs.srv import Empty, EmptyResponse

mh = Adafruit_MotorHAT(addr = 0x60)
stepper = mh.getStepper(200, 1) # 200 steps/rev, motor port #1
stepper2 = mh.getStepper(200,2)

#Button on GPIO pin #18, Experimental button on GPIO pin #24 (clicked on motor rotation)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(24, GPIO.IN,pull_up_down=GPIO.PUD_UP)

def bye():	#To shut off all motors
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

atexit.register(bye)

#We tested a function to release and tug the object a couple times, in order to avoid it getting stuck. Results were mixed, didn't show promise
#def tug(length):
#	time.sleep(1.5)
#	for i in range (0,3):
#		for i in range(0,length):
#			stepper.oneStep(Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE)
#			time.sleep(0.01)
#		for i in range(0,length):
#			stepper.oneStep(Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE)
#			time.sleep(0.01)
#	bye()

# To unwind motors 3 steps each
def unwind():
	for i in range(0,2):
		stepper.oneStep(Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE) # Direction, Type
        	stepper2.oneStep(Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE)
        	time.sleep(0.0025) # Seconds between steps (speed, lower is faster)


# main function	
def handle_windup(request):
	past_GPIO = 1
	past_time = 0
	rospy.loginfo("Windup request received!")	#Notification that server has received message from client to windup
	count = 0
	stuck_bool = False
	while True:
		current_GPIO = GPIO.input(24)
		rospy.loginfo(GPIO.input(24))
		current_time = rospy.get_time() / 1000000000
		if GPIO.input(18) == False:	#If reset button is clicked, object is reset, Shutdown all motors
			rospy.loginfo("Success")
			bye()
			break
#			tug(15)
		else:	#Otherwise continue 
			
			#	print count	
			if count > 1500:	#Activated if motor rotation switch is not clicked for 1500 cycles. Motors unwinded 3 steps, and shut off. 
				rospy.loginfo("STUCK!!!")
				unwind()
				bye()					
				stuck_bool = True
				break
			elif current_GPIO != past_GPIO:		# If the motor rotation button changes status, motor rotation continues, and count resets.
				rospy.loginfo("elif")
				count = 0
				past_GPIO = current_GPIO
				past_time = current_time
				stepper.oneStep(Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE) # Direction, Type
				stepper2.oneStep(Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE)
				time.sleep(0.0025) # Seconds between steps (speed, lower is faster)
			else:	#If motor rotation button is the same as it was last cycle the count is increased, and motors rotate.
				count += 1
				rospy.loginfo("else")
			stepper.oneStep(Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE) # Direction, Type
                        stepper2.oneStep(Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE)
                        time.sleep(0.0025) # Seconds between steps (speed, lower is faster)

		
	return stuck_bool

def windup_server():	#Sets up up node and service
	rospy.init_node('windup_server')
	rospy.loginfo("Windup server online!!(Hooray!)")
	s = rospy.Service('windup', Empty, handle_windup)
	rospy.spin()

if __name__ == "__main__":
	windup_server()
