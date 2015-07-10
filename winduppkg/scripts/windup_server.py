#!/usr/bin/python

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_StepperMotor, Adafruit_DCMotor
import time
import RPi.GPIO as GPIO
import atexit
import sys
import rospy
from std_srvs.srv import Empty

mh = Adafruit_MotorHAT(addr = 0x60)
stepper = mh.getStepper(200, 1) # 200 steps/rev, motor port #1

#Button on GPIO pin #18
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN,pull_up_down=GPIO.PUD_UP)

def bye():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	sys.exit()

atexit.register(bye)

def tug(length):
	time.sleep(1.5)
	for i in range (0,3):
		for i in range(0,length):
			stepper.oneStep(Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE)
			time.sleep(0.01)
		for i in range(0,length):
			stepper.oneStep(Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE)
			time.sleep(0.01)
	bye()
	
def handle_windup(request):
	while True:
		if GPIO.input(18) == False:
			tug(15)
		else:
			stepper.oneStep(Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE) # Direction, Type
			time.sleep(0.01) # Seconds between steps (speed, lower is faster)

def windup_server():
	rospy.init_node('windup_server')
	rospy.loginfo("Windup server online!!(Hooray!)")
	s = rospy.Service('windup', Empty, handle_windup)
	rospy.spin()

if __name__ == "__main__":
	windup_server()
