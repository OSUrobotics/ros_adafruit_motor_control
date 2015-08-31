#!/usr/bin/python
import RPi.GPIO as GPIO
import rospy

GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)
rospy.init_node("swag")

while True:

	if GPIO.input(24) == 1:
		rospy.loginfo(1)
	else:
		rospy.loginfo(0)
	
