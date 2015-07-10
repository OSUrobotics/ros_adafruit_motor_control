#!/usr/bin/env python
from std_srvs.srv import Empty
import sys
import rospy

def windup(windup_srv):
	try:
		windup_srv()
	except:
		rospy.logerr("Service call failure for windup!")

if __name__ == "__main__":
	rospy.init_node("windup_client")
	rospy.loginfo("Windup_client node online!")
	rospy.wait_for_service('windup')
	windup_srv = rospy.ServiceProxy('windup', Empty)
	
	windup(windup_srv)	#call this to windup.
