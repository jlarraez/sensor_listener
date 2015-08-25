#!/usr/bin/python

# Standard Python Libraries:
import sys

# ROS Libraries:
import rospy
from schunk_api.srv import *

def schunk_api_client(joint1, joint2, joint3, joint4, joint5, joint6):
	rospy.wait_for_service("PositionAPIJointSpace")
	try:
		schunk_api_call = rospy.ServiceProxy("PositionAPIJointSpace", PositionAPIJointSpace)
		response = schunk_api_call(joint1, joint2, joint3, joint4, joint5, joint6)
		return 0
	except rospy.ServiceException, e:
		print("Service call failed: %s") % e

if __name__ == "__main__":	
	# Expecting 6 arguments:
	if len(sys.argv) == 7:
		joint1 = float(sys.argv[1])
		joint2 = float(sys.argv[2])
		joint3 = float(sys.argv[3])
		joint4 = float(sys.argv[4])
		joint5 = float(sys.argv[5])
		joint6 = float(sys.argv[6])
	else:
		# Print usage information:
		print("Usage: <joint1> <joint2> <joint3> <joint4> <joint5> <joint6>")
		sys.exit(1)

	print("Now requesting Powerball Arm to Move:")
	schunk_api_client(joint1, joint2, joint3, joint4, joint5, joint6)
