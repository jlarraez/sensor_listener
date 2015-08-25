#!/usr/bin/python

'''
schunk_api_test_coord.py - Sample Program to demonstrate how to send a target
Cartesian location and target end-effector orientation to the Powerball.

This method sends 7 constants to the Powerball.

Bryant Pong
RPI CS Robotics Laboratory
4/22/15

Last Updated: 4/22/15 - 5:41 PM    
'''

# Standard Python Libraries:
import sys

# ROS Libraries:
import rospy
from schunk_api.srv import *

def schunk_api_client(x, y, z, qx, qy, qz, qw):
	rospy.wait_for_service("PositionAPICoordSpace")
	try:
		schunk_api_call = rospy.ServiceProxy("PositionAPICoordSpace", PositionAPICoordSpace)
		response = schunk_api_call(x, y, z, qx, qy, qz, qw)
		return 0
	except rospy.ServiceException, e:
		print("Service call failed: %s") % e

if __name__ == "__main__":	
	# Expecting 3 arguments:
	if len(sys.argv) == 8:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		z = float(sys.argv[3])

		quatX = float(sys.argv[4])
		quatY = float(sys.argv[5])
		quatZ = float(sys.argv[6])
		quatW = float(sys.argv[7])
	else:
		# Print usage information:
		print("Usage: <x> <y> <z> <quat x> <quat y> <quat z> <quat w>")
		sys.exit(1)

	print("Now requesting Powerball Arm to Move:")
	schunk_api_client(x,y,z, quatX, quatY, quatZ, quatW)
