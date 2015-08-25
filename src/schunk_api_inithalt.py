#!/usr/bin/python

'''
This script shows how to initialize, halt, and emergency stop the Powerball
through the schunk API. 

Bryant Pong
RPI CS Robotics Laboratory
2/26/15

Last Updated: 4/6/15 - 12:46 PM
'''

# Python Imports:
import rospy
import sys
from schunk_api.srv import *

'''
Service handler for initHalt service. 
'''
def initHaltClient(usrInput):
	rospy.wait_for_service('InitHaltAPI')

	# Send the command 
	try:
		initHaltAPI = rospy.ServiceProxy('InitHaltAPI', InitHaltAPI)
		resp = initHaltAPI(usrInput)
		return resp.status
	except rospy.ServiceException, e:
		print("Service call failed: %s" % e)	

'''
Main function:

We need 1 argument:
cmd: a STRING argument that is either 'init', 'halt', or 'estop' 
'''
def main():
		
	usrIn = str(raw_input("Please enter your next command (q to quit):"))

	while True:
		
		# Call the initHalt service with the input:
		initHaltClient(usrIn)
		
		usrIn = str(raw_input("Please enter your next command(q to quit):")) 

		if usrIn == 'q':
			break

		print("Now requesting Powerball Arm to: " + str(usrIn))        
	
	print("Goodbye!")

if __name__ == '__main__':
	main()
