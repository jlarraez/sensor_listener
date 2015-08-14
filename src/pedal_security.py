#!/usr/bin/python

'''
This script initialize the pedal_security_node that will be constantly reading the state of the pedal and will stop the program if the presh the right pedal. It based on the work of Bryant Pong and the RPI CS Robotics Laboratory. 

Last Updated: 13/8/15 - 14:21 PM
'''

# Python Imports:
import rospy
import sys
import roslib
import actionlib
import simple_script_server


from schunk_api.srv import *
# Fraunhofer Libraries:
from cob_sound.msg import *
from cob_script_server.msg import *
from cob_srvs.srv import *
from std_srvs.srv import *

# Manifests to load:
roslib.load_manifest('cob_script_server')

'''
Service handler for initHalt service. 
'''
def initHaltClient(usrInput):
	rospy.wait_for_service('/arm/driver/halt')

	# Send the command 
	try:
		initHaltAPI = rospy.ServiceProxy('/arm/driver/halt', Trigger)
		resp = initHaltAPI()
		return resp.message	  
	except rospy.ServiceException, e:
		print("Service call failed: %s" % e)	

'''
Main function:

We need 1 argument:
cmd: a STRING argument that is either 'init', 'halt', or 'estop' 
'''
def main():
		
	while True:
		usrIn = str(raw_input("Presh right pedal to quit"))
		if usrIn == '2':
			# Call the initHalt service with the input:
		  usrIn ="halt"
		  initHaltClient(usrIn)
		print("Now requesting Powerball Arm Driver to stop")   
		if usrIn == '1':
			# Call the initHalt service with the input:
		  usrIn ="init"
		  #initHaltClient(usrIn)
		print("Now requesting Powerball Arm Driver to initialise")        
	
	print("Goodbye!")

if __name__ == '__main__':
	main()
