#!/usr/bin/env python

# Standard Python Libraries:
import time
import numpy as np

# ROS Libraries:
import rospy
import roslib
import actionlib
import simple_script_server

# Standard ROS Messages:
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from tf.transformations import *
from control_msgs.msg import *
from sensor_msgs.msg import JointState

# Custom ROS Messages:
from sensor_listener.srv import *
from std_srvs.srv import Trigger

# Fraunhofer Libraries:
from cob_sound.msg import *
from cob_script_server.msg import *
#from cob_srvs.srv import *

# Manifests to load:
roslib.load_manifest('cob_script_server')

# Custom Kinematic Libraries:
import kinematic_functions as kf
from powerball_constants import *

def add_two_ints_client(req):
    print "inside handler"
    rospy.wait_for_service('/arm/driver/init')
    try:
        print "inside try 1"
        add_two_ints = rospy.ServiceProxy('/arm/driver/init',Trigger)
        resp1 = add_two_ints()
        print "inside try 2"
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def api_server():

	# Initialize the API Server node:
	rospy.init_node('test_node')
	# This service accepts API calls to initialize, halt, and emergency stop the Powerball:
	InitHaltAPIsrv = rospy.Service('InitHaltAPI', InitHaltAPI, add_two_ints_client)	   
	rospy.loginfo("InitHaltAPI Handle Up!")	
	rospy.spin() 


if __name__ == "__main__":
    api_server()
