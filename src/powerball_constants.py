#!/usr/bin/python

'''
powerball_constants.py - This file contains length and joint constants
for the Powerball Arm.
	
Bryant Pong
RPI CS Robotics Lab
11/12/14

Last Updated: 3/2/14 - 3:02 PM 
'''

import math
import numpy as np

'''
Length constants.  These constants can be found on Page 2 of the file:
"powerball_datasheet".

All these constants are in centimeters (cm).

The reference frame 0 is the base of the robot.
''' 

L1 = 205.0
L2 = 350.0
L3 = 305.0
LT = 300.0 

P01 = np.array([[0.0, 0.0, L1]]).T
P12 = np.array([[0.0, 0.0, 0.0]]).T
P23 = np.array([[0.0, 0.0, L2]]).T
P34 = np.array([[0.0, 0.0, 0.0]]).T
P45 = np.array([[0.0, 0.0, L3]]).T
P56 = np.array([[0.0, 0.0, 0.0]]).T
P6T = np.array([[0.0, 0.0, LT]]).T

'''
Frames.  There are 4 frames that we are concerned about:

1) World Frame (currently centered at the center of the base)  
2) Robot Base Frame (currently centered at the center of the base)
3) Arm Frame (currently centered in the center of the bottom joint).  This is 
   calculated by the forward kinematics - fkineArm() function.
4) End-Effector Frame (currently centered at the top half of the last joint).  
   The end-effector frame is calculated by the function calcEndEffectorFrame(). 
'''
WORLDFRAME = np.matrix([ [1, 0, 0, 0], \
                         [0, 1, 0, 0], \
						 [0, 0, 1, 0], \
						 [0, 0, 0, 1] ]) 

BASEFRAME = np.matrix([ [1, 0, 0, 0], \
                        [0, 1, 0, 0], \
						[0, 0, 1, 0], \
						[0, 0, 0, 1] ])

'''
Joint limits.  These limits are in radians.  Joint 1 is the base of the 
Powerball; Joint 6 is the end-effector wrist.
'''
DEGS_TO_RADS = math.pi / 180.0

JOINT_1_MIN = -170.0 * DEGS_TO_RADS
JOINT_1_MAX = 170.0 * DEGS_TO_RADS

JOINT_2_MIN = -110.0 * DEGS_TO_RADS
JOINT_2_MAX = 110.0 * DEGS_TO_RADS

JOINT_3_MIN = -155.0 * DEGS_TO_RADS
JOINT_3_MAX = 155.0 * DEGS_TO_RADS

JOINT_4_MIN = -170.0 * DEGS_TO_RADS
JOINT_4_MAX = 170.0 * DEGS_TO_RADS

JOINT_5_MIN = -140.0 * DEGS_TO_RADS
JOINT_5_MAX = 140.0 * DEGS_TO_RADS

JOINT_6_MIN = -170.0 * DEGS_TO_RADS
JOINT_6_MAX = 170.0 * DEGS_TO_RADS

'''
Rotation axes.  These axes follow the following convention:

(z)
^    (y)
|   /
|  / 
| / 
|/
----------->(x)
'''

JOINT_1_ROTATION_AXIS = np.array([[0, 0, 1]]).T
JOINT_2_ROTATION_AXIS = np.array([[0, 1, 0]]).T
JOINT_3_ROTATION_AXIS = np.array([[0,-1, 0]]).T
JOINT_4_ROTATION_AXIS = np.array([[0, 0, 1]]).T
JOINT_5_ROTATION_AXIS = np.array([[0,-1, 0]]).T
JOINT_6_ROTATION_AXIS = np.array([[0, 0, 1]]).T
   

