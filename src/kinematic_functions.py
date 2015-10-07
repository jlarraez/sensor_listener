'''
These functions provide forward and inverse kinematics solutions. 

Bryant Pong
RPI CS Robotics Lab
11/12/14

Last Updated: 3/2/15 - 4:23 PM
'''

from powerball_constants import *
import math
import numpy as np

''' 
This function takes in a numpy array representing a 3x1 vector and returns
a 3 x 3 matrix representing kcross. 

Arguments:
vector: a numpy array of the form: array([[x],
                                          [y],
										  [z]])

TESTED/VALIDATED 2/3/15 - 6:23 PM
'''
def kcross(vector):
	return np.matrix( ([0, -1 * vector[2][0], vector[1][0]], 
	                  [vector[2][0], 0, -1 * vector[0][0]],
					  [-1 * vector[1][0], vector[0][0], 0]), dtype = float)

'''
This function is an implementation of the Euler-Rodrigues formula for 3D
rotations.

Arguments:
rotAxis: The rotation axis (in the form of a numpy column array) 
angleToRotate: Angle to rotate (must be in radians)

TESTED/VALIDATED 2/3/15 - 6:28 PM
'''
def rot3D(rotAxis, angleToRotate):
	identity3 = np.matrix( ([1, 0, 0], [0, 1, 0], [0, 0, 1]), dtype=float )
	return identity3 + (math.sin(angleToRotate) * kcross(rotAxis)) + ( (1 - math.cos(angleToRotate)) * kcross(rotAxis) * kcross(rotAxis))

# Forward Kinematic Functions:

'''
Calculate the forward kinematics of the Powerball Arm (JOINTS ONLY).  The forward kinematics
is found by passing in a list of the joint angles.

Parameters:
jointAngles: A list of 6 floating point numbers representing the 6 joint angles (in radians)

Returns: A 4x4 homogenous transformation matrix containing the rotational and the
         translational forward kinematics for the ARM only.         
'''
def fkineArm(jointAngles):

	# Extract the joint angles from the list:
	q1 = jointAngles[0]
	q2 = jointAngles[1]
	q3 = jointAngles[2]
	q4 = jointAngles[3]	
	q5 = jointAngles[4]
	q6 = jointAngles[5]

	# The rotation matrices for the joints:
	R01 = rot3D(JOINT_1_ROTATION_AXIS, q1)
	R12 = rot3D(JOINT_2_ROTATION_AXIS, q2)
	R23 = rot3D(JOINT_3_ROTATION_AXIS, q3)
	R34 = rot3D(JOINT_4_ROTATION_AXIS, q4)
	R45 = rot3D(JOINT_5_ROTATION_AXIS, q5)
	R56 = rot3D(JOINT_6_ROTATION_AXIS, q6)

	R02 = R01 * R12
	R03 = R02 * R23
	R04 = R03 * R34
	R05 = R04 * R45
	R06 = R05 * R56

	# Calculate the translational forward kinematics:
	transFK = (R01 * P12) + (R02 * P23) + (R03 * P34) + (R04 * P45) + (R05 * P56)
	# Calculate the rotational forward kinematics:
	rotFK = R06 

	# Create a 4x4 Homogenous matrix that contains the forward and inverse kinematics of the core arm:
	return np.matrix([ [R06[0,0], R06[0,1], R06[0,2], transFK[0,0]], \
	                   [R06[1,0], R06[1,1], R06[1,2], transFK[1,0]], \
					   [R06[2,0], R06[2,1], R06[2,2], transFK[2,0]], \
					   [0, 0, 0, 1] ])

'''
This function calculates the forward kinematics for the end effector frame.  
This function requires 1 argument: the joint angle of the 6th joint.

Returns: 4x4 Homogenous Transformation matrix          
'''

# Inverse Kinematics Functions

'''
Subproblem 0: Find the angle between two vectors.

Inputs:
p: 1st Vector (numpy array of 3 floats)
q: 2nd Vector (numpy array of 3 floats)
k: Rotation Axis (numpy array of 3 floats)

TESTED/VALIDATED 2/5/15 - 4:31 PM
'''
def subproblem0(p, q, k):

	#print("inside sub0")

	# Normalize the vectors p and q:
	normP = p / np.linalg.norm(p)
	normQ = q / np.linalg.norm(q)

	# Calculate the angle between p and q:
	theta = 2 * math.atan2( (np.linalg.norm(normP - normQ)), (np.linalg.norm(normP + normQ)))

	# A Numpy Representation of the Rotation Axis:
	numpyMatrixK = np.matrix([k.item(0,0), k.item(1,0), k.item(2,0)])
	#print("numpyMatrixK: " + str(numpyMatrixK))
	#print("numpyMatrixK.shape: " + str(numpyMatrixK.shape))

	cross = np.cross(p.T, q.T)
	cross = np.matrix([cross.item(0), cross.item(1), cross.item(2)]).T
	#print("cross: " + str(cross))
	#print("cross.shape: " + str(cross.shape))


	# Determine the sign of the angle:
	temp = numpyMatrixK * cross 
	#print("temp: " + str(temp))
	
	# The crossproduct was negative; reverse the sign of theta:
	if temp.item(0,0) < 0.0:
		#print("before negating theta, theta is: " + str(theta))
		theta *= -1.0
		#print("after negating theta, theta is: " + str(theta))

	#print("leaving sub0")
	return theta

'''
subproblem1() finds the angle between two vectors in 3D Space.

Arguments:
P: 1st Vector (numpy array of 3 floats)
Q: 2nd Vector (numpy array of 3 floats)
K: Rotation Axis (numpy array of 3 floats)

TESTED/VALIDATED 2/5/15 - 4:39 PM
'''
def subproblem1(p, q, k):
		
	# Normalize all arguments:
	normP = p / np.linalg.norm(p)
	normQ = q / np.linalg.norm(q)
	normK = k / np.linalg.norm(k)

	#print("normP: " + str(normP))
	#print("normQ: " + str(normQ))
	#print("normK: " + str(normK))

	# Calculate the modified P and Q vectors:
	pPrime = np.subtract(normP, (np.dot(normP.T, normK) * normK))
	#print("pPrime: " + str(pPrime))
	qPrime = np.subtract(normQ, (np.dot(normQ.T, normK) * normK))
	#print("qPrime: " + str(qPrime))

	theta = subproblem0(pPrime, qPrime, k)

	return theta

'''
subproblem2() finds 0, 1, or 2 solutions between spinning two vectors
around two separate rotation axis.

Arguments:
p 
q 
k1 
k2

TESTED/VALIDATED 2/5/15 - 6:00 PM

'''
def subproblem2(p, q, k1, k2):

	#print("inside sub2: ")

	#print("p: " + str(p))
	#print("q: " + str(q))
	#print("k1: " + str(k1))
	#print("k2: " + str(k2))

	k12 = np.dot(k1.T, k2)[0][0]
	pk = np.dot(p.T, k2)[0][0]
	qk = np.dot(q.T, k1)[0][0]
	#print("k12: " + str(k12))
	#print("pk: " + str(pk))
	#print("qk: " + str(qk))


	# No solution exists:
	if abs(k12**2 - 1) < 0.001:
		print("No solution exists!")
		return [[-9001, -9001], [-9001, -9001]]
	
	a = np.matrix([ [k12, -1], [-1, k12] ]) * \
	    np.matrix([ [pk / (k12**2 - 1)], [qk / (k12**2 - 1)]])
	#print("a: " + str(a)) # PASSED

	bb = (np.linalg.norm(p)**2 - np.linalg.norm(a)**2 - \
	      2*a[0][0]*a[1][0]*k12).item(0, 0)
	#print("bb: " + str(bb)) # PASSED

	if abs(bb) < 0.0001:
		bb = 0
	if bb < 0:
		print("No solution exists!")
		return [[-9001, -9001], [-9001, -9001]]

	# Check if only 1 solution exists:
	gamma = math.sqrt(bb) / np.linalg.norm(np.cross(k1.T, k2.T))
	#print("gamma: " + str(gamma)) # PASSED

	if abs(gamma) < 0.0001:
		print("1 Solution Found!")
		k1xk2 = np.cross(k1.T, k2.T)  
		c1 = np.matrix(([k1.item(0,0), k2.item(0,0), k1xk2.item(0)],\
		                [k1.item(1,0), k2.item(1,0), k1xk2.item(1)],\
						[k1.item(2,0), k2.item(2,0), k1xk2.item(2)]\
						)) * np.matrix([ [a.item(0,0)],\
						                 [a.item(1,0)],\
										 [gamma] ]) 
		#print("c1: " + str(c1)) # PASSED
		#print("type c1: " + str(type(c1)))
		c1 = np.array([[c1.item(0,0), c1.item(1,0), c1.item(2,0)]]).T

		theta1 = -1*subproblem1(q, c1, k1)
		theta2 = subproblem1(p, c1, k2)
		return [[theta1, theta1], [theta2, theta2]]

	# General Case - 2 Solutions
	theta1 = np.zeros([1,2])
	theta2 = np.zeros([1,2])

	k1xk2 = np.cross(k1.T, k2.T)
	
	c1 = np.matrix(([k1.item(0,0), k2.item(0,0), k1xk2.item(0)],\
	                [k1.item(1,0), k2.item(1,0), k1xk2.item(1)],\
					[k1.item(2,0), k2.item(2,0), k1xk2.item(2)]\
				  )) * np.matrix([ [a.item(0,0)],\
				                   [a.item(1,0)],\
								   [gamma] ])
	c2 = np.matrix(([k1.item(0,0), k2.item(0,0), k1xk2.item(0)],\
	                [k1.item(1,0), k2.item(1,0), k1xk2.item(1)],\
					[k1.item(2,0), k2.item(2,0), k1xk2.item(2)]\
				  )) * np.matrix([ [a.item(0,0)],\
				                   [a.item(1,0)],\
								   [-1.0*gamma] ])

	c1 = np.array([[c1.item(0,0), c1.item(1,0), c1.item(2,0)]]).T
	c2 = np.array([[c2.item(0,0), c2.item(1,0), c2.item(2,0)]]).T

	#print("c1: " + str(c1));
	#print("c2: " + str(c2));

	theta2.itemset((0,0), subproblem1(p, c1, k2))
	theta2.itemset((0,1), subproblem1(p, c2, k2))
	theta1.itemset((0,0), -1*subproblem1(q, c1, k1))
	theta1.itemset((0,1), -1*subproblem1(q, c2, k1))

	#print("theta1: " + str(theta1))
	#print("theta2: " + str(theta2))
	#print("leaving sub2")
	return [theta1, theta2]

'''
This function creates a 4x4 Homogenous Transformation Matrix from Denavit-
Hartenberg parameters.  
'''
def homoTrans(th, d, a, al):
	return np.matrix([ [math.cos(th), -math.sin(th)*math.cos(al), \
	                    math.sin(th)*math.sin(al), a*math.cos(th)], \
					   [math.sin(th), math.cos(th)*math.cos(al), \
					    -math.cos(th)*math.sin(al), a*math.sin(th)], \
					   [0, math.sin(al), math.cos(al), d],
					   [0, 0, 0, 1] ])

'''
This function calculates the inverse kinematics for the Powerball arm.

Arguments:
T06 - a numpy.matrix containing the 4x4 homogenous transformation matrix.
thP - a Python list containing the current joint angles.   
'''

def ikine(T06, thP):
	
	# Create the matrix that will hold the Inverse Kinematics solution:
	thIK = np.zeros([7, 8])

	# This list holds the joint limits of the Powerball in radians:
	thLimits = [JOINT_1_MAX, JOINT_2_MAX, JOINT_3_MAX, JOINT_4_MAX, JOINT_5_MAX, JOINT_6_MAX]	 

	# Denavit-Hartenberg Parameters for the Powerball: d1 with the normal platform 205. In our simulation joint_0 is over the robotfoot
	#d1 = 205 
	d1 = 89.7
	a2 = 350
	d4 = 305
	# with Gripper d6 = 300
	#d6 = 300
	#d6 = 93 from cad
	d6 = 75
	# Solve for theta 3 (Joint 3 angle): 	 

	# Vector from the spherical wrist to the tooltip:
	dx = np.matrix([ [T06[0, 0], T06[0, 1], T06[0, 2] ], \
	                 [T06[1, 0], T06[1, 1], T06[1, 2] ], \
					 [T06[2, 0], T06[2, 1], T06[2, 2] ] ]) * \
					 np.matrix([0, 0, d6]).T
	dx = np.array([[dx[0, 0], dx[1, 0], dx[2, 0]]]).T
	#print("dx: " + str(dx)) # TEST PASSED

	# Vector with the tool tip distance and base distance removed:
	temp1 = np.subtract(np.array([[T06[0, 3], T06[1, 3], T06[2, 3]]]).T, dx,)
	dElbow = np.subtract(temp1, np.array([[0, 0, d1]]).T)
	#print("dElbow is: " + str(dElbow)) # TEST PASSED 2/6/15 - 2:29 PM

	dElbowNorm = np.linalg.norm(dElbow)	
	#print("dElbowNorm is: " + str(dElbowNorm)) # TEST PASSED 2/6/15 - 2:33 PM

	# Angle of Elbow (found by Law of Cosines)
	temp = math.pi - math.acos( ( (a2**2)+(d4**2)-(dElbowNorm**2))/(2*a2*d4))
	#print("temp is: " + str(temp)) # TEST PASSED 2/6/15 - 2:33 PM

	# 8 Solutions - first 4 rows are for elbow up; bottom 4 are elbow down:
	thIK[2][0] = thIK[2][1] = thIK[2][2] = thIK[2][3] = temp
	thIK[6][0] = thIK[6][0] + 2**1
	thIK[6][1] = thIK[6][1] + 2**1
	thIK[6][2] = thIK[6][2] + 2**1
	thIK[6][3] = thIK[6][3] + 2**1
	thIK[2][4] = thIK[2][5] = thIK[2][6] = thIK[2][7] = -temp
	#print("thIK: " + str(thIK)) # TEST PASSED 2/6/15 - 2:39 PM

	# Solve for Joints 1 and 2:

	p1pt1 = np.array([[0, 0, a2]]).T
	p1pt2 = np.array([[-d4*math.sin(thIK.item(2,0)), 0, d4*math.cos(thIK.item(2,0))]]).T
	p1 = np.add(p1pt1, p1pt2)
	#print("p1pt1: " + str(p1pt1)) # TEST PASSED 2/6/15 - 3:02 PM
	#print("p1pt2: " + str(p1pt2)) # TEST PASSED 2/6/15 - 3:02 PM
	#print("p1: " + str(p1)) # TEST PASSED 2/6/15 - 3:02 PM

	p2pt1 = np.array([[0, 0, a2]]).T
	p2pt2 = np.array([[-d4*math.sin(thIK.item(2,4)), 0, d4*math.cos(thIK.item(2,4))]]).T
	p2 = np.add(p2pt1, p2pt2)
	#print("p2pt1: " + str(p2pt1)) # THREE LINES PASSED
	#print("p2pt2: " + str(p2pt2))
	#print("p2: " + str(p2))

	# Call subproblem 2 to solve for Joint angles 1 and 2:
	elbowUpSolution = subproblem2(p1, dElbow, JOINT_1_ROTATION_AXIS,\
	                              JOINT_2_ROTATION_AXIS)
	elbowDownSolution = subproblem2(p2, dElbow, \
	                                JOINT_1_ROTATION_AXIS, JOINT_2_ROTATION_AXIS)


	#print("elbowUpSolution: " + str(elbowUpSolution))
	#print("elbowDownSolution: " + str(elbowDownSolution))

	# Replace invalid solutions from subproblem2:
	theta11 = elbowUpSolution[0]
	theta21 = elbowUpSolution[1]
	theta12 = elbowDownSolution[0]
	theta22 = elbowDownSolution[1]
	
	#print("theta11: " + str(theta11))
	#print("theta21: " + str(theta21))
	#print("theta12: " + str(theta12))
	#print("theta22: " + str(theta22))

	#print("theta11 dim: " + str(theta11.shape))
	#print("theta21 dim: " + str(theta21.shape))
	#print("theta12 dim: " + str(theta12.shape))
	#print("theta22 dim: " + str(theta22.shape))


	#print("theta11.item(0,0) is: " + str(theta11.item(0,0)))
	#print("theta11.item(0,1) is: " + str(theta11.item(0,1)))

	# Check for any invalid values:

	'''
	if np.isnan(theta11[0,0]) or np.isnan(theta11[0,1]):
		print("A value in theta11 is nan!")
		theta11 = [thP[0,0], thP[0,0]]
	if np.isnan(theta12[0,0]) or np.isnan(theta12[0,1]):
		print("A value in theta12 is nan!")
		theta12 = [thP[0,0], thP[0,0]]
	if np.isnan(theta21[0,0]) or np.isnan(theta21[0,1]):
		print("A value in theta21 is nan!")
		theta21 = [thP[0,0], thP[0,0]]
	if np.isnan(theta22[0,0]) or np.isnan(theta22[0,1]):
		print("A value in theta22 is nan!")
		theta22 = [thP[0,0], thP[0,0]]
	'''
	#print("thP is: " + str(thP))
	if np.any(np.isnan(theta11)):
		theta11 = np.array([[thP[0], thP[0]]])
		theta21 = np.array([[theta21[0], theta21[1]]])
	if np.any(np.isnan(theta21)):
		theta21 = np.array([[thP[0], thP[0]]])
		theta11 = np.array([[theta11[0], theta11[1]]])
	if np.any(np.isnan(theta12)):
		theta12 = np.array([[thP[0], thP[0]]])
		theta22 = np.array([[theta22[0], theta22[1]]])
	if np.any(np.isnan(theta22)):
		theta22 = np.array([[thP[0], thP[0]]])
		theta12 = np.array([[theta12[0], theta12[1]]])

	#print(thIK[0, 0:4])

	#print("After setting theta11, theta21, theta12, theta22:") 
	#print("theta11: " + str(theta11))
	#print("theta21: " + str(theta21))
	#print("theta12: " + str(theta12))
	#print("theta22: " + str(theta22))

	# ALL TESTS PASSED TO THIS POINT
		
	# Set the solutions for shoulder right:
	thIK[0, 0:4] = [theta11[0,0], theta11[0,1], theta11[0,0], theta11[0,1]]
	thIK[6, [0,2]] = thIK[6, [0,2]] + 2**0
	thIK[0, 4:8] = [theta12[0,0], theta12[0,1], theta12[0,0], theta12[0,1]]	
	thIK[6, [4,6]] = thIK[6, [4,6]] + 2**0
	thIK[1, 0:4] = [theta21[0,0], theta21[0,1], theta21[0,0], theta21[0,1]]
	thIK[1, 4:8] = [theta22[0,0], theta22[0,1], theta22[0,0], theta22[0,1]]	 

	#print("After setting the solutions for shoulder right and shoulder left")
	#print("thIK is: ") 
	#print(thIK)

	# ALL TESTS PASSED TO THIS POINT

	# Solve for Joint 4, 5, 6 Angles:
	for z in [0, 1, 4, 5]:
		th1 = thIK[0, z]
		th2 = thIK[1, z]
		th3 = thIK[2, z]			

		#print("th1: " + str(th1))
		#print("th2: " + str(th2))
		#print("th3: " + str(th3))

		# Joint 1 Transformation Matrix:
		T01 = homoTrans(th1, d1, 0, -math.pi / 2)
		# Joint 2 Transformation Matrix:
		T12 = homoTrans(th2 - math.pi / 2, 0, a2, math.pi) 
		# Joint 3 Transformation Matrix:
		T23 = homoTrans(th3 - math.pi / 2, 0, 0, -math.pi / 2)	 

		T02 = T01*T12
		T03 = T02*T23

		#print("T01: " + str(T01))
		#print("T12: " + str(T12))
		#print("T23: " + str(T23))
		#print("T02: " + str(T02))
		#print("T03: " + str(T03))

		# ALL TESTS PASSED TO THIS POINT

		t03Rot = T03[0:3, 0:3].T
		#print("t03Rot:")
		#print(t03Rot)
		thirdRow = -T03[0:3, 0:3].T*T03[0:3, 3]
		#print("thirdRow:")
		#print(thirdRow)
	


		Twrist = np.matrix([ [t03Rot[0,0], t03Rot[0,1], t03Rot[0,2], \
		                      thirdRow[0,0]], \
							 [t03Rot[1,0], t03Rot[1,1], t03Rot[1,2], \
							  thirdRow[1,0]], \
							 [t03Rot[2,0], t03Rot[2,1], t03Rot[2,2], \
							  thirdRow[2,0]], \
							 [0, 0, 0, 1] ]) * T06
	
		#print("Twrist: " + str(Twrist))

		# Remove first 3 joint angles to isolate joints 4, 5, 6 angles:
		thIK[3,z] = math.atan2(-Twrist[1,2], -Twrist[0,2])
		thIK[6,z] = thIK[6,z]+2**2
		# Wrist up:
		thIK[4,z] = math.acos(Twrist[2,2])
		thIK[5,z] = math.atan2(-Twrist[2,1], Twrist[2,0])
		thIK[3,z+2] = math.atan2(Twrist[1,2], Twrist[0,2])
		# Wrist down:
		thIK[4,z+2] = -math.acos(Twrist[2,2])
		thIK[5,z+2] = math.atan2(Twrist[2,1], -Twrist[2,0])

		#print("Final Twrist:")
		#print(Twrist)

	#print("After solving for joints 4, 5, 6, thIK is: ")
	#print(thIK) 

	# ALL TESTS PASS TO THIS POINT

	# Finally, take into account any joint limits and find closest solutions:
	counter = 0		
	tempOut = np.zeros([7,8])

	for x in range(8):
		# Check the joint limits:

		#print("thIK[0:6, x] is: " + str(thIK[0:6, x]))

		# The number of joints that are within joint limits:
		absTHIK = np.abs(thIK[0:6, x])  
		numGoodJoints = 0   
		nextJoint = 0
		for i in absTHIK:
			#print("i is: " + str(i))
			#print("thLimits[nextJoint] is: " + str(thLimits[nextJoint]))
			if i <= thLimits[nextJoint]:
				numGoodJoints += 1
			nextJoint += 1

		if numGoodJoints == 6:
			tempOut[0:7, counter] = thIK[0:7, x]
			counter += 1
		#if sum(np.abs(thIK[0:6, x]) <= thLimits) == 6:
			#tempOut[0:7, counter] = thIK[0:7, x]
			#counter += 1

	#print("tempOut: ")
	#print(tempOut)
	print("STEP 1")
	tempOutLim = tempOut[0:7, 0:counter]

	#print("tempOutLim.shape[0]: " + str(tempOutLim.shape[0]))
	#print("tempOutLim.shape[1]: " + str(tempOutLim.shape[1]))
	if tempOutLim.shape[1] != 0:

		#print("tempOutLim is: ")
		#print(tempOutLim)

		# ALL TESTS PASS TO THIS POINT

		#print("np.abs(thP) is: " + str(np.abs(thP)))

		#if max(np.abs(thP)) > 0:
		minDiff = np.zeros([counter, 1])	
		for x in range(counter):
			minDiff[x] = np.linalg.norm(tempOutLim[0:6, x].T - thP)
		#print("minDiff:" + str(minDiff))
		smallestIndex = 9001
		if minDiff.item(0, 0) < minDiff.item(1, 0):
			smallestIndex = 0
		else:	 
			smallestIndex = 1	 
		thOut = []	 
		for i in range(7): 
			thOut.append(tempOutLim.item(i, smallestIndex))	 
		return thOut	 	  
			#minDiff = np.zeros([counter, 1])
			#for x in range(counter):
				#minDiff[x] = np.linalg.norm(tempOutLim[0:6, x].T - thP)

			#print("minDiff:" + str(minDiff))

			# ALL TESTS PASS TO THIS POINT
		
			# Find the closest solution:
			#smallestIndex = 9001
		
			#if minDiff.item(0, 0) < minDiff.item(1, 0):
				#smallestIndex = 0
			#else:
				#smallestIndex = 1

			#print("smallestIndex: " + str(smallestIndex))
			#print("STEP 2")		
			#thOut = []
			#for i in range(7):
				#thOut.append(tempOutLim.item(i, smallestIndex))
			#print("STEP 3")
			#return thOut
		#else:
			#thOut = tempOutLim
			#print("STEP 4")
			#print thOut
			#return thOut
	else:
		print("STEP 5")
		return []
