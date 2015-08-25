#!/usr/bin/python

'''
This node provides Position Control API access to the Schunk Powerball Arm.

Bryant Pong
RPI CS Robotics Lab
10/17/14

Last Updated: 4/24/15 - 4:46 PM
'''

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
from std_msgs.msg import String, ColorRGBA
from control_msgs.msg import *
from sensor_msgs.msg import JointState

# Custom ROS Messages:
from sensor_listener.srv import *

# Fraunhofer Libraries:
from cob_sound.msg import *
from cob_script_server.msg import *
from cob_srvs.srv import *

# Manifests to load:
roslib.load_manifest('cob_script_server')

# Custom Kinematic Libraries:
import kinematic_functions as kf
from powerball_constants import *

'''
Callback for the leapmotion listener


'''
def leapmotioncallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

'''
This function sends a position control request to the Powerball.  The request
message consists of 6 float64 parameters representing the target angles for
the joints.  This API Function works in JOINT SPACE.
'''  
def position_api_joint_space_handler(req):
	rospy.loginfo("Inside  PositionAPIJointSpace Service call!")
	'''
	This simple_script_server is a custom library that was created by 
	the Fraunhofer institute.  An action_handle will listen for position
	commands.
	'''
	ah = simple_script_server.action_handle("move", "arm", "home", False, False)
	if False:
		return ah
	else:
		ah.set_active()

	# Form a list of the target joint angles:
	targetJointAngles = [req.joint1, req.joint2, req.joint3, req.joint4, req.joint5, req.joint6]

	'''
    Encapsulate the target jointangles into another list (this forms a trajectory)
	'''
	traj = [targetJointAngles]

	# Generate the trajectory message to send to the Powerball:
	traj_msg = JointTrajectory()
	traj_msg.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
	traj_msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint']
	point_nr = 0

	# Set the target velocities of the target joints.  They are set to 0 to denote stopping at the destinations:
	for point in traj:
		point_nr += 1
		point_msg = JointTrajectoryPoint()
		point_msg.positions = point
		point_msg.velocities = [0] * 6
		point_msg.time_from_start = rospy.Duration(3 * point_nr)
		traj_msg.points.append(point_msg)

	# Send the position control message to the action server node:
	action_server_name = '/arm_controller/follow_joint_trajectory'

	client = actionlib.SimpleActionClient(action_server_name, FollowJointTrajectoryAction)
	if not client.wait_for_server(rospy.Duration(5)):
		print("Action server not ready within timeout.  Aborting...")
		ah.set_failed(4)
		return ah
	else:
		print("Action server ready!")

	client_goal = FollowJointTrajectoryGoal()
	client_goal.trajectory = traj_msg
	client.send_goal(client_goal)
	ah.set_client(client)

	ah.wait_inside()
	# return ah
	return 0

jointAngles = [-9001.0, -9001.0, -9001.0, -9001.0, -9001.0, -9001.0]
jointStateCallbackEx = False

'''
This callback assigns the new joint positions to variables j1...j6.
It will also toggle the "hasNewMessage" flag to True. 
'''
def jointStateCallback(data):

	global jointAngles
	global jointStateCallbackEx
	global obj1
	obj1=data
	#joint_msg_leap.name=list(obj1.name)
	#joint_msg_leap.position=list(obj1.position)
	#joint_msg_leap.velocity=list(obj1.velocity)
	#joint_msg_leap.effort=list(obj1.effort)
	#print("I received jointStates!")
	#print("data: " + str(data))
	#joint_msg_leap=[]
	#name=list(data.name)
	#position=list(data.position)
	#velocity=list(data.velocity)
	#effort=list(data.effort)
	#obj1.position=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]		
	j1 = data.position[0]
	j2 = data.position[1]
	j3 = data.position[2]
	j4 = data.position[3]
	j5 = data.position[4]
	j6 = data.position[5]
	jointAngles = [j1, j2, j3, j4, j5, j6]

	jointStateCallbackEx = True

'''
This function handles a position command given in the coordinate space.  This 
function expects a message (defined in msg/PositionCoordSpace.msg) in the form 
of an (X, Y, Z) tuple, where X/Y/Z are floating point numbers indicating the X, Y, Z
destinations of the end effector (in mm.)
'''
def position_api_coord_space_handler(req):
	rospy.loginfo("Inside  PositionAPICoordSpace Service call!")		
	global jointStateCallbackEx

	'''
	This simple_script_server is a custom library that was created by
	the Fraunhofer institute.  An action_handle will listen for position commands.
	'''
	ah = simple_script_server.action_handle("move", "arm", "home", False, False)
	if False:
		return ah
	else:
		ah.set_active()
	
	# Get the target (X, Y, Z) coordinates to move to:
	targetCoords = [req.xCoord, req.yCoord, req.zCoord]

	# Get the desired rotation (in quaternion) to move to:
	targetRot = [req.quatW, req.quatX, req.quatY, req.quatZ]

	'''
	Currently the Powerball requires a list of the 6 target joint angles to move.
	We can calculate these target joint angles by calling the inverse kinematics
	functions:
	'''

	'''
	First, get a list of the current joint angles.  The joint angles can be
	found from rostopic /joint_states 
	'''
	sub = rospy.Subscriber("/joint_states", JointState, jointStateCallback) 

	while jointStateCallbackEx == False:
		pass

	rospy.Subscriber.unregister(sub)
	jointStateCallbackEx = False

	# We need to convert the quaternion into a 4x4 homogeneous transformation matrix:
	homoMat = quaternion_matrix(targetRot) 
	
	# Insert the desired target joint coordinate into the transformation matrix:
	homoMat[0,3] = req.xCoord
	homoMat[1,3] = req.yCoord
	homoMat[2,3] = req.zCoord    

	'''
	Calculate the inverse kinematics given the target rotation/position and
	the list of current joint angles:
	'''
	print(homoMat)
	print(jointAngles)
	targetJointAngles = kf.ikine(homoMat, jointAngles)
	print("targetJointAngles")
	if len(targetJointAngles) != 0:
		# We have a valid solution!  Move the Powerball to this location:
	
		targetJointAngles = targetJointAngles[:6]

		# Encapsulate the targetJointAngles into a trajectory:
		traj = [targetJointAngles] 

		# Generate the trajectory message to send to the Powerball:
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
		traj_msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint']
		point_nr = 0

		# Set the target velocities of the target joints.  They are set to 0 to denote stopping at the destinations:
		for point in traj:
			point_nr += 1
			point_msg = JointTrajectoryPoint()
			point_msg.positions = point
			point_msg.velocities = [0] * 6
			point_msg.time_from_start = rospy.Duration(3 * point_nr)
			traj_msg.points.append(point_msg)

		# Send the position control message to the action server node:
		action_server_name = '/arm_controller/follow_joint_trajectory'
		
		client = actionlib.SimpleActionClient(action_server_name, FollowJointTrajectoryAction)
		if not client.wait_for_server(rospy.Duration(5)):
			print("Action server not ready within timeout.  Aborting...")
			ah.set_failed(4)
			return ah
		else:
			print("Action server ready for Coordinate API Request")
		
		client_goal = FollowJointTrajectoryGoal()
		client_goal.trajectory = traj_msg
		client.send_goal(client_goal)
		ah.set_client(client)

		ah.wait_inside()
	return 0

'''
This handler allows a user to send a geometry_msgs/Pose message and calls the inverse kinematics
to move the Powerball to a target location.    
'''
def position_api_coord_space_quat_handler(req):
	rospy.loginfo("Inside PostionAPICoordSpaceQuat Service call!")	
	global jointStateCallbackEx
	#global joint_msg_leap
	#joint_msg_leap.name=['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint','base_joint_gripper_left','base_joint_gripper_right']
	#joint_msg_leap.position=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	#joint_msg_leap.velocity=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	#joint_msg_leap.effort=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

	#rospy.loginfo("Inside server_callback!")
	'''
	This simple_script_server is a custom library that was created by
	the Fraunhofer institute.  An action_handle will listen for position commands.
	'''
	ah = simple_script_server.action_handle("move", "arm", "home", False, False)
	if False:
		rospy.loginfo("Action server ready!")
		return ah
	else:
		ah.set_active()
		#rospy.loginfo("ah active!")	
	# Get the target (X, Y, Z) coordinates to move to:
	targetCoords = [req.target.position.x, req.target.position.y, req.target.position.z]
	#rospy.loginfo("TargetCoords read!")
	# Get the desired rotation (in quaternion) to move to:
	targetRot = [req.target.orientation.w, req.target.orientation.x, req.target.orientation.y, req.target.orientation.z]
	#rospy.loginfo("TargetRot read!")
	'''
	Currently the Powerball requires a list of the 6 target joint angles to move.
	We can calculate these target joint angles by calling the inverse kinematics
	functions:
	'''

	'''
	First, get a list of the current joint angles.  The joint angles can be
	found from rostopic /joint_states 
	'''
	sub = rospy.Subscriber("/joint_states", JointState, jointStateCallback) 
	pub = rospy.Publisher("joint_leap", JointState, queue_size=10) 
	while jointStateCallbackEx == False:
		pass

	rospy.Subscriber.unregister(sub)
	jointStateCallbackEx = False

	# We need to convert the quaternion into a 4x4 homogeneous transformation matrix:
	homoMat = quaternion_matrix(targetRot) 
	
	# Insert the desired target joint coordinate into the transformation matrix:
	homoMat[0,0] = 1
	homoMat[0,1] = 0
	homoMat[0,2] = 0
	homoMat[1,0] = 0
	homoMat[1,1] = 1
	homoMat[1,2] = 0
	homoMat[2,0] = 0
	homoMat[2,1] = 0
	homoMat[2,2] = 1
	homoMat[0,3] = req.target.position.x
	homoMat[1,3] = req.target.position.y
	homoMat[2,3] = req.target.position.z
	#print(homoMat)
	'''
	Calculate the inverse kinematics given the target rotation/position and
	the list of current joint angles:
	'''
	print(homoMat)
	print(jointAngles)
	try:
		targetJointAngles = kf.ikine(homoMat, jointAngles)
	except ValueError:
		print("ERROR: Position out of Range")
		return 0		
	if len(targetJointAngles) != 0:
		# We have a valid solution!  Move the Powerball to this location: 
		#targetJointAngles = targetJointAngles.flatten()
		print(targetJointAngles)		
		# Encapsulate the targetJointAngles into a trajectory:
		traj = []
		#traj = numpy.array(targetJointAngles[0][0:6]).tolist()
		#print(targetJointAngles[0][0])
		#print(targetJointAngles[1][0])
		joint1=targetJointAngles[0]
		joint1=targetJointAngles[0]
		joint2=targetJointAngles[1]
		joint3=targetJointAngles[2]
		joint4=targetJointAngles[3]
		joint5=targetJointAngles[4]
		joint6=targetJointAngles[5]
		#traj.append(targetJointAngles[0][0])
		#traj.append(targetJointAngles[1][0])
		#traj.append(targetJointAngles[2][0])
		#traj.append(targetJointAngles[3][0])
		#traj.append(targetJointAngles[4][0])
		#traj.append(targetJointAngles[5][0])
		traj.append(joint1)
		traj.append(joint2)
		traj.append(joint3)
		traj.append(joint4)
		traj.append(joint5)
		traj.append(joint6)
		print(traj)		
		#traj = [float(targetJointAngles[0,0]),(targetJointAngles[1,0]),(targetJointAngles[2,0]),(targetJointAngles[3,0]),(targetJointAngles[4,0]),(targetJointAngles[5,0])]
		#print(targetJointAngles)    
		# Generate the trajectory message to send to the Powerball:
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
		traj_msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint']
		point_nr = 0
		position = []
		# Set the target velocities of the target joints.  They are set to 0 to denote stopping at the destinations:
		for point in traj:	
			#print(point)	
			position.append((point))
			point_nr += 1
			point_msg = JointTrajectoryPoint()
			point_msg.positions = point
			point_msg.velocities = [0] * 6
			point_msg.time_from_start = rospy.Duration(3 * point_nr)
			traj_msg.points.append(point_msg)
		# Send the position control message to the action server node:
		position.append(-0.02885) 
		position.append(0.02885)
		obj1.position=position
		#print(obj1.position[0])
		#print(obj1.position[1])
		#print(obj1.position[2])
		#print(obj1.position[3])
		#print(obj1.position[4])
		#print(obj1.position[5])
		#print(obj1.position[6])
		#print(obj1.position[7])
		action_server_name = '/arm_controller/follow_joint_trajectory'
		pub.publish(obj1)
		#client = actionlib.SimpleActionClient(action_server_name, FollowJointTrajectoryAction)
		#if not client.wait_for_server(rospy.Duration(5)):
		#	print("Action server not ready within timeout.  Aborting...")
		#	ah.set_failed(4)
		#	return ah
		#else:
		#	print("Action server ready for Coordinate API Request")
		
		#client_goal = FollowJointTrajectoryGoal()
		#client_goal.trajectory = traj_msg
		#client.send_goal(client_goal)
		#ah.set_client(client)

		#ah.wait_inside()
		return 1
	

'''
This handler allows a user to initialize, halt, and emergency stop the Powerball   
'''
def init_halt_api_handler(req):
	rospy.loginfo("Inside  HaltAPI Service call!")	
	# Acquire the requested action (a string):
#	userCmd = req.command

	# What command is it?
#	if userCmd == 'init':
	#if userCmd == '1':		
	#	rospy.wait_for_service('/arm_controller/init')
	#	try:
	#		initRobot = rospy.ServiceProxy('/arm_controller/init', Trigger)
	#		resp = initRobot()

	#		print("resp.success.data is: " + str(resp.success.data))

			# If initialization fails, try again:
		#	while resp.success.data != True:
		#		resp = initRobot()
			
		#	return 0
	#	except rospy.ServiceException, e:
	'''		print("Service call failed when initializing robot: %s" % e)
	elif userCmd == 'halt':
	#elif userCmd == '2':
		rospy.wait_for_service('/arm_controller/halt')
		try:
			haltRobot = rospy.ServiceProxy('/arm_controller/halt', Trigger)
			resp = haltRobot()
			print("Stop the robot")
			return 0
		except rospy.ServiceException, e:
			print("Service call failed when calling robot halt: %s" % e)
	else:
		# Treat any other command received as an emergency stop:
		rospy.wait_for_service('/arm_controller/stop', Trigger)
		try:
			estopRobot = rospy.ServiceProxy('/arm_controller/stop', Trigger)
			resp = estopRobot()
			print("Emergency stop the robot")
			return 0
		except rospy.ServiceException, e:
			print("Service call failed when calling robot emergency stop; %s" % e)'''
	return 0
def api_server():

	# Initialize the API Server node:
	rospy.init_node('schunk_api_server')
	#rospy.Subscriber("/leapmotion/data", leapros2, leapmotioncallback)
	#only loginfo after init_node
	rospy.loginfo("Schunk API Node Up!")
	# Start service listeners to accept API calls given in Joint and Coordinate spaces:
	PositionAPIJoint = rospy.Service('PositionAPIJointSpace', PositionAPIJointSpace, position_api_joint_space_handler)
	rospy.loginfo("PositionAPIJointSpace Handle Up!")
	PositionAPICoord = rospy.Service('PositionAPICoordSpace', PositionAPICoordSpace, position_api_coord_space_handler)
	rospy.loginfo("PositionAPICoordpace Handle Up!")
	PositionAPICoordQuat = rospy.Service('PositionAPICoordSpaceQuat', PositionAPICoordSpaceQuat, position_api_coord_space_quat_handler)
	rospy.loginfo("PositionAPICoordpaceQuat Handle Up!")
	# This service accepts API calls to initialize, halt, and emergency stop the Powerball:
	InitHaltAPIsrv = rospy.Service('InitHaltAPI', InitHaltAPI, init_halt_api_handler)	   
	rospy.loginfo("InitHaltAPI Handle Up!")	
	rospy.spin() 

# Main function.  This node will listen for a position control message and will
# then execute the command.  
if __name__ == '__main__':	
	api_server()
