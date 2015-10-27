Explanation of the files we have in the folder

-powerball_diret_griper.cpp

Current file were we make our last changes. 
LAST CHANGE 12/10/15: Direct control to the robot including the gripper and sincronise with the simulation. Currently only gripper will move. First need to presh 1 to run reading sensor and send the Initliase signal to the CAN.

To run it we should write on the Terminal
~/catkin_ws/devel/setup.bash
roslaunch sensor_listener powerball_gripper.launch
roslaunch sensor_listener robot.launch 

it work together with schunk_api_node.py kinematics_functions.py powerball_constants.py

-movegroup_obstacles.cpp
LAST CHANGE: 07/2015. Avoid collision of the robot with MoveIt!

to run use 

First compile the file uncommenting 
add_executable(listener src/movegroup_obstacles.cpp)
 target_link_libraries(listener
   ${catkin_LIBRARIES}
)

in the CMakeLists.txt

roslaunch pr2_moveit_tutorials move_group_interface_tutorial.launch

-pedal_security.cpp

LAST CHANGE: 08/2015

Attempt to use the foot pedal to stop the robot

-powerball_direct.cpp

LAST CHANGE: 09/2015

Simulate the robot arm and the gripper. Dont need to have the robot connected.

To make it run 

- Uncomment in sensor_listener/srv/PositionAPICoordSpaceQuat.srv all the joints
- Uncomment in CMakeLists.txt
add_executable(listener_direct src/powerball_direct.cpp)
 target_link_libraries(listener_direct
   ${catkin_LIBRARIES}
)
-Compile
-Write in the terminal
-~/catkin_ws/devel/setup.bash
-roslaunch sensor_listener robot_sim.launch 





