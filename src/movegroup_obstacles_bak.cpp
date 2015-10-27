#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "std_msgs/UInt8.h"
#include "leap_motion/leapros2.h"
#include "leap_motion/leap2.h"
#include <sstream>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
//Planning Tutorial
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//Kinematik Tutorial
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
//Planning scene
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/bind.hpp>

//Schunk_api msg include
#include "sensor_listener/PositionAPICoordSpaceQuat.h"
#include "sensor_listener/InitHaltAPI.h"

//Pedal includes for changing way we read values from keyboard
#include<stdio.h>
#include <termios.h>     //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

//Allow to use Pythons scripts
#include </usr/include/python2.7/Python.h>
#include <numpy/arrayobject.h>
#include <math.h>

//#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
/*Global Variables*/
std_msgs::UInt8 myogest_;

geometry_msgs::Pose target_pose1;
//Callback pose
geometry_msgs::Pose old_pose;
//waypoints
geometry_msgs::PoseStamped pose;




std::string s;
const double degree = M_PI/180;
//von Experiment: max Finger_distance=163 min Finger_distance=14
//163-14/(0.548-0)=(163-x)/(0.548-y)
const double DtAx=0.003677852;
const double DtA=-0.05148933;
//Variable of the rotation in grades of every axi
double rot0 = 0;
double* prot;
double rot1 = 0;
double rot2 = 0;
double rot3 = 0;
double rot4 = 0;
double rot5 = 0;
double rot6 = 0;
double rot7 = 0;
double rot8 = 0;

leap_motion::leapros2 dataLastHand_;

const float Uplimitez=1.32;
const float Downlimitez=1.0;
float Downdifferencez;
float Downdifferencey;
float Downdifferencex;
float Updifferencez;
float Updifferencey;
float Updifferencex;

//Our myo only detectate change in the sensor reading. We want a continue output 0->stop 1->forward 2->backward
int myo_state=0;
bool FIRST_VALUE=false;
bool CAPTURE_MOVEMENT;
//Variables for comparing with the output of the myo sensor
std_msgs::UInt8 BACKWARD;
std_msgs::UInt8 FORWARD;
std_msgs::UInt8 SWITCHMODE; 
//ROs publisher
ros::Publisher robo_pub;
//msg for the joint
sensor_msgs::JointState joint_msg_leap;
//Pointer to moveit Objects
moveit::planning_interface::MoveGroup *pointertogroup;
moveit::planning_interface::MoveGroup::Plan *pointertoplan;
planning_pipeline::PlanningPipeline *pointertoppipeline;
planning_scene::PlanningScenePtr *pointertopscene;
planning_scene::PlanningScene *pointertopscene1;
const robot_model::JointModelGroup* joint_model_group;
ros::Publisher *pointertod_publisher;
ros::Subscriber *pointertod_subscriber;
//Number of points we have implement for the trajectory of the arm
int arm_trajectory_point;
//waypoints
std::vector<geometry_msgs::Pose> waypoints;

//We declare a vectr of PoseStamp
std::vector<leap_motion::leapros2> trajectory_hand; 
leap_motion::leapros2 TrajectoryPoint;

//Count
int count;
int aux_enter;


//We implement the LeapMotionListener Class with the leapmotionCallback function. We will use this function as callback for
//subscriber
class LeapMotionListener
{

private:
  /*Limit variables*/
  //const float Uplimitex;
  //const float Downlimitex;
  //const float Uplimitey;
  //const float Downlimitey;

  
  moveit::planning_interface::MoveGroup *pgroup;
  moveit::planning_interface::MoveGroup::Plan *pplan;
  planning_pipeline::PlanningPipeline *pplanningpipeline;
  planning_scene::PlanningScenePtr *pplanningscene;
  planning_scene::PlanningScenePtr planningscene;
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;
  moveit_msgs::Constraints pose_goal;
  int counter;
  ros::Publisher *pdisplay_publisher;
  ros::Subscriber *pdisplay_subscriber;

  
public:
   void leapmotionCallback(const leap_motion::leapros2::ConstPtr& dataHand);
   void setPointertoGroup(moveit::planning_interface::MoveGroup *pointertogroup);
   void setPointertoPlan(moveit::planning_interface::MoveGroup::Plan *pointertoplan);
   void setPointertoPipe(planning_pipeline::PlanningPipeline *pointertoppipeline);
   void setPointertoScene(planning_scene::PlanningScenePtr *pointertopscene);
   void setPointertoPublisher(ros::Publisher *pointertod_publisher);
   void setPointertoSubscriber(ros::Subscriber *pointertod_subscriber);
   void Configure(int count);
   LeapMotionListener();
   
   leap_motion::leapros2 dataHand_;

   
};
 
 LeapMotionListener::LeapMotionListener(void)
{     
      
      
  
      
} 

  void LeapMotionListener::Configure(int count)
{
   counter=count;   
    
}
  void LeapMotionListener::leapmotionCallback(const leap_motion::leapros2::ConstPtr& dataHand)
{
      dataHand_=(*dataHand);
      ROS_INFO("INSIDE CALLBACK");
      if (counter>0)
      {
        ROS_INFO("ADDING POINT %d TO TRAJECTORY",counter);
        trajectory_hand.push_back(dataHand_);
        counter--;
      }
      else
      {
        ROS_INFO("TRAJECTORY OF THE HAND COMPLETE");
        CAPTURE_MOVEMENT=true;
       sleep(5);
      }
    
}

  void LeapMotionListener::setPointertoGroup(moveit::planning_interface::MoveGroup *pointertogroup)
  {
    pgroup=pointertogroup;
  }
  
  void LeapMotionListener::setPointertoPlan(moveit::planning_interface::MoveGroup::Plan *pointertoplan)
  {
   pplan=pointertoplan;
  }

  void LeapMotionListener::setPointertoPipe(planning_pipeline::PlanningPipeline *pointertoppipeline)
  {
   pplanningpipeline=pointertoppipeline;
  }
   void LeapMotionListener::setPointertoScene(planning_scene::PlanningScenePtr *pointertopscene)
  {
   pplanningscene=pointertopscene;
   planningscene=*pplanningscene;
  }
  void LeapMotionListener::setPointertoPublisher(ros::Publisher *pointertod_publisher)
  {
   pdisplay_publisher=pointertod_publisher;
  }
  
    void LeapMotionListener::setPointertoSubscriber(ros::Subscriber *pointertod_subscriber)
  {
   pdisplay_subscriber=pointertod_subscriber;
  }




/*void myogestCallback( const std_msgs::UInt8::ConstPtr& myogest)
{
      
      myogest_=(*myogest);
      joint_msg_leap.header.stamp = ros::Time::now();
      printf("%08x\n",myogest_);
      if(myogest_.data==BACKWARD.data)
      {
        	//ROS_INFO("BACKWARD");
        	myo_state=2;
	}	
      if(myogest_.data==FORWARD.data)
      {
	      //ROS_INFO("FORWARD");
	      myo_state=1;
      }
       if(myogest_.data==SWITCHMODE.data)
      {
	      //ROS_INFO("FORWARD");
	      myo_state=3;
      }
}*/


int main(int argc, char **argv)
{     
      /*Initialise Variables*/
      
      CAPTURE_MOVEMENT=false;//know when you have reach the maximum of points to handle
      //Creating the joint_msg_leap
      joint_msg_leap.name.resize(8);
      joint_msg_leap.position.resize(8);
      joint_msg_leap.name[0]="arm_1_joint";
      joint_msg_leap.name[1]="arm_2_joint";
      joint_msg_leap.name[2]="arm_3_joint";
      joint_msg_leap.name[3]="arm_4_joint";
      joint_msg_leap.name[4]="arm_5_joint";
      joint_msg_leap.name[5]="arm_6_joint";
      joint_msg_leap.name[6]="base_joint_gripper_left";
      joint_msg_leap.name[7]="base_joint_gripper_right";
      aux_enter=1;
      FIRST_VALUE=true;//Help knowing Initial Position of the hand
      int arm_trajectory_point=1;
      
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      moveit_msgs::DisplayTrajectory display_trajectory;
      
      /*Finish Variables Initialitation*/

      //ROS DECLARATION
      ros::init(argc, argv,"listener");
      if (argc != 2)
      {
        ROS_WARN("WARNING: you should specify number of points to capture");
      }
      else
      {
        count=atoi(argv[1]);
        ROS_INFO ("Number of points /n%d", count);
      }
      
      //Create an object of the LeapMotionListener Class
      LeapMotionListener leapmotionlistener;
      leapmotionlistener.Configure(count);
      
      //ros::NodeHandle node_handle;
      ros::NodeHandle node_handle("~");
      // start a ROS spinning thread
      ros::AsyncSpinner spinner(1);
      spinner.start();
      //we need this for leap
      ros::Rate r(1);
            
      //robo_pub = n.advertise<sensor_msgs::JointState>("joint_leap", 100);
      
      //Creating a Robot Model
      robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
      
      /* MOVEIT Setup*/
      // ^^^^^
      moveit::planning_interface::MoveGroup group("arm");
      group.setPlanningTime(0.5);
      moveit::planning_interface::MoveGroup::Plan my_plan;
      // We will use the :planning_scene_interface:`PlanningSceneInterface`
      // class to deal directly with the world.
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
      
      // Create a publisher for visualizing plans in Rviz.
      ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);


      /* Sleep a little to allow time to startup rviz, etc. */
      ros::WallDuration sleep_time(15.0);
      sleep_time.sleep();  
      /*end of MOVEIT Setup*/

      // We can print the name of the reference frame for this robot.
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      // We can also print the name of the end-effector link for this group.
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
      
      // Planning to a Pose goal 1
      // ^^^^^^^^^^^^^^^^^^^^^^^
      // We can plan a motion for this group to a desired pose for the 
      // end-effector  
      ROS_INFO("Planning to INITIAL POSE");
      
      geometry_msgs::Pose pose;
      pose.position.x = 0.0;
      pose.position.y = 0.3;
      pose.position.z = 0.7;
      pose.orientation.w = 1.0;
      //std::vector<double> tolerance_pose(3, 0.01);
      //std::vector<double> tolerance_angle(3, 0.01);
      old_pose=pose;
      group.setPoseTarget(pose);
      bool success = group.plan(my_plan);
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

      // Visualize the result
      // ^^^^^^^^^^^^^^^^^^^^
      /* Visualize the trajectory */
      if (1)
      {
      
      display_trajectory.trajectory_start = my_plan.start_state_;
      display_trajectory.trajectory.push_back(my_plan.trajectory_);
      display_publisher.publish(display_trajectory);
      sleep(5.0);
      }
      /* End Planning to a Pose goal 1*/

     // First, set the state in the planning scene to the final state of the last plan 
     
      robot_state::RobotState start_state(*group.getCurrentState());
      //joint_model_group = start_state.getJointModelGroup("arm");
      //start_state.setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
      //group.setStartState(start_state);
      //my_plan.start_state_=start_state;
      //spinner.stop();
      

      /* Adding/Removing Objects and Attaching/Detaching Objects*/
      ROS_INFO("CREATING PLANNING_SCENE PUBLISHER");
      ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
      while(planning_scene_diff_publisher.getNumSubscribers() < 1)
      {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
      }
      moveit_msgs::CollisionObject right_wall;
      moveit_msgs::CollisionObject left_wall;
      moveit_msgs::CollisionObject top_roof;
      moveit_msgs::CollisionObject base;
      moveit_msgs::CollisionObject collision_object;
      right_wall.header.frame_id = group.getPlanningFrame();
      left_wall.header.frame_id = group.getPlanningFrame();
      top_roof.header.frame_id = group.getPlanningFrame();
      collision_object.header.frame_id = group.getPlanningFrame();
      /* The id of the object is used to identify it. */
      right_wall.id = "right_wall";
      left_wall.id = "left_wall";
      top_roof.id = "top_roof";
      base.id = "base"; 
      collision_object.id = "box1";       
      /* Define the lateral wall to add them to the world. */
      shape_msgs::SolidPrimitive side_wall1;
      side_wall1.type = side_wall1.BOX;
      side_wall1.dimensions.resize(3);
      side_wall1.dimensions[1] = 0.115;
      side_wall1.dimensions[0] = 0.47;
      side_wall1.dimensions[2] = 0.83;
      
      // Define the top roof to add them to the world.     
      shape_msgs::SolidPrimitive top_wall1;
      top_wall1.type = top_wall1.BOX;
      top_wall1.dimensions.resize(3);
      top_wall1.dimensions[1] = 0.40;
      top_wall1.dimensions[0] = 0.67;
      top_wall1.dimensions[2] = 0.115;
      
      shape_msgs::SolidPrimitive base_wall;
      base_wall.type = base_wall.CYLINDER;
      base_wall.dimensions.resize(2);
      base_wall.dimensions[0] = 0.75;
      base_wall.dimensions[1] = 0.372;
      
      // Define a box to add to the world. 
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.25;
      primitive.dimensions[1] = 0.1;
      primitive.dimensions[2] = 0.25;
      
      // Pose of the right wall 
      geometry_msgs::Pose right_pose;
      right_pose.orientation.w = 1.0;
      right_pose.position.y =  0.2575;
      right_pose.position.x = -0.09;
      right_pose.position.z =  0.415;
      
      // Pose of the left wall       
      
      geometry_msgs::Pose left_pose;
      left_pose.orientation.w = 1.0;
      left_pose.position.y = -0.2575;
      left_pose.position.x = -0.09;
      left_pose.position.z =  0.415;
      
      // Pose of the base
      geometry_msgs::Pose base_pose;
      base_pose.orientation.w = 1.0;
      base_pose.position.y = 0;
      base_pose.position.x = 0;
      base_pose.position.z = -0.4;
      
      // Pose of the top roof
           
      geometry_msgs::Pose top_pose;
      top_pose.orientation.w = 1.0;
      top_pose.position.y = 0.0;
      top_pose.position.x = -0.055;
      top_pose.position.z =  0.775;
      
      /* A pose for the box (specified relative to frame_id) */
      geometry_msgs::Pose box_pose;
      box_pose.orientation.w = 1.0;
      box_pose.position.x =  -0.15;
      box_pose.position.y =  0.15;
      box_pose.position.z =  0.7;
      
      //collision box 
      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD; 
      /* 
          
      //right wall
      right_wall.primitives.push_back(side_wall1);
      right_wall.primitive_poses.push_back(right_pose);
      right_wall.operation = right_wall.ADD;
      
      //left wall
      left_wall.primitives.push_back(side_wall1);
      left_wall.primitive_poses.push_back(left_pose);
      left_wall.operation = left_wall.ADD;
      
      //top_roof
      top_roof.primitives.push_back(top_wall1);
      top_roof.primitive_poses.push_back(top_pose);
      top_roof.operation = top_roof.ADD;
      
      //base
      base.primitives.push_back(base_wall);
      base.primitive_poses.push_back(base_pose);
      base.operation = base.ADD;
      */
      std::vector<moveit_msgs::CollisionObject> collision_objects;
      collision_objects.push_back(collision_object);
      //collision_objects.push_back(right_wall);
      //collision_objects.push_back(left_wall);
      //collision_objects.push_back(top_roof);
      //collision_objects.push_back(base);

      ROS_INFO("ADDING COLLISION OBJECT TO THE WORLD");
      planning_scene_interface.addCollisionObjects(collision_objects);
      moveit_msgs::PlanningScene planning_scene_msg;
      planning_scene_msg.world.collision_objects.push_back(collision_object);
      //planning_scene_msg.world.collision_objects.push_back(right_wall);                 
      //planning_scene_msg.world.collision_objects.push_back(left_wall);
      //planning_scene_msg.world.collision_objects.push_back(top_roof);
      //planning_scene_msg.world.collision_objects.push_back(base);
      planning_scene_msg.is_diff = true;
      planning_scene_diff_publisher.publish(planning_scene_msg);
      //sleep_time.sleep();
      
      // Planning to a Pose goal 2 with the object
      // ^^^^^^^^^^^^^^^^^^^^^^^
      // We can plan a motion for this group to a desired pose for the 
      // end-effector  
      ROS_INFO("Planning to INITIAL POSE");
      group.setPlanningTime(10.0);
      group.setStartState(*group.getCurrentState());
      geometry_msgs::Pose pose2;
      pose2.position.x = 0.0;
      pose2.position.y = 0.3;
      pose2.position.z = 0.7;
      pose2.orientation.w = 1.0;
      //std::vector<double> tolerance_pose(3, 0.01);
      //std::vector<double> tolerance_angle(3, 0.01);
      //old_pose=pose;
      group.setPoseTarget(pose2);
       success = group.plan(my_plan);
      ROS_INFO("Visualizing plan 2 collision test %s",success?"":"FAILED");

      // Visualize the result
      // ^^^^^^^^^^^^^^^^^^^^
      /* Visualize the trajectory */
      /*
      display_trajectory.trajectory_start = my_plan.start_state_;
      display_trajectory.trajectory.push_back(my_plan.trajectory_);
      display_publisher.publish(display_trajectory);
      */
      
        
   //ros::Subscriber myogestsub = n.subscribe("/myo_gest", 1000, myogestCallback);
        
      return 0;
}

















