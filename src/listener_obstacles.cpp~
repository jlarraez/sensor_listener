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
#include <tf/tf.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
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

//#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
/*Global Variables*/
std_msgs::UInt8 myogest_;

geometry_msgs::Pose target_pose1;
//Callback pose
geometry_msgs::PoseStamped old_pose;
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
      //Initialise PlanningSceneMonitorPtr

      
      /* MOVEIT Setup*/
      // ^^^^^
      moveit::planning_interface::MoveGroup group("arm");
      group.setPlanningTime(0.5);
      moveit::planning_interface::MoveGroup::Plan my_plan;
      // We will use the :planning_scene_interface:`PlanningSceneInterface`
      // class to deal directly with the world.
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
      
      /* Adding/Removing Objects and Attaching/Detaching Objects*/
      ROS_INFO("CREATING PLANNING_SCENE PUBLISHER");
      ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
      while(planning_scene_diff_publisher.getNumSubscribers() < 1)
      {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
      }
      ROS_INFO("CREATING COLLISION OBJECT");
      
      moveit_msgs::AttachedCollisionObject attached_object;
      attached_object.link_name = "";
      /* The header must contain a valid TF frame*/
      attached_object.object.header.frame_id = "gripper_left";
      /* The id of the object */
      attached_object.object.id = "box";

      /* A default pose */
      geometry_msgs::Pose posebox;
      posebox.orientation.w = 1.0;

      /* Define a box to be attached */
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.1;
      primitive.dimensions[1] = 0.1;
      primitive.dimensions[2] = 0.1;

      attached_object.object.primitives.push_back(primitive);
      attached_object.object.primitive_poses.push_back(posebox);
      attached_object.object.operation = attached_object.object.ADD;

      
      ROS_INFO("ADDING COLLISION OBJECT TO THE WORLD");
      
moveit_msgs::PlanningScene planning_scene_msg;
planning_scene_msg.world.collision_objects.push_back(attached_object.object);
planning_scene_msg.is_diff = true;
planning_scene_diff_publisher.publish(planning_scene_msg);
//sleep_time.sleep();

/* First, define the REMOVE object message*/
moveit_msgs::CollisionObject remove_object;
remove_object.id = "box";
remove_object.header.frame_id = "odom_combined";
remove_object.operation = remove_object.REMOVE;

/* Carry out the REMOVE + ATTACH operation */
ROS_INFO("Attaching the object to the right wrist and removing it from the world.");
planning_scene_msg.world.collision_objects.clear();
planning_scene_msg.world.collision_objects.push_back(remove_object);
planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_object);
planning_scene_diff_publisher.publish(planning_scene_msg);

//sleep_time.sleep();
     
      // Create a publisher for visualizing plans in Rviz.
      ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
      planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model,collision_detection::WorldPtr(new collision_detection::World())));
      
      //Creating planning_scene_monitor
      boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(planning_scene,"robot_description", tf));
      planning_scene_monitor->startSceneMonitor();
      //planning_scene_monitor->initialize(planning_scene);
      planning_pipeline::PlanningPipeline *planning_pipeline= new planning_pipeline::PlanningPipeline(robot_model,node_handle,"planning_plugin", "request_adapters");

      /* Sleep a little to allow time to startup rviz, etc. */
      ros::WallDuration sleep_time(20.0);
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
      planning_interface::MotionPlanRequest req;
      planning_interface::MotionPlanResponse res;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "/odom_combined";
      pose.pose.position.x = 0;
      pose.pose.position.y = 0;
      pose.pose.position.z = 1.15;
      pose.pose.orientation.w = 1.0;
      std::vector<double> tolerance_pose(3, 0.01);
      std::vector<double> tolerance_angle(3, 0.01);
      old_pose=pose;
      
      // We will create the request as a constraint using a helper function available
      req.group_name = "arm";
      moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("gripper_base_link", pose, tolerance_pose, tolerance_angle);
      req.goal_constraints.push_back(pose_goal);
      // Now, call the pipeline and check whether planning was successful.
      planning_pipeline->generatePlan(planning_scene, req, res);
      /* Check that the planning was successful */
      if(res.error_code_.val != res.error_code_.SUCCESS)
      {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
      }
      // Visualize the result
      // ^^^^^^^^^^^^^^^^^^^^
      /* Visualize the trajectory */
      moveit_msgs::DisplayTrajectory display_trajectory;
      ROS_INFO("Visualizing the trajectory 1");
      moveit_msgs::MotionPlanResponse response;
      res.getMessage(response);
      display_trajectory.trajectory_start = response.trajectory_start;
      display_trajectory.trajectory.push_back(response.trajectory);
      display_publisher.publish(display_trajectory);
      //sleep_time.sleep();
      /* End Planning to a Pose goal 1*/

     // First, set the state in the planning scene to the final state of the last plan 
      robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
      //planning_scene->setCurrentState(response.trajectory_start);
      joint_model_group = robot_state.getJointModelGroup("arm");
      robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
      spinner.stop();
      
      /*Capturing Stage*/
      /*****************/
      ROS_INFO("PRESH ENTER TO START CAPTURING POINTS");
      while (getline(std::cin,s))
      {
        if ('\n' == getchar())
          break;
      }
      
      /* SENSOR SUBSCRIBING */
      //LEAP MOTION
      ROS_INFO("SUBSCRIBING LEAPMOTION");
      ros::Subscriber leapsub = node_handle.subscribe("/leapmotion/data", 1000, &LeapMotionListener::leapmotionCallback, &leapmotionlistener);
      ros::Subscriber trajectorysub = node_handle.subscribe("/move_group/", 1000, &LeapMotionListener::leapmotionCallback, &leapmotionlistener);
      while(!CAPTURE_MOVEMENT==true)
      {
       
       ros::spinOnce();
       
      }
      leapsub.shutdown();
      ROS_INFO("CAPTURING POINTS FINISH...PROCESSING POINTS");
      // End of Capturing Stage
      

      /* Start Creating Arm Trajectory*/
      /**********************************/
      
      ROS_INFO("START CREATING ARM TRAJECTORY");
      for (unsigned i=0; i<trajectory_hand.size(); i++)
      {
        //First we set the initial Position of the Hand
        if (FIRST_VALUE)
        {
        dataLastHand_.palmpos.x=trajectory_hand.at(i).palmpos.x;
        dataLastHand_.palmpos.y=trajectory_hand.at(i).palmpos.y;
        dataLastHand_.palmpos.z=trajectory_hand.at(i).palmpos.z;
        FIRST_VALUE=0;
        ROS_INFO("ORIGINAL POSITION OF THE HAND SET TO \n X: %f\n  Y: %f\n Z: %f\n ",trajectory_hand.at(i).palmpos.x,trajectory_hand.at(i).palmpos.y,trajectory_hand.at(i).palmpos.z);
        sleep(2);
        }
        else
        {
        // Both limits for x,y,z to avoid small changes
        Updifferencex=dataLastHand_.palmpos.x+10;
        Downdifferencex=dataLastHand_.palmpos.x-10;
        Updifferencez=dataLastHand_.palmpos.z+10;
        Downdifferencez=dataLastHand_.palmpos.z-20;
        Updifferencey=dataLastHand_.palmpos.y+20;
        Downdifferencey=dataLastHand_.palmpos.y-20;
        if ((trajectory_hand.at(i).palmpos.x<Downdifferencex)||(trajectory_hand.at(i).palmpos.x>Updifferencex)||(trajectory_hand.at(i).palmpos.y<Downdifferencey)||(trajectory_hand.at(i).palmpos.y>Updifferencey)||(trajectory_hand.at(i).palmpos.z<Downdifferencez)||(trajectory_hand.at(i).palmpos.z>Updifferencez))
          {
            
            ros::AsyncSpinner spinner(1);
            spinner.start();
            ROS_INFO("TRYING TO ADD POINT %d TO TRAJECTORY",arm_trajectory_point);
            // Cartesian Paths
            // ^^^^^^^^^^^^^^^
            // You can plan a cartesian path directly by specifying a list of waypoints
            // for the end-effector to go through. Note that we are starting
            // from the new start state above. The initial pose (start state) does not
            // need to be added to the waypoint list.
            pose.header.frame_id = "/odom_combined";
            pose.pose.orientation.w = 1.0;
            pose.pose.position.y +=(trajectory_hand.at(i).palmpos.x-dataLastHand_.palmpos.x)/500 ;
            pose.pose.position.z +=(trajectory_hand.at(i).palmpos.y-dataLastHand_.palmpos.y)/1000 ;
            if(pose.pose.position.z>Uplimitez)
            pose.pose.position.z=Uplimitez;
            pose.pose.position.x +=-(trajectory_hand.at(i).palmpos.z-dataLastHand_.palmpos.z)/500 ;
            ROS_INFO("END EFFECTOR POSITION \n X: %f\n  Y: %f\n Z: %f\n", pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
            ROS_INFO("Palmpos \n X: %f\n  Y: %f\n Z: %f\n ",trajectory_hand.at(i).palmpos.x,trajectory_hand.at(i).palmpos.y,trajectory_hand.at(i).palmpos.z);
            // We will create the request as a constraint using a helper function available
            ROS_INFO("1");
            pose_goal= kinematic_constraints::constructGoalConstraints("gripper_base_link", pose, tolerance_pose, tolerance_angle);
            ROS_INFO("2");
            req.goal_constraints.push_back(pose_goal);
            
            // Now, call the pipeline and check whether planning was successful.
            planning_pipeline->generatePlan(planning_scene, req, res);
            ROS_INFO("3");
            if(res.error_code_.val != res.error_code_.SUCCESS)
            {
            ROS_ERROR("Could not compute plan successfully");
            pose=old_pose;
            }
            else
            {
            // Now when we plan a trajectory it will avoid the obstacle
            
            
            res.getMessage(response);
            
            collision_result.clear();
            collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
            robot_state::RobotState copied_state = planning_scene->getCurrentState();
            planning_scene->checkCollision(collision_request, collision_result, copied_state, acm);
            ROS_INFO_STREAM("Collision Test "<< (collision_result.collision ? "in" : "not in")<< " collision");
            arm_trajectory_point++;
            // Visualize the trajectory 
            ROS_INFO("VISUALIZING NEW POINT");
            //req.goal_constraints.clear();
            display_trajectory.trajectory_start = response.trajectory_start;
            ROS_INFO("AXIS 1 NEXT TRAJECTORY IS %f\n",response.trajectory_start.joint_state.position[1] );
            display_trajectory.trajectory.push_back(response.trajectory); 
            // Now you should see two planned trajectories in series
            display_publisher.publish(display_trajectory);
            planning_scene->setCurrentState(response.trajectory_start);
            if (planning_scene_monitor->updatesScene(planning_scene))
            {
            ROS_INFO("CHANGING STATE");
            planning_scene_monitor->updateSceneWithCurrentState();
            }
            else
            {
            ROS_ERROR("NOT POSSIBLE TO CHANGE THE SCENE");
            }
            robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
            req.goal_constraints.clear();
            old_pose=pose;
            dataLastHand_.palmpos.x=trajectory_hand.at(i).palmpos.x;
            dataLastHand_.palmpos.y=trajectory_hand.at(i).palmpos.y;
            dataLastHand_.palmpos.z=trajectory_hand.at(i).palmpos.z;
            }
            //sleep(2);
            spinner.stop();     
          }
          
        }
      
      }  
   //ros::Subscriber myogestsub = n.subscribe("/myo_gest", 1000, myogestCallback);
        
      return 0;
}

















