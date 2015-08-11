#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "leap_motion/leapros2.h"
#include "leap_motion/leap2.h"
#include <sstream>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <boost/bind.hpp>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
/*Global Variables*/
std_msgs::UInt8 myogest_;
leap_motion::leapros2 dataHand_;
leap_motion::leapros2 dataLastHand_;
geometry_msgs::Pose target_pose1;
//Callback pose
geometry_msgs::Pose target_pose2;

//const float Uplimitex;
//const float Downlimitex;
//const float Uplimitey;
//const float Downlimitey;
const float Uplimitez=1.32;
const float Downlimitez=1.0;
float Downdifferencez;
float Downdifferencey;
float Downdifferencex;
float Updifferencez;
float Updifferencey;
float Updifferencex;



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
double rot4 =0;
double rot5 = 0;
double rot6 = 0;
double rot7 = 0;
double rot8 = 0;
//Our myo only detectate change in the sensor reading. We want a continue output 0->stop 1->forward 2->backward
int myo_state=0;
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

//We implement the LeapMotionListener Class with the leapmotionCallback function. We will use this function as callback for
//subscriber
class LeapMotionListener
{

private:
  moveit::planning_interface::MoveGroup *pgroup;
  moveit::planning_interface::MoveGroup::Plan *pplan;
  int i;
  
public:
 
  void leapmotionCallback(const leap_motion::leapros2::ConstPtr& dataHand)
{
      dataHand_=(*dataHand);
      
      
      //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
      
      // Both limits for x,y,z to avoid small changes
      Updifferencex=dataLastHand_.palmpos.x+0.2;
      Downdifferencex=dataLastHand_.palmpos.x-0.2;
      Updifferencez=dataLastHand_.palmpos.z+0.3;
      Downdifferencez=dataLastHand_.palmpos.z-0.2;
      Updifferencey=dataLastHand_.palmpos.y+0.5;
      Downdifferencey=dataLastHand_.palmpos.y-0.5;
      //joint_msg_leap.header.stamp = ros::Time::now();
      
      if ((dataHand_.palmpos.x<Downdifferencex)||(dataHand_.palmpos.x>Updifferencex)||(dataHand_.palmpos.y<Downdifferencey)||(dataHand_.palmpos.y>Updifferencey)||(dataHand_.palmpos.z<Downdifferencez)||(dataHand_.palmpos.z>Updifferencez))
      {
      ros::AsyncSpinner spinner(1);
      spinner.start();
      //target_pose2.position.y +=(dataHand_.palmpos.x-dataLastHand_.palmpos.x)/100 ;
      target_pose2.position.z +=(dataHand_.palmpos.y-dataLastHand_.palmpos.y)/100 ;
      //target_pose2.position.x +=-(dataHand_.palmpos.z-dataLastHand_.palmpos.z)/100 ;
      if(target_pose2.position.z>Uplimitez)target_pose2.position.z=Uplimitez;
      if(target_pose2.position.z<Downlimitez)target_pose2.position.z=Downlimitez;
      //target_pose2.position.z += 0.2
      
      pgroup->setPoseTarget(target_pose2);
      bool success = pgroup->plan(*pplan);
      //pgroup->setStartStateToCurrentState();

      //we change the initial state
      //robot_state::RobotState start_state(*pgroup->getCurrentState());
      //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
      //target_pose2.position.x=dataHand_.palmpos.x-dataLastHand_.palmpos.x;
      //target_pose2.position.y=dataHand_.palmpos.y-dataLastHand_.palmpos.y;
      //target_pose2.position.z=dataHand_.palmpos.z-dataLastHand_.palmpos.z;*/
      //ROS_INFO("Move ARM");
      //sleep(5.0);
    /*const robot_state::JointModelGroup *joint_model_group =
    start_state.getJointModelGroup(pgroup->getName());
    if(start_state.setFromIK(joint_model_group, target_pose2))
    {
    ROS_INFO("ik calculated");
    sleep (1);
    }/*
    

      pgroup->setStartState(start_state);
      spinner.stop();
      
      //group.setPoseTarget(target_pose2);
      /*moveit::planning_interface::MoveGroup::Plan my_plan;
      bool success = group.plan(my_plan);
      
      
      */
      }
      else
      {
      //printf("Palmpos \n X: %f\n  Y: %f\n Z: %f\n ",dataHand_.palmpos.x,dataHand_.palmpos.y,dataHand_.palmpos.z);
      }
      //save new value of the last position of the hand
       dataLastHand_=(*dataHand);

      //we print the position of all the hand
      
      //}

}

  void setPointertoGroup(moveit::planning_interface::MoveGroup *pointertogroup)
  {
    pgroup=pointertogroup;
  }
  
  void setPointertoPlan(moveit::planning_interface::MoveGroup::Plan *pointertoplan)
  {
   pplan=pointertoplan;
  }


};



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
      //Create an object of the LeapMotionListener Class
      LeapMotionListener leapmotionlistener;
      
      //ROS DECLARATION
      ros::init(argc, argv,"listener");
      ros::NodeHandle n;
      // start a ROS spinning thread
      ros::AsyncSpinner spinner(1);
      spinner.start();
      //we need this for leap
      ros::Rate r(1);
      robo_pub = n.advertise<sensor_msgs::JointState>("joint_leap", 100);
      
      /* This sleep is ONLY to allow Rviz to come up */
      sleep(10.0);
      // MOVEIT Setup
      // ^^^^^
      moveit::planning_interface::MoveGroup group("arm");
      group.setPlanningTime(0.5);
      //pointer to group
      pointertogroup=&group;
      leapmotionlistener.setPointertoGroup(pointertogroup);
      moveit::planning_interface::MoveGroup::Plan my_plan;
      //pointer to plan
      pointertoplan=&my_plan;
      leapmotionlistener.setPointertoPlan(pointertoplan);
      // We will use the :planning_scene_interface:`PlanningSceneInterface`
      // class to deal directly with the world.
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
      // (Optional) Create a publisher for visualizing plans in Rviz.
      ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
      moveit_msgs::DisplayTrajectory display_trajectory;
      /*end of MOVEIT Setup*/
      //ROS Subscription
      //We have to create the JointState msg
      // We can print the name of the reference frame for this robot.
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      // We can also print the name of the end-effector link for this group.
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
      // Planning to a Pose goal
      // ^^^^^^^^^^^^^^^^^^^^^^^
      // We can plan a motion for this group to a desired pose for the 
      // end-effector  
      
      target_pose1.orientation.w = 1.0;
      target_pose1.position.x = 0.04;
      target_pose1.position.y = -0.08;
      target_pose1.position.z = 1.2;
      group.setPoseTarget(target_pose1);
      target_pose2 = target_pose1;
      // Now, we call the planner to compute the plan
      // and visualize it.
      // Note that we are just planning, not asking move_group 
      // to actually move the robot.
      
      bool success = group.plan(my_plan);
      
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
      sleep(5.0); 
      // Visualizing plans
      // ^^^^^^^^^^^^^^^^^
      // Now that we have a plan we can visualize it in Rviz.  This is not
      // necessary because the group.plan() call we made above did this
      // automatically.  But explicitly publishing plans is useful in cases that we
      // want to visualize a previously created plan.
      
      //ROS_INFO("Visualizing plan 1 (again)");    
      //display_trajectory.trajectory_start = my_plan.start_state_;
      //display_trajectory.trajectory.push_back(my_plan.trajectory_);
      //display_publisher.publish(display_trajectory);
      /* Sleep to give Rviz time to visualize the plan. */
      //sleep(5.0);
      spinner.stop();
      
      //After this first positioning we will subscribe the callback of our Leapmotionlistener
      ros::Subscriber leapsub = n.subscribe("/leapmotion/data", 1000, &LeapMotionListener::leapmotionCallback, &leapmotionlistener);
      
      //ros::Subscriber myogestsub = n.subscribe("/myo_gest", 1000, myogestCallback);
      ros::spin();
      return 0;
}


