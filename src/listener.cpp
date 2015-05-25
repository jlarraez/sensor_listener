#include "ros/ros.h"
#include "std_msgs/String.h"
#include "leap_motion/leapros2.h"
#include "leap_motion/leap2.h"
#include <sstream>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
/*Global Variables*/
leap_motion::leapros2 dataHand_;
leap_motion::leapros2 dataLastHand_;

float Uplimitex;
float Downlimitex;
float Uplimitey;
float Downlimitey;
float Uplimitez;
float Downlimitez;
const double degree = M_PI/180;
double rot7 = 0;
double rot0 = 0;
double rot1 = 0;
double rot2 = 0;
double rot3 = 0;
ros::Publisher robo_pub;
//tf::TransformBroadcaster broadcaster;
sensor_msgs::JointState joint_msg_leap;
//geometry_msgs::TransformStamped odom_trans;



void leapmotionCallback(const leap_motion::leapros2::ConstPtr& dataHand)
{
  //ros::Rate loop_rate(30);
  //odom_trans.header.frame_id = "odom";
//odom_trans.child_frame_id = "base_link";
  dataHand_=(*dataHand);
  // Both limits for x,y,z to avoid small changes
  Uplimitex=dataLastHand_.palmpos.x+0.3;
  Downlimitex=dataLastHand_.palmpos.x-0.3;
  Uplimitez=dataLastHand_.palmpos.z+0.3;
  Downlimitez=dataLastHand_.palmpos.z-0.3;
  Uplimitey=dataLastHand_.palmpos.y+0.5;
  Downlimitey=dataLastHand_.palmpos.y-0.5;

  //We print the direction of the x Movement
  if (dataHand_.palmpos.x>Uplimitex)
  {
  ROS_INFO("RIGHT");
  }
  else 
  {
	  if (dataHand_.palmpos.x<Downlimitex)
	  {
		ROS_INFO("LEFT");
	  }
  }

  if (dataHand_.palmpos.z>Uplimitez)
  {
  	ROS_INFO("BACKWARD");
		joint_msg_leap.header.stamp = ros::Time::now();
		joint_msg_leap.position[0] = rot0*degree;
		joint_msg_leap.position[1] = rot1*degree;
		joint_msg_leap.position[2] = rot2*degree;
		joint_msg_leap.position[3] = rot3*degree;
		robo_pub.publish(joint_msg_leap);
		rot0 -= 1;
    		if (rot0 < -180) rot0 = 180;
		rot1 -= 1;
    		if (rot1 < -180) rot1 = 180;
		rot2 -= 1;
    		if (rot2 < -180) rot2 = 180;
		rot3 -= 1;
    		if (rot3 < -180) rot3 = 180;
		
  }
  else 
  {
	  if (dataHand_.palmpos.z<Downlimitez)
	  {
		ROS_INFO("FORWARD");
		joint_msg_leap.header.stamp = ros::Time::now();
		joint_msg_leap.position[0] = rot0*degree;
		joint_msg_leap.position[1] = rot1*degree;
		joint_msg_leap.position[2] = rot2*degree;
		joint_msg_leap.position[3] = rot3*degree;
		robo_pub.publish(joint_msg_leap);
		rot0 += 1;
    		if (rot0 > 180) rot0 = -180;
		rot1 += 1;
    		if (rot1 > 180) rot1 = -180;
		rot2 += 1;
    		if (rot2 > 180) rot2 = -180;
		rot7 += 1;
    		if (rot3 > 180) rot3 = -180;
		robo_pub.publish(joint_msg_leap);
		//broadcaster.sendTransform(odom_trans);
	  }
  }

  
  if (dataHand_.palmpos.y>Uplimitey)
  {
  	ROS_INFO("UP");
  }
  else 
  {
	  if (dataHand_.palmpos.y<Downlimitey)
	  {
	  	ROS_INFO("DOWN");
	  }
  }
  
  switch (dataHand_.type_gesture)
  {
  	case 1: 
		ROS_INFO("SWIPE");
		break;
	case 4: 
		ROS_INFO("KEY_TAP");
		break;
	case 5: 
		ROS_INFO("CIRCLE");
		//head_position=head_position+0.1;
		//if (head_position>3.14)
		//update joint_msg
		//joint_msg_leap.name.resize(8);
		
		joint_msg_leap.header.stamp = ros::Time::now();
		joint_msg_leap.position[7] = rot7*degree;
		/*odom_trans.header.stamp = ros::Time::now();
    		odom_trans.transform.translation.x = 0;
    		odom_trans.transform.translation.y = 0;
    		odom_trans.transform.translation.z = 0;
    		odom_trans.transform.rotation = 		   tf::createQuaternionMsgFromYaw(0);*/
		robo_pub.publish(joint_msg_leap);
		//broadcaster.sendTransform(odom_trans);
		rot7 += 1;
    		if (rot7 > 180) rot7 = -180;

 		break;


  }
  //save new value of the last position of the hand
  dataLastHand_=(*dataHand);

  
  //we print the position of all the hand
  printf("Palmpos \n X: %f\n  Y: %f\n Z: %f\n ",dataHand_.palmpos.x,dataHand_.palmpos.y,dataHand_.palmpos.z);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"listener");
  ros::NodeHandle n;
  ros::Rate r(1);
  //robo_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 100);
  robo_pub = n.advertise<sensor_msgs::JointState>("joint_leap", 100);
  joint_msg_leap.name.resize(8);
  joint_msg_leap.position.resize(8);
  joint_msg_leap.name[0] ="right_front_wheel_joint";
  joint_msg_leap.name[1] ="right_back_wheel_joint";
  joint_msg_leap.name[2] ="left_front_wheel_joint";
  joint_msg_leap.name[3] ="left_back_wheel_joint";
  joint_msg_leap.name[4] ="gripper_extension";
  joint_msg_leap.name[5] ="left_gripper_joint";
  joint_msg_leap.name[6] ="right_gripper_joint";
  joint_msg_leap.name[7] ="head_swivel";
  ros::Subscriber sub = n.subscribe("/leapmotion/data", 1000, leapmotionCallback);
  ros::spin();
  return 0;
}
