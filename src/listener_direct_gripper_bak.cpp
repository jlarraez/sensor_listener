#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "std_msgs/UInt8.h"
//leapmotion msg include
#include "leap_motion/leapros2.h"
#include "leap_motion/leap2.h"
//Schunk_api msg include
#include "sensor_listener/PositionAPICoordSpaceQuat.h"
#include "sensor_listener/InitHaltAPI.h"


#include <sstream>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
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

//Pedal includes for changing way we read values from keyboard
#include<stdio.h>
#include <termios.h>     //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

//Allow to use Pythons scripts
#include </usr/include/python2.7/Python.h>
#include <numpy/arrayobject.h>
#include <math.h>
//In order to work export PYTHONPATH="${PYTHONPATH}:/home/jorgearraez/catkin_ws/src/sensor_listener/src and create __init__.py in that directory"

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
geometry_msgs::Pose pose;


//We define the 4x4 Matrix wit the orientation and the position


std::string command="init";
int s;
const double degree = M_PI/180;
//Simulation:von Experiment: max Finger_distance=163 min Finger_distance=14
//163-14/(0.06-0)=(163-x)/(0.06-y)
//Real:von Experiment: max Finger_distance=163 min Finger_distance=14
//163-14/(0.06-0)=(163-x)/(0.06-y)
//163-14/1.14=(163-x)/(1.14-y)
// teoretico sim
//const double DtAx=-0.000402685;
//const double DtA=0.005637593;
//sim real
//const double DtAx=-0.000502685;
//const double DtA=0.007;
//real teoretico

const float DtAx=0.006308725;
const float DtA=-0.111677852;

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
float gripper_pose=0.0;
float old_gripper_pose=0.0;

leap_motion::leapros2 dataLastHand_;
//Transformation of yout movement of the arm into movement of the robot
const float FactorTransformation=2;
const float Uplimitez=1300;
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
ros::Subscriber robo_sub;
//msg from the joinstate
sensor_msgs::JointState jointstate_;
//msg that we send
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
ros::ServiceClient *pointerto_client;
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
//Srv to comunicate with CLient and Server
sensor_listener::PositionAPICoordSpaceQuat srv;



//joinstateCallback 
void joinstateCallback (const sensor_msgs::JointState::ConstPtr& JointState)
{
jointstate_=*JointState;
//ROS_INFO("JOINT STATE \n J1: %f\n  J2: %f\n J3: %f\n J4: %f\n  J5: %f\n J6: %f\n J5: %f\n J6: %f\n ",jointstate_.position[0],jointstate_.position[1],jointstate_.position[2],jointstate_.position[3],jointstate_.position[4],jointstate_.position[5],jointstate_.position[6],jointstate_.position[7],jointstate_.position[7]);
}

//LeapMotonListener Class 
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
  ros::ServiceClient *pclient;
  tf::Quaternion q;


  
public:
   void leapmotionCallback(const leap_motion::leapros2::ConstPtr& dataHand);
   void setPointertoGroup(moveit::planning_interface::MoveGroup *pointertogroup);
   void setPointertoPlan(moveit::planning_interface::MoveGroup::Plan *pointertoplan);
   void setPointertoPipe(planning_pipeline::PlanningPipeline *pointertoppipeline);
   void setPointertoScene(planning_scene::PlanningScenePtr *pointertopscene);
   void setPointertoPublisher(ros::Publisher *pointertod_publisher);
   void setPointertoSubscriber(ros::Subscriber *pointertod_subscriber);
   void setPointertoClient(ros::ServiceClient *pointerto_client);
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
      //ROS_INFO("INSIDE CALLBACK");
      //We create the values of reference for the first postion of our hand
      if (FIRST_VALUE)
        {
        dataLastHand_.palmpos.x=dataHand_.palmpos.x;
        dataLastHand_.palmpos.y=dataHand_.palmpos.y;
        dataLastHand_.palmpos.z=dataHand_.palmpos.z;
        FIRST_VALUE=0;
        Updifferencex=dataLastHand_.palmpos.x+10;
        Downdifferencex=dataLastHand_.palmpos.x-10;
        Updifferencez=dataLastHand_.palmpos.z+10;
        Downdifferencez=dataLastHand_.palmpos.z-20;
        Updifferencey=dataLastHand_.palmpos.y+20;
        Downdifferencey=dataLastHand_.palmpos.y-20;
        old_gripper_pose=DtA+DtAx*dataHand_.finger_distance;
        //ROS_INFO("ORIGINAL POSITION OF THE HAND SET TO \n X: %f\n  Y: %f\n Z: %f\n ",trajectory_hand.at(i).palmpos.x,trajectory_hand.at(i).palmpos.y,trajectory_hand.at(i).palmpos.z);
        
        //sleep(2);
        }
        else
        {
            //We get the distance between the finger and transform it into gripper distance
            //rot7=DtA+DtAx*dataHand_.finger_distance;
            //rot8=DtA+DtAx*dataHand_.finger_distance;
            gripper_pose=DtA+DtAx*dataHand_.finger_distance;
            joint_msg_leap=jointstate_;
            joint_msg_leap.position[7] = -gripper_pose/10;
            joint_msg_leap.position[6] = gripper_pose/10;
            if ((dataHand_.palmpos.x<Downdifferencex)||(dataHand_.palmpos.x>Updifferencex)||(dataHand_.palmpos.y<Downdifferencey)||(dataHand_.palmpos.y>Updifferencey)||(dataHand_.palmpos.z<Downdifferencez)||(dataHand_.palmpos.z>Updifferencez)||(gripper_pose>old_gripper_pose)||(gripper_pose<old_gripper_pose))
              {
                //q.setRPY(0,0,M_PI/2);//Fixed Position for testing
                q.setRPY(0,0,M_PI/2);//Fixed Position for testing
                pose.orientation.x = q.getAxis().getZ();//cambiado aposta
                pose.orientation.y = q.getAxis().getY();
                pose.orientation.z = q.getAxis().getX();//cambiado aposta
                pose.orientation.w = q.getW();
                //pose.orientation.w = ;
                //pose.orientation.z=1;
                //pose.orientation.y=0;
                //pose.orientation.x=0;
                //We need to send the correct axis to the robot. Currently they are rotated and x is z
                //ROS_INFO("VALUES OF THE QUATERNION SET TO \n X: %f\n  Y: %f\n Z: %f W: %f\n",pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
                pose.position.y +=(dataHand_.palmpos.x-dataLastHand_.palmpos.x) ;
                pose.position.z +=(dataHand_.palmpos.y-dataLastHand_.palmpos.y);
                if(pose.position.z>Uplimitez)
                pose.position.z=Uplimitez;
                pose.position.x +=(dataHand_.palmpos.z-dataLastHand_.palmpos.z);
                //Here we instantiate an autogenerated service class 
                srv.request.target = pose ;
                srv.request.gripper = gripper_pose;
                if (pclient->call(srv))
                {
                  //ROS_INFO("Ret: %d", (int)srv.response.ret);
                  dataLastHand_.palmpos.x=dataHand_.palmpos.x;
                  dataLastHand_.palmpos.y=dataHand_.palmpos.y;
                  dataLastHand_.palmpos.z=dataHand_.palmpos.z;
                  // Both limits for x,y,z to avoid small changes
                  Updifferencex=dataLastHand_.palmpos.x+10;//
                  Downdifferencex=dataLastHand_.palmpos.x-10;
                  Updifferencez=dataLastHand_.palmpos.z+10;
                  Downdifferencez=dataLastHand_.palmpos.z-20;
                  Updifferencey=dataLastHand_.palmpos.y+20;
                  Downdifferencey=dataLastHand_.palmpos.y-20;
                  old_pose=pose;
                  old_gripper_pose=gripper_pose;
                  ROS_INFO("response %f\n",srv.response.ret);
                }
                else
                {
                  ROS_ERROR("Position out of range");
                  pose=old_pose;
                } 
              }
         //We get the aswer of the service and publish it into the joint_msg_leap message (simulation)
         robo_pub.publish(joint_msg_leap);
         /*joint_msg_leap.position[0] = srv.response.joint1;
         joint_msg_leap.position[1] = srv.response.joint2;
         joint_msg_leap.position[2] = srv.response.joint3;
         joint_msg_leap.position[3] = srv.response.joint4;
         joint_msg_leap.position[4] = srv.response.joint5;
         joint_msg_leap.position[5] = srv.response.joint6;
         robo_pub.publish(joint_msg_leap);*/
        }
 //ROS_INFO("END EFFECTOR POSITION \n X: %f\n  Y: %f\n Z: %f\n", pose.position.x,pose.position.y,pose.position.z);
    
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
      void LeapMotionListener::setPointertoClient(ros::ServiceClient *pointerto_client)
  {
   pclient=pointerto_client;
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


int main(int argc, char *argv[])
{     
      
      //Initial position
      
      pose.position.x = 0;
      pose.position.y = 0;
      //Real Position
      pose.position.z = 375;
      old_pose=pose;
      //Simulation position
      //pose.position.z = 859.9;
              
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
      //joint_msg_leap.name[7]="base_joint_gripper_right";
      aux_enter=1;
      FIRST_VALUE=true;//Help knowing Initial Position of the hand
      int arm_trajectory_point=1;
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
      ros::NodeHandle node_handle("~");
      robo_pub = node_handle.advertise<sensor_msgs::JointState>("/joint_leap", 1);
      robo_sub = node_handle.subscribe("/joint_states", 1, joinstateCallback);
      ros::ServiceClient clientinit = node_handle.serviceClient<sensor_listener::InitHaltAPI>("/InitHaltAPI");
      sensor_listener::InitHaltAPI srvinit;
      // start a ROS spinning thread
      //ros::AsyncSpinner spinner(1);
      //we need this for leap
      ros::Rate r(10);
     
      ROS_INFO("PRESH LEFT PEDAL TO START");
      while((s=getchar())!= '1')      
        ROS_INFO("PRESH LEFT PEDAL TO START");
      
      //Enable this part to communicate with the real robot
      srvinit.request.command=command;
      
      if (clientinit.call(srvinit))
      {
      ROS_INFO("Init correct"); 
      /* SENSOR SUBSCRIBING */
      //LEAP MOTION
      
      ROS_INFO("SUBSCRIBING LEAPMOTION");
      ros::Subscriber leapsub = node_handle.subscribe("/leapmotion/data", 1, &LeapMotionListener::leapmotionCallback, &leapmotionlistener);

      ros::ServiceClient client = node_handle.serviceClient<sensor_listener::PositionAPICoordSpaceQuat>("/PositionAPICoordSpaceQuat");
      pointerto_client=&client;
      leapmotionlistener.setPointertoClient(pointerto_client);

      while(!CAPTURE_MOVEMENT==true)
      {
       
       ros::spinOnce();
       
      }
      leapsub.shutdown();
      ROS_INFO("CAPTURING POINTS FINISH");
      // End of Capturing Stage
      return 0;
      }
      else
      {
        ROS_INFO("Could not init");
      }
}


