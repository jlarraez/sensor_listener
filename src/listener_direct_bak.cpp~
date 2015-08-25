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
geometry_msgs::PoseStamped old_pose;
//waypoints
geometry_msgs::Pose pose;


//We define the 4x4 Matrix wit the orientation and the position


std::string command="init";
int s;
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


int main(int argc, char *argv[])
{     
      /*Initialise Variables*/
      
      //Configuring the terminal for the pedals  
     /* 
        static struct termios oldt, newt;

        /*tcgetattr gets the parameters of the current terminal
        STDIN_FILENO will tell tcgetattr that it should write the settings
        of stdin to oldt*/
    //    tcgetattr( STDIN_FILENO, &oldt);
        /*now the settings will be copied*/
     //   newt = oldt;

        /*ICANON normally takes care that one line at a time will be processed
        that means it will return if it sees a "\n" or an EOF or an EOL*/
     //   newt.c_lflag &= ~(ICANON| ECHO);          

        /*Those new settings will be set to STDIN
        TCSANOW tells tcsetattr to change attributes immediately. */
     //   tcsetattr( STDIN_FILENO, TCSANOW, &newt);
        
      //End configuring the terminal for the pedals
      
      /*Run a python function code*/
    /*  
    PyObject *pName, *pModule, *pDict, *pFunc;
    PyObject *pArgs, *pValue;
    long value_python=0;
    const char* PythonFile="kinematic_functions";
    //std::string PythonFunction="ikine";
    const char* PythonFunction="multiply";
    int i=0;
    */
    
      
      
      /* End run a python script*/
      
      //Initial position
      
      pose.position.x = 0;
      pose.position.y = 0;
      //See datasheet
      pose.position.z = 859.9;
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
      /*Finish Variables Initialitation*/

      //ROS DECLARATION
      ros::init(argc, argv,"listener");
      
      /*
      //Use a python Function in C++
      Py_Initialize();
      pName = PyString_FromString(PythonFile);

      pModule = PyImport_Import(pName);
      ROS_INFO("given name of file");
      //Py_DECREF(pName);

      if (pModule != NULL) {
          pFunc = PyObject_GetAttrString(pModule, PythonFunction);
          /* pFunc is a new reference */
      /*
          if (pFunc && PyCallable_Check(pFunc)) {
          
              ROS_INFO("Function exist");
              pArgs = PyTuple_New(2);
              
              value_python=3;
              pValue = PyInt_FromLong(value_python);
              ROS_INFO("first value given");
              //pValue="";
              if (!pValue) {
                  Py_DECREF(pArgs);
                  Py_DECREF(pModule);
                  fprintf(stderr, "Cannot convert argument 1\n");
                  return 1;
              }
              /* pValue reference stolen here: */
     /*         ROS_INFO("first value in Tuple");
              PyTuple_SetItem(pArgs, i, pValue);
              i++;
              pValue = PyInt_FromLong(value_python);
              ROS_INFO("second value given");
              if (!pValue) {
                  Py_DECREF(pArgs);
                  Py_DECREF(pModule);
                  fprintf(stderr, "Cannot convert argument 2\n");
                  return 1;
              }
              PyTuple_SetItem(pArgs, i, pValue);

              pValue = PyObject_CallObject(pFunc, pArgs);
              Py_DECREF(pArgs);
              if (pValue != NULL) {
                  printf("Result of call: %ld\n", PyInt_AsLong(pValue));
                  Py_DECREF(pValue);
              }
              else {
                  Py_DECREF(pFunc);
                  Py_DECREF(pModule);
                  PyErr_Print();
                  fprintf(stderr,"Call failed\n");
                  return 1;
              }
          }
          else {
              if (PyErr_Occurred())
                  PyErr_Print();
              fprintf(stderr, "Cannot find function \"%s\"\n", argv[2]);
          }
          Py_XDECREF(pFunc);
          Py_DECREF(pModule);
      }
      else {
          PyErr_Print();
          fprintf(stderr, "Failed to load \"%s\"\n", argv[1]);
          return 1;
      }
      Py_Finalize();
      */
      sensor_listener::PositionAPICoordSpaceQuat srv;
      sensor_listener::InitHaltAPI srv2;
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
      //spinner.start();
      //we need this for leap
      ros::Rate r(1);
      //robo_pub = n.advertise<sensor_msgs::JointState>("joint_leap", 100);
      
      //Creating a Robot Model
      //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      //robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
     
      ROS_INFO("PRESH LEFT PEDAL TO START");
      while((s=getchar())!= '1')      
        ROS_INFO("PRESH LEFT PEDAL TO START");
      /* SENSOR SUBSCRIBING */
      //LEAP MOTION
      ROS_INFO("SUBSCRIBING LEAPMOTION");
      ros::Subscriber leapsub = node_handle.subscribe("/leapmotion/data", 1000, &LeapMotionListener::leapmotionCallback, &leapmotionlistener);
      ros::ServiceClient client = node_handle.serviceClient<sensor_listener::PositionAPICoordSpaceQuat>("/PositionAPICoordSpaceQuat");
      ros::ServiceClient client2 = node_handle.serviceClient<sensor_listener::InitHaltAPI>("/InitHaltAPI");
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
        ROS_INFO("END EFFECTOR POSITION \n X: %f\n  Y: %f\n Z: %f\n", pose.position.x,pose.position.y,pose.position.z);
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
            ROS_INFO("TRYING TO ADD POINT %d TO TRAJECTORY",arm_trajectory_point);
            //pose.header.frame_id = "/odom_combined";
            pose.orientation.w = 1.0;
            /*pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.position.x=0;
            pose.position.y=0;
            pose.position.z=1.2;*/
            //pose.position.y +=(trajectory_hand.at(i).palmpos.x-dataLastHand_.palmpos.x)/500 ;
            //pose.position.z +=(trajectory_hand.at(i).palmpos.y-dataLastHand_.palmpos.y)/1000 ;
            pose.position.y +=(trajectory_hand.at(i).palmpos.x-dataLastHand_.palmpos.x) ;
            pose.position.z +=(trajectory_hand.at(i).palmpos.y-dataLastHand_.palmpos.y);
            if(pose.position.z>Uplimitez)
            pose.position.z=Uplimitez;
            pose.position.x +=(trajectory_hand.at(i).palmpos.z-dataLastHand_.palmpos.z);
            //ROS_INFO("END EFFECTOR POSITION \n X: %f\n  Y: %f\n Z: %f\n", pose.position.x,pose.position.y,pose.position.z);
            //ROS_INFO("Palmpos \n X: %f\n  Y: %f\n Z: %f\n ",trajectory_hand.at(i).palmpos.x,trajectory_hand.at(i).palmpos.y,trajectory_hand.at(i).palmpos.z);
            //Here we instantiate an autogenerated service class 
            srv.request.target = pose ;
             //ROS_INFO("END EFFECTOR POSITION WE SEND \n X: %f\n  Y: %f\n Z: %f\n", srv.request.target.position.x,srv.request.target.position.y,srv.request.target.position.z);
            ROS_INFO("1");
            //ROS_INFO("Response: exist %s",client2.getService());

            if (client.call(srv))
            {
              ROS_INFO("Ret: %d", (int)srv.response.ret);
            }
            else
            {
              ROS_ERROR("Position out of range");
              //return 1;
            }
            dataLastHand_.palmpos.x=trajectory_hand.at(i).palmpos.x;
            dataLastHand_.palmpos.y=trajectory_hand.at(i).palmpos.y;
            dataLastHand_.palmpos.z=trajectory_hand.at(i).palmpos.z;
          }
    
        }
          
      }
      
      
   //ros::Subscriber myogestsub = n.subscribe("/myo_gest", 1000, myogestCallback);
     /*restore the old settings*/
    //tcsetattr( STDIN_FILENO, TCSANOW, &oldt);   
      //while (true);
      return 0;
}


