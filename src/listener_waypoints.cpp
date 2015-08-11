#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "leap_motion/leapros2.h"
#include "leap_motion/leap2.h"
#include <sstream>
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
geometry_msgs::PoseStamped new_pose;





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
int FIRST_VALUE=0;
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

//waypoints
std::vector<geometry_msgs::Pose> waypoints;



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
  const float Uplimitez=1.32;
  const float Downlimitez=1.0;
  float Downdifferencez;
  float Downdifferencey;
  float Downdifferencex;
  float Updifferencez;
  float Updifferencey;
  float Updifferencex;
  
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

  
public:
   void leapmotionCallback(const leap_motion::leapros2::ConstPtr& dataHand);
   void setPointertoGroup(moveit::planning_interface::MoveGroup *pointertogroup);
   void setPointertoPlan(moveit::planning_interface::MoveGroup::Plan *pointertoplan);
   void setPointertoPipe(planning_pipeline::PlanningPipeline *pointertoppipeline);
   void setPointertoScene(planning_scene::PlanningScenePtr *pointertopscene);
   void setPointertoPublisher(ros::Publisher *pointertod_publisher);
   void Configure();
   LeapMotionListener();
   
   leap_motion::leapros2 dataHand_;
   leap_motion::leapros2 dataLastHand_;
   
};
 
 LeapMotionListener::LeapMotionListener(void)
{     
      counter=7;
      
  
      
} 

  void LeapMotionListener::Configure(void)
{
      
    
}
  void LeapMotionListener::leapmotionCallback(const leap_motion::leapros2::ConstPtr& dataHand)
{
      dataHand_=(*dataHand);

      //joint_msg_leap.header.stamp = ros::Time::now();
      
      /*We will take a first measurement of the hand as reference*/
      /*We will use this first measurement as our origin of the hand and First dataLastHand*/
      if (FIRST_VALUE)
      {
      dataLastHand_.palmpos.x=dataHand_.palmpos.x;
      dataLastHand_.palmpos.y=dataHand_.palmpos.x;
      dataLastHand_.palmpos.z=dataHand_.palmpos.x;
      FIRST_VALUE=0;
      ROS_INFO("ORIGINAL POSITION OF THE HAND SET TO \n X: %f\n  Y: %f\n Z: %f\n ",dataHand_.palmpos.x,dataHand_.palmpos.y,dataHand_.palmpos.z);
      sleep(2);
      }
      else
      {
      robot_state::RobotState& robot_state = planningscene->getCurrentStateNonConst();
      // Both limits for x,y,z to avoid small changes
      Updifferencex=dataLastHand_.palmpos.x+10;
      Downdifferencex=dataLastHand_.palmpos.x-10;
      Updifferencez=dataLastHand_.palmpos.z+10;
      Downdifferencez=dataLastHand_.palmpos.z-20;
      Updifferencey=dataLastHand_.palmpos.y+20;
      Downdifferencey=dataLastHand_.palmpos.y-20;
      if (counter>0)
      {
      if ((dataHand_.palmpos.x<Downdifferencex)||(dataHand_.palmpos.x>Updifferencex)||(dataHand_.palmpos.y<Downdifferencey)||(dataHand_.palmpos.y>Updifferencey)||(dataHand_.palmpos.z<Downdifferencez)||(dataHand_.palmpos.z>Updifferencez))
        {
         
            ros::AsyncSpinner spinner(1);
            spinner.start();
            ROS_INFO("ADDING POINT %d TO TRAJECTORY",counter);
            // Cartesian Paths
            // ^^^^^^^^^^^^^^^
            // You can plan a cartesian path directly by specifying a list of waypoints
            // for the end-effector to go through. Note that we are starting
            // from the new start state above. The initial pose (start state) does not
            // need to be added to the waypoint list.
            new_pose.header.frame_id = "/odom_combined";
            new_pose.pose.orientation.w = 1.0;
            new_pose.pose.position.y +=(dataHand_.palmpos.x-dataLastHand_.palmpos.x)/500 ;
            new_pose.pose.position.z +=(dataHand_.palmpos.y-dataLastHand_.palmpos.y)/1000 ;
            if(new_pose.pose.position.z>Uplimitez)
            new_pose.pose.position.z=Uplimitez;
            new_pose.pose.position.x +=-(dataHand_.palmpos.z-dataLastHand_.palmpos.z)/500 ;
            ROS_INFO("END EFFECTOR POSITION \n X: %f\n  Y: %f\n Z: %f\n", new_pose.pose.position.x,new_pose.pose.position.y,new_pose.pose.position.z);
            ROS_INFO("Palmpos \n X: %f\n  Y: %f\n Z: %f\n ",dataHand_.palmpos.x,dataHand_.palmpos.y,dataHand_.palmpos.z);
            //new_pose.pose.position.y +=0.05;
            //new_pose.pose.position.z -=0.02;
            //new_pose.pose.position.x +=-0.05;
            std::vector<double> tolerance_pose(3, 0.01);
            std::vector<double> tolerance_angle(3, 0.01);
            // We will create the request as a constraint using a helper function available
            req.group_name = "arm";
            ROS_INFO("1");
            pose_goal= kinematic_constraints::constructGoalConstraints("gripper_base_link", new_pose, tolerance_pose, tolerance_angle);
            ROS_INFO("2");
            
            req.goal_constraints.push_back(pose_goal);
            
            // Now, call the pipeline and check whether planning was successful.
            pplanningpipeline->generatePlan(*pplanningscene, req, res);
            ROS_INFO("3");
            if(res.error_code_.val != res.error_code_.SUCCESS)
            {
            ROS_ERROR("Could not compute plan successfully");
            new_pose=old_pose;
            }
            else
            {
            
            counter--;
            // Visualize the trajectory 
            ROS_INFO("VISUALIZING NEW POINT");
            //req.goal_constraints.clear();
            res.getMessage(response);
            display_trajectory.trajectory_start = response.trajectory_start;
            display_trajectory.trajectory.push_back(response.trajectory); 
            // Now you should see two planned trajectories in series
            pdisplay_publisher->publish(display_trajectory);
            planningscene->setCurrentState(response.trajectory_start);
            robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
            req.goal_constraints.clear();
            old_pose=new_pose;
            }
            //sleep(2);
            spinner.stop();     
           }
           
          }
        else
        {
          
          printf("Palmpos \n X: %f\n  Y: %f\n Z: %f\n ",dataHand_.palmpos.x,dataHand_.palmpos.y,dataHand_.palmpos.z);
        }
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
      //Initialising variables
      
      //ROS DECLARATION
      ros::init(argc, argv,"listener");
      //ros::NodeHandle node_handle;
      ros::NodeHandle node_handle("~");
      // start a ROS spinning thread
      ros::AsyncSpinner spinner(1);
      spinner.start();
      //we need this for leap
      ros::Rate r(1);
      //robo_pub = n.advertise<sensor_msgs::JointState>("joint_leap", 100);
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
      
      FIRST_VALUE=1;
      //Creating a Robot Model
      robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      //robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
      robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

      /* This sleep is ONLY to allow Rviz to come up */
      sleep(2.0);
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
      //ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
      //publish joint state
      //ros::Publisher moveit_joint_publisher = n.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1, true);
      //Planning_scene Tutorial
      //planning_scene::PlanningScene planning_scene(kinematic_model);
      planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
      pointertopscene=&planning_scene;
      //pointertopscene1=planning_scene;
      leapmotionlistener.setPointertoScene(pointertopscene);
      planning_pipeline::PlanningPipeline *planning_pipeline= new planning_pipeline::PlanningPipeline(robot_model,node_handle,"planning_plugin", "request_adapters");
      pointertoppipeline=planning_pipeline;
      leapmotionlistener.setPointertoPipe(pointertoppipeline);
        /* Sleep a little to allow time to startup rviz, etc. */
      ros::WallDuration sleep_time(20.0);
      sleep_time.sleep();
       
        
      /*end of MOVEIT Setup*/
      //ROS Subscription
      //We have to create the JointState msg
      // We can print the name of the reference frame for this robot.
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      // We can also print the name of the end-effector link for this group.
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
      
      // Planning to a Pose goal 1
        // ^^^^^^^^^^^^^^^^^^^^^^^
        // We can plan a motion for this group to a desired pose for the 
        // end-effector  
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
        new_pose=pose;
        old_pose=pose;
        // We will create the request as a constraint using a helper function available
        req.group_name = "arm";
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("gripper_base_link", pose, tolerance_pose, tolerance_angle);
        req.goal_constraints.push_back(pose_goal);
        // Now, call the pipeline and check whether planning was successful.
        pointertoppipeline->generatePlan(planning_scene, req, res);
        /* Check that the planning was successful */
        if(res.error_code_.val != res.error_code_.SUCCESS)
        {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
        }
        // Visualize the result
        // ^^^^^^^^^^^^^^^^^^^^
        ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        pointertod_publisher=&display_publisher;
        leapmotionlistener.setPointertoPublisher(pointertod_publisher);
        
        moveit_msgs::DisplayTrajectory display_trajectory;
        /* Visualize the trajectory */
        ROS_INFO("Visualizing the trajectory 1");
        moveit_msgs::MotionPlanResponse response;
        res.getMessage(response);
        display_trajectory.trajectory_start = response.trajectory_start;
        display_trajectory.trajectory.push_back(response.trajectory);
        display_publisher.publish(display_trajectory);
        //sleep_time.sleep();
        
     /* End Planning to a Pose goal 1*/
     
     // Planning to a Pose goal 2
        // ^^^^^^^^^^^^^^^^^^^^^^^
     
     // First, set the state in the planning scene to the final state of the last plan 
      robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
      planning_scene->setCurrentState(response.trajectory_start);
      joint_model_group = robot_state.getJointModelGroup("arm");
      robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
      
        // Check that the planning was successful 
        /*planning_interface::MotionPlanRequest req2;
        planning_interface::MotionPlanResponse res2;
        robot_state::RobotState goal_state(robot_model);
        std::vector<double> joint_values(8, 0.0);
        joint_values[0] = 0;
        joint_values[1] = 0;
        joint_values[2] = 1.5;
        joint_values[3] = 0;
        joint_values[5] = 0;
        joint_values[6] = 0;
        joint_values[7] = 0;
        goal_state.setJointGroupPositions(joint_model_group, joint_values);
        //new_pose=pose;
        // We will create the request as a constraint using a helper function available
        req2.group_name = "arm";
        moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
        //pose_goal = kinematic_constraints::constructGoalConstraints("gripper_base_link", pose, tolerance_pose, tolerance_angle);
        req2.goal_constraints.clear();
        //req2.goal_constraints.push_back(pose_goal);
        req2.goal_constraints.push_back(joint_goal);
        // Now, call the pipeline and check whether planning was successful.
        pointertoppipeline->generatePlan(planning_scene, req2, res2);
        if(res2.error_code_.val != res2.error_code_.SUCCESS)
        {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
        }
        // Visualize the trajectory 
        ROS_INFO("Visualizing the trajectory 2");
        res2.getMessage(response);
        display_trajectory.trajectory_start = response.trajectory_start;
        display_trajectory.trajectory.push_back(response.trajectory);
        // Now you should see two planned trajectories in series
        display_publisher.publish(display_trajectory);
        
        //sleep_time.sleep();
        //We change again the start pose
        new_pose=pose;
        planning_scene->setCurrentState(response.trajectory_start);
        robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
        //req2.goal_constraints.clear();
        req.goal_constraints.clear();*/

      spinner.stop();
      
      /* SENSOR SUBSCRIBING */
      //LEAP MOTION
      ROS_INFO("Subscribing Leap Motion");
      ros::Subscriber leapsub = node_handle.subscribe("/leapmotion/data", 1000, &LeapMotionListener::leapmotionCallback, &leapmotionlistener);
      //MYO
      //ros::Subscriber myogestsub = n.subscribe("/myo_gest", 1000, myogestCallback);
      ros::spin();
      return 0;
}


