#include "ros/ros.h"
#include "std_msgs/String.h"
#include "leap_motion/leapros2.h"
#include "leap_motion/leap2.h"
#include <sstream>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
leap_motion::leapros2 dataHand_;
leap_motion::leapros2 dataLastHand_;
float Uplimitex;
float Downlimitex;
float Uplimitey;
float Downlimitey;
void leapmotionCallback(const leap_motion::leapros2::ConstPtr& dataHand)
{
  dataHand_=(*dataHand);
  // Both limits for x,y,z to avoid small changes
  Uplimitex=dataLastHand_.palmpos.x+0.3;
  Downlimitex=dataLastHand_.palmpos.x-0.3;
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
  	case 1: ROS_INFO("SWIPE");
	case 4: ROS_INFO("KEY_TAP");
	case 5: ROS_INFO("CIRCLE");
 


  }
  //save new value of the last position of the hand
  dataLastHand_=(*dataHand);

  
  //we print the position of all the hand
  printf("Palmpos \n X: %f\n  Y: %f\n Z: %f\n ",dataHand_.palmpos.x,dataHand_.palmpos.y,dataHand_.palmpos.z);
  
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/leapmotion/data", 1000, leapmotionCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
