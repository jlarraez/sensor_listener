#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "std_msgs/UInt8.h"
#include <sstream>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>
#include<stdio.h>
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

int main(int argc, char **argv)
{     
      ros::init(argc, argv,"pedal_security_exit");
      ros::NodeHandle node_handle;
      ros::ServiceClient client = node_handle.serviceClient<schunk_api::InitHaltAPI>("InitHaltAPI");
      schunk_api::InitHaltAPI srv;
      while((s=getchar())!= '2')      
        ROS_INFO("NOT STOP DETECTED");
      srv.request.command = "halt";
      //srv.request.b = atoll(argv[2]);
      if (client.call(srv))
      {
        ROS_INFO("Status: %d", (int)srv.response.status);
      }
      else
      {
        ROS_ERROR("Failed to call service InitHaltAPI");
        return 1;
      }


}
