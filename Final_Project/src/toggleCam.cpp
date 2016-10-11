#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <stdlib.h>
#include <sensor_msgs/Range.h>
#include <ros/callback_queue.h> //to create our own callback queues
#include "Final_Project/lander.h"
#include "Final_Project/camServ.h"

ros::ServiceClient toggleClient; // Service client for toggle cam
int count;

bool camFront;

bool camCallback(Final_Project::camServ::Request &req, Final_Project::camServ::Response &res) {

  std_srvs::Empty empty_srv;

  if(req.bottom == 1 && camFront){
    toggleClient.call(empty_srv);
    camFront =false;
  }
  else if(req.front == 1 && !camFront){
    toggleClient.call(empty_srv);
    camFront = true;
  }
  
  res.out = 1;
  
  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "toggle_cam");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  
  camFront = true;
  
  // Land server
  ros::ServiceServer toggle_cam = nh.advertiseService("cam_service", camCallback);
  ROS_INFO("Ready to toggle cam");

  // Client to toggle the cam
  toggleClient = nh.serviceClient<std_srvs::Empty>("/ardrone/togglecam");
  
  while(ros::ok()){

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
