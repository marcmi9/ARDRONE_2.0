#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "Final_Project/takeoff.h"
#include "ar_pose/ARMarker.h"
#include <ros/callback_queue.h> //to create our own callback queues
#include <stdlib.h>

ros::Publisher takeoff_pub;
float sonarHeight;
ros::CallbackQueue sonar_callbackQueue;

float change_rate;
float old_pos;
float ar_position[3];

float pos_x;
float pos_y;
float integral_x;
float integral_y;


bool takeoffCallback(Final_Project::takeoff::Request &req, Final_Project::takeoff::Response &res) {

  std_msgs::Empty empty_msg;
  takeoff_pub.publish(empty_msg);

  res.output = 1;
  return true;
}

/*
void sonarCallback(const sensor_msgs::Range::ConstPtr& msg) {
  float pos = msg->range;
  ROS_INFO("[%f], [%f]", pos, old_pos);
  change_rate = -(old_pos - pos);
  old_pos = pos;
}


void arposeCallback(const ar_pose::ARMarker::ConstPtr& msg) {

  ar_position[0] = msg->pose.pose.position.x;
  ar_position[1] = msg->pose.pose.position.y;
  ar_position[2] = msg->pose.pose.position.z;
}
*/

int main(int argc, char **argv) {

  ros::init(argc, argv, "takeoff_server");
  ros::NodeHandle nh;
  
  // Takeoff server
  ros::ServiceServer takeoff_server = nh.advertiseService("take_off", takeoffCallback);
  ROS_INFO("Ready to takeoff");

  // Takeoff publisher to /ardrone/takeoff
  takeoff_pub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);

  /*
  nh.setCallbackQueue(&sonar_callbackQueue);
  // Subscriber to /sonar_height. We set queue to 1 because we don't want to accumulate all messages that we don't read, we'll make sure we read them all by setting a proper frequency
  ros::Subscriber sonar_sub = nh.subscribe("/sonar_height", 1, sonarCallback);
  
  ros::Subscriber arpose_sub = nh.subscribe("/ar_pose_marker", 1000, arposeCallback); 
  */

  ros::spin();

  return 0;
}
