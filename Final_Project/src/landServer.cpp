#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <stdlib.h>
#include <sensor_msgs/Range.h>
#include "ardrone_autonomy/Navdata.h" // To measure height with the real drone!
#include <ros/callback_queue.h> //to create our own callback queues
#include "Final_Project/lander.h"

ros::Publisher land_pub; //global in order to publish from landCallback
ros::CallbackQueue sonar_callbackQueue; //global in order to read sonar topic from landCallback
float sonarHeight = 10; 

bool landCallback(Final_Project::lander::Request &req, Final_Project::lander::Response &res) {

  std_msgs::Empty empty_msg;
  land_pub.publish(empty_msg);
  
  sonar_callbackQueue.callAvailable();
  while (sonarHeight>0.1) {
    ros::Rate(5).sleep(); //sonar loop rate is 5Hz
    sonar_callbackQueue.callAvailable();
  }

  return true;
}

void sonarCallback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
  sonarHeight = msg->altd;
  sonarHeight = sonarHeight / 1000; // From mm to m
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "land_server");
  ros::NodeHandle nh;
  
  // Land server
  ros::ServiceServer land_server = nh.advertiseService("land", landCallback);
  ROS_INFO("Ready to land");

  // Publisher to /ardrone/land
  land_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1000);

  // Change queue for callbacks added from here on
  nh.setCallbackQueue(&sonar_callbackQueue);
  // Subscriber to /sonar_height. We set queue to 1 because we don't want to accumulate all messages that we don't read, we'll make sure we read them all by setting a proper frequency
  ros::Subscriber sonar_sub = nh.subscribe("/ardrone/navdata", 1, sonarCallback);
  
  ros::spin();

  return 0;
}
