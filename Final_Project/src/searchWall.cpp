#include "ros/ros.h"
#include "ar_pose/ARMarker.h"
#include "geometry_msgs/Twist.h"
#include <Final_Project/searchwallAction.h>
#include <actionlib/server/simple_action_server.h>

bool detected;
ros::Publisher wallController;

void wallCallback(const Final_Project::searchwallGoalConstPtr &goal, actionlib::SimpleActionServer<Final_Project::searchwallAction>* as) {

  float speed = goal->rotationSpeed;
  int timeRot = goal->timeRot * 1000; //to ms
  int timeWait = goal->timeWait * 1000; //to ms
  geometry_msgs::Twist vel;
  bool succeed = true;
  detected = false;
  
  double begin = ros::Time::now().toSec();
  
  float freq = 20; //same frequency as the controller
  
  while(!detected) {
    
    // Check if the action has been cancelled or the node has been shut down
    if (as->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("SearchWall: Action cancelled.");
      as->setPreempted();
      succeed = false;
      break;
    }
    int now = (ros::Time::now().toSec() - begin) * 1000; //ms
    if (now % (timeRot + timeWait) < timeRot) { //rotating
      // Set desired pose
      vel.angular.z = speed;
      wallController.publish(vel);
    }else{
      wallController.publish(geometry_msgs::Twist()); //send zero
    }

    ros::Rate(freq).sleep();
  }
  
  wallController.publish(geometry_msgs::Twist()); //send zero

  if (succeed) {
    ROS_INFO("Search controller: Found the wall marker");
    as->setSucceeded();
  }
  
  // Stop rotating
  vel.angular.z = 0;
  wallController.publish(vel);
}

void arposeCallback(const ar_pose::ARMarker::ConstPtr& msg) {

  detected = true;

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "search_wall");
  ros::NodeHandle nh;
  
  // Subscriber to /ar_pose_front topic
  detected = false;
  ros::Subscriber arpose_sub = nh.subscribe("/ar_pose_front", 1, arposeCallback); 

  // Action server for the align controller
  actionlib::SimpleActionServer<Final_Project::searchwallAction> wallCont(nh, "search_wall",boost::bind(&wallCallback, _1, &wallCont),false);

  wallCont.start();

  // Publisher to /pose_ref
  wallController = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
  while(ros::ok()) {

    ros::spinOnce();
    ros::Rate(100).sleep();
  }

  return 0;
}
