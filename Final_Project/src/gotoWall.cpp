#include "ros/ros.h"
#include "ar_pose/ARMarker.h"
#include "tf/transform_listener.h"
#include <Final_Project/gotowallAction.h>
#include <actionlib/server/simple_action_server.h>
#include "LowPass.h"
#include "AverageFilter.h"
#include "Final_Project/controlMsg.h"

#define PI 3.141592653

ros::Publisher wallController;

float marker_position[3];
float px, py, pz;
double roll, pitch, yaw;
double yaw_error;
LowPass filterX;
LowPass filterY;
LowPass filterZ;

AverageFilter filterYaw(5);

float freq = 20;

void wallCallback(const Final_Project::gotowallGoalConstPtr &goal, actionlib::SimpleActionServer<Final_Project::gotowallAction>* as) {

  Final_Project::gotowallFeedback feedback;
  Final_Project::gotowallResult result;

  geometry_msgs::Vector3 lin_signal;
  geometry_msgs::Vector3 ang_signal;
  Final_Project::controlMsg control;
  control.mode = 2;
  geometry_msgs::Twist err_signal;

  bool success = true;

  while(true) {

    if (as->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("GoToWall: Action cancelled.");
      as->setPreempted();
      success = false;
      break; //get out of the while loop
    }

    // Control algorithm here:

    // From cam to controller axis:
    
    //float err_angle = (0 - px);
    float err_angle = (0 - yaw_error);
    float err_z = -py;
    //float err_y = angle;
    float err_y = (0 - px);
    float err_x = (pz - 1.2); // To stay at 1m

    lin_signal.x = err_x;
    lin_signal.y = err_y;
    lin_signal.z = err_z; // To stay at the same height as the marker.
    ang_signal.z = err_angle;

    err_signal.linear = lin_signal;
    err_signal.angular = ang_signal;
    control.error = err_signal;
    wallController.publish(control);

    ros::Rate(freq).sleep(); // Changed Ts from 0.01 to 0.1

  }
  
  wallController.publish(Final_Project::controlMsg()); //send zero

  if (success) {
    //ROS_INFO("Some info message to send");
    as->setSucceeded(result);
  }
  
}

void arposeCallback(const ar_pose::ARMarker::ConstPtr& msg) {

  // Obtain the right position and orientation for the marker

  px = filterX.Update(msg->pose.pose.position.x);
  py = filterY.Update(msg->pose.pose.position.y);
  pz = filterZ.Update(msg->pose.pose.position.z);
  
  // Convert quaternion to RPY
  bool is_marker_ok = true;
  tf::Quaternion qt;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, qt);
  tf::Matrix3x3 matrix(qt);
  matrix.getRPY(roll,pitch,yaw);
  
  if(std::isnan(roll) || !std::isfinite(roll) || (msg->pose.pose.position.z<0)) {
    is_marker_ok = false;
  }
  
  if (is_marker_ok) {
    tf::Vector3 z_marker = matrix.getColumn(2);
    
    // Correction of marker orientation
    double rotx;
    tf::Matrix3x3 rotation;
    if ( z_marker.getY() < 0) {
      rotx = 30*PI/180;
    } else {
      rotx = -30*PI/180;
    }
    rotation.setRPY(rotx,0,0);
    matrix = rotation * matrix;
    
    z_marker = matrix.getColumn(2); //matrix has changed
    
    yaw_error = -z_marker.getX();
    
    ROS_WARN("angle: %f", yaw_error);

    //ROS_INFO("angle: %f, px: %f, py: %f, pz: %f", angle, px, py, pz);


  } 
  else {
    ROS_WARN("Could not convert properly from quaternion");
  }

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "search_wall");
  ros::NodeHandle nh;
  
  // Subscriber to /ar_pose_front topic
  ros::Subscriber arpose_sub = nh.subscribe("/ar_pose_front", 1, arposeCallback); 

  // Action server for the align controller
  actionlib::SimpleActionServer<Final_Project::gotowallAction> wallCont(nh, "goto_wall",boost::bind(&wallCallback, _1, &wallCont),false);
  wallCont.start();

  // Publisher to /pose_ref
  wallController = nh.advertise<Final_Project::controlMsg>("/pose_ref", 1);
  
  while(ros::ok()) {

    ros::spinOnce();
    ros::Rate(freq).sleep(); // Changed Ts from 0.01 to 0.1
  }
  
  //ros::spin();

  return 0;
}
