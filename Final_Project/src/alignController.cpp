#include "ros/ros.h"
#include "ar_pose/ARMarker.h"
#include "tf/transform_listener.h"
#include <sensor_msgs/Range.h>
#include <Final_Project/aligningAction.h>
#include <actionlib/server/simple_action_server.h>
#include "ardrone_autonomy/Navdata.h"
#include "Final_Project/controlMsg.h"

//#define ALIGN_HEIGHT_REF 1.2

tf::Vector3 marker_position;

ros::Publisher controller;
tf::TransformListener* tfListener;
float height;
float freq = 50;
float markerLostTime = 0.5; //in seconds
float waitingTime = 2; //in seconds
double markerDetectedTime;

float maxHeight = 2;

double begin;

bool flag;

enum State {Aligning, Rising, Waiting};

State state;

void alignCallback(const Final_Project::aligningGoalConstPtr &goal, actionlib::SimpleActionServer<Final_Project::aligningAction>* as) {

  bool success = true;
  
  markerDetectedTime = ros::Time::now().toSec()*10; //set markerDetectedTime to a high number
  
  ros::Rate loop_rate(freq);
  // Set goal variables
  float goalTime = goal->aligningTime;
  float tolerance = goal->tolerance;
  float refHeight = goal->refHeight;
  
  int timeRise = 0.5 * 1000; //to ms
  int timeStop = 0.5 * 1000; //to ms
  double beginRise;

  // Action feedback and result publishers
  Final_Project::aligningFeedback feedback;
  Final_Project::aligningResult result;

  Final_Project::controlMsg control;
  control.mode = 1;
  geometry_msgs::Twist zero_signal;
  flag = false;

  int count = 0;  
  state = Waiting;
  
  while(count < goalTime*freq) {
    
    // Check if the goal has been cancelled or the node has been shut down (for example by a Ctrl-C)
    if (as->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("AlignController: Action cancelled.");
      as->setPreempted();
      success = false;
      break; //get out of the while loop
    }
    
    if (state == Aligning){
      ROS_ERROR("ALIGNING");
      // Set reference pose to align with floor mark being at a certain height
      control.error.linear.x = -marker_position.getY(); //markerFromDrone.x();
      control.error.linear.y = -marker_position.getX(); //markerFromDrone.y();
      control.error.linear.z = refHeight - height;
      
      // Calculate error. Error = (markerFromDrone - 0) as it is seen from the drone reference frame.
      float error = sqrt(pow(control.error.linear.x,2) + pow(control.error.linear.y,2));
          
      // Send feedback to the action client
      feedback.error = error;
      as->publishFeedback(feedback);
      
      // Check if we are aligned. If we are, update timer, otherwise restart it
      
      if (error < tolerance) {
        count++;
      } else {
        count = 0;
      }
      
      // Check if the marker has been lost
      if (ros::Time::now().toSec() - markerDetectedTime > markerLostTime) {
        state = Waiting;
        flag = false;
        begin = ros::Time::now().toSec();
      }
      
    } else if (state == Waiting) {
      ROS_ERROR("WAITING");
      control.error = zero_signal;
      if (ros::Time::now().toSec() - begin > waitingTime) {
        state = Rising;
        beginRise = ros::Time::now().toSec();
      }
      
      if (flag) {
        state = Aligning;
        count = 0;
      }
      
      
    } else {
      ROS_ERROR("RISING");
      control.error = zero_signal;
      int now = (ros::Time::now().toSec() - beginRise) * 1000; //ms
      if (now % (timeRise + timeStop) < timeRise) { //rising
        control.error.linear.z = maxHeight - height; //It is the same as "height". Should be the same as markerFromDrone.z();
      }

      if(flag) state = Aligning; //If we have seen the marker, align
      count = 0; //This count has to be zero when entering the aligning state
      
      if (height > maxHeight - 0.1) {
        ROS_ERROR("AlignController: Action aborted. MARKER HAS BEEN LOST: %f", ros::Time::now().toSec());
        as->setAborted();
        success = false;
        break;
      }
    }
    // Publish the control action
    controller.publish(control);

    loop_rate.sleep();
  }
  
  // Set control signal to 0
  control.error = zero_signal;
  controller.publish(control);

  if (success) {
    // Sending result to the action client
    result.error = feedback.error;
    ROS_INFO("Controller action: Finished aligning");
    as->setSucceeded(result);
  }
  
}

void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg) {

  height = msg->altd;
  height = height / 1000; //mm to m
  //ROS_WARN("HEIGHT: %f", height);
}

void arposeCallback(const ar_pose::ARMarker::ConstPtr& msg) {

  marker_position.setValue(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

  markerDetectedTime = ros::Time::now().toSec();
  
  ROS_ERROR("Marker position: (%f, %f, %f, %f)", marker_position.getX(), marker_position.getY(), marker_position.getZ(), markerDetectedTime);
  
  flag = true;
  
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "Align_controller");
  ros::NodeHandle nh;
  ros::Rate loop_rate(freq);
  
  // Transformation listener
  tfListener = new tf::TransformListener(); //we have defined it as a pointer because we need to instantiate it here. If we do it up there as a global variable we will get the error: couldn't find an AF_INET address for [].
  
  // Subscriber to /ar_pose topic
  ros::Subscriber arpose_sub = nh.subscribe("/ar_pose_bottom", 1, arposeCallback);
  
  // Subscriber to /ardrone/navdata topic
  ros::Subscriber navdata_sub = nh.subscribe("/ardrone/navdata", 1, navdataCallback); 

  // Action server for the align controller
  actionlib::SimpleActionServer<Final_Project::aligningAction> alignCont(nh, "align_controller",boost::bind(&alignCallback, _1, &alignCont),false);

  // Publisher to /ref_pose
  controller = nh.advertise<Final_Project::controlMsg>("/pose_ref", 1);

  alignCont.start();

  while(ros::ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }

  delete tfListener;

  return 0;
}
