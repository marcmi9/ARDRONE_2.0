/* Controller node
 * 
 * It gets the reference signal by reading periodically the topic
 * ref_pose. If there is a new reference, it computes the control 
 * signal and publishes it through cmd_vel.
 * 
 */

#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "Final_Project/controllerConfig.h"
#include "geometry_msgs/Twist.h"
#include "PIDcontroller.h"
#include <algorithm>
#include "Final_Project/controlMsg.h"

float controlFreq = 20;
float Ts = 1/controlFreq;
double u_max = 0.1;
double u_min = -0.1;
double u_max_z = 80;
double u_min_z = -80;
double yaw_max = 0.1;
double yaw_min = -0.1;

enum PIDcnt {
  Xaxis = 0, //values don't really need to be specified: http://stackoverflow.com/questions/404231/using-an-enum-as-an-array-index
  Yaxis = 1,
  Zaxis = 2,
  Yaw = 3
}; 

PIDcontroller pid_1[4];
PIDcontroller pid_2[4];
ros::Publisher controllerPub;
int count;

void dynrecCallback(Final_Project::controllerConfig &config, uint32_t level) {
  //Update pid constants
  pid_1[Xaxis].kp = config.P_XY_1;
  pid_1[Xaxis].ki = config.I_XY_1;
  pid_1[Xaxis].kd = config.D_XY_1;
  
  pid_1[Yaxis].kp = config.P_XY_1;
  pid_1[Yaxis].ki = 0; //config.I_XY_1;
  pid_1[Yaxis].kd = config.D_XY_1;

  pid_1[Zaxis].kp = config.P_Z_1;
  pid_1[Zaxis].ki = 0; //config.I_Z_1;
  pid_1[Zaxis].kd = 0; //config.D_Z_1;

  pid_1[Yaw].kp = config.P_Yaw_1;
  pid_1[Yaw].ki = 0; //config.I_Yaw_1;
  pid_1[Yaw].kd = config.D_Yaw_1;
  
  pid_2[Xaxis].kp = config.P_X_2;
  pid_2[Xaxis].ki = 0; //config.I_XY_2;
  pid_2[Xaxis].kd = config.D_X_2;
  
  pid_2[Yaxis].kp = config.P_Y_2;
  pid_2[Yaxis].ki = config.I_Y_2;
  pid_2[Yaxis].kd = config.D_Y_2;

  pid_2[Zaxis].kp = config.P_Z_2;
  pid_2[Zaxis].ki = 0; //config.I_Z_2;
  pid_2[Zaxis].kd = 0; //config.D_Z_2;

  pid_2[Yaw].kp = config.P_Yaw_2;
  pid_2[Yaw].ki = config.I_Yaw_2;
  pid_2[Yaw].kd = config.D_Yaw_2;
}

void poseRefCallback(const Final_Project::controlMsg::ConstPtr& msg) {
  
  PIDcontroller* pid;
  geometry_msgs::Twist errorPose = msg->error; //error = ref - 0 (viewed from drone reference frame)
  geometry_msgs::Twist controlSignal;
  
  if (msg->mode == 1) {
    pid = pid_1;
  } else {
    pid = pid_2;
  }
  
  //If we haven't controlled for a while... Reset PIDs:
  if (count > 10) {
    for (int i = 0; i < 4; i++) {
      pid[i].Reset();
    }
    count = 1;
  }
  
  float elapsedTime = count*Ts;

  // Generate control signal from reference: u = pid.Update(error, Ts), and error = ref - 0 because it is from the drone's reference frame.
  controlSignal.linear.x = std::min(u_max,std::max(u_min,pid[Xaxis].Update(errorPose.linear.x, elapsedTime)));
  controlSignal.linear.y = std::min(u_max,std::max(u_min,pid[Yaxis].Update(errorPose.linear.y, elapsedTime)));
  controlSignal.linear.z = std::min(u_max_z,std::max(u_min_z,pid[Zaxis].Update(errorPose.linear.z, elapsedTime)));
  controlSignal.angular.z = std::min(yaw_max,std::max(yaw_min,pid[Yaw].Update(errorPose.angular.z, elapsedTime)));

  ROS_WARN("%f %f %f %f", controlSignal.linear.x, controlSignal.linear.y, controlSignal.linear.z, controlSignal.angular.z);

  // Publish the control action
  controllerPub.publish(controlSignal);
  
  count = 0; //Reset timer
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Rate loop_rate(controlFreq);

  // For dynamic reconfigure.
  dynamic_reconfigure::Server<Final_Project::controllerConfig> dynrec_server;
  dynamic_reconfigure::Server<Final_Project::controllerConfig>::CallbackType f;
  f = boost::bind(&dynrecCallback, _1, _2);
  dynrec_server.setCallback(f);

  // Subscriber to pose reference topic.
  ros::Subscriber poseRefSub = n.subscribe("/pose_ref", 1, poseRefCallback); //we just want the last reference
  
  // Publisher to cmd_vel
  controllerPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  //Time steps counter
  count = 0;
  
  while (ros::ok()) {
    
    // Get reference and compute control signal
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
}
