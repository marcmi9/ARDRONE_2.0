#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ar_pose/ARMarker.h"
#include <dynamic_reconfigure/server.h>
#include <session_6/paramsConfig.h>
#include <math.h>
#include "session_6/mess.h"


double xd;
double yd;
double zd;


ros::Publisher pub; // Global publisher to publish inside the callback function.

void callback(const ar_pose::ARMarker::ConstPtr& msg) {
  
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  float z = msg->pose.pose.position.z;

  float posit[3] = {x,y,z};
  float targ[3] = {xd,yd,zd};

  float error = sqrt(pow(xd-x,2)+pow(yd-y,2)+pow(zd-z,2));

  // Displays in the console the position and the error
  ROS_INFO("You are in: [%f, %f, %f]", x, y, z);
  ROS_INFO("Euclidean error: %f", error);

  session_6::mess message;
 
  for (int i = 0; i < sizeof(posit)/sizeof(*posit); i++) {
    message.current_position[i] = posit[i];
  }
  for (int i = 0; i < sizeof(targ)/sizeof(*targ); i++) {
    message.target_position[i] = targ[i];
  }

  message.Euclidean_error = error;
  pub.publish(message); // Publishes the message to the publisher topic.

}

void callback2(session_6::paramsConfig &config, uint32_t level) {
  xd = config.x_des;
  yd = config.y_des;
  zd = config.z_des;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "subs");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  dynamic_reconfigure::Server<session_6::paramsConfig> server;
  dynamic_reconfigure::Server<session_6::paramsConfig>::CallbackType f;
  f = boost::bind(&callback2, _1, _2);
  server.setCallback(f);

  ros::Subscriber sub = n.subscribe("/ar_pose_marker", 1000, callback);

  pub = n.advertise<session_6::mess>("position_error", 1000);

  while (ros::ok()) {

    ros::spinOnce();
    
    loop_rate.sleep();
  }
  return 0;
}
