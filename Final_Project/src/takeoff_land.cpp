#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "ard");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.25);

  ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);

  
  
  ros::Publisher land = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);

  std_msgs::Empty flag;
  bool var = 0;

  while (ros::ok()) {

    if(var == 0) {
      takeoff.publish(flag);
      var = 1;
    }
    
    else if(var == 1) {
      land.publish(flag);
      var = 0;
    }
    
    
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
