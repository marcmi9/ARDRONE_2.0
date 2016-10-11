#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "dynamic_reconfigure/server.h"
#include <Final_Project/takeoff_landConfig.h>

bool take_flag;
bool land_flag;

void callback(Final_Project::takeoff_landConfig &config, uint32_t level) {
  
  take_flag = config.takeoff;
  config.takeoff = 0;
  land_flag = config.land;
  config.land = 0;
  
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "ard");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  dynamic_reconfigure::Server<Final_Project::takeoff_landConfig> server;
  dynamic_reconfigure::Server<Final_Project::takeoff_landConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);

  
  
  ros::Publisher land = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);

  std_msgs::Empty empty_msg;
  bool var = 0;




  while (ros::ok()) {

    if(take_flag == 1) {
      takeoff.publish(empty_msg);
      take_flag = 0;
    }
    
    if(land_flag == 1) {
      land.publish(empty_msg);
      land_flag = 0;
    }
    
    
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
