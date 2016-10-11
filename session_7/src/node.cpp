#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h" 
#include <dynamic_reconfigure/server.h>

#include <session_7/commConfig.h>
#include <math.h>


ros::Publisher publ; // Global publisher to publish inside the callback function.

double refer;

void callback(session_7::commConfig &config, uint32_t level) {
  
  refer = config.ref;

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "subs");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  
  dynamic_reconfigure::Server<session_7::commConfig> server;
  dynamic_reconfigure::Server<session_7::commConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  publ = n.advertise<std_msgs::Float64>("/arm/joint1_position_controller/command", 1000);
  
  while (ros::ok()) {

    std_msgs::Float64 mess;
    mess.data = refer;
    publ.publish(mess);

    ros::spinOnce();
    
    loop_rate.sleep();
  }
  return 0;
}
