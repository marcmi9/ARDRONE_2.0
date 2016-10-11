#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>                //read input from terminal
#include <termios.h>              //termios, TCSANOW, ECHO, ICANON
#include <sys/select.h>           //select and struct timeval, STDIN_FILENO and fd_set

#define SPEED 1

float LOOP_FREQ = 30;

int main(int argc, char **argv) {

  ros::init(argc, argv, "keyboardControl");
  ros::NodeHandle n;
  ros::Rate loop_rate(LOOP_FREQ);
  
  // Takeoff and land publishers.
  ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
  ros::Publisher land = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
  ros::Publisher controller = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  
  std_msgs::Empty empty_msg;
  geometry_msgs::Twist controlSignal;
  geometry_msgs::Twist zeroSignal;
  zeroSignal.linear.x = 0;
  zeroSignal.linear.y = 0;
  zeroSignal.linear.z = 0;
  zeroSignal.angular.x = 0;
  zeroSignal.angular.y = 0;
  zeroSignal.angular.z = 0;

  bool flying = false;
  char a;
  int res;


  fd_set readfds; //set of file descriptors
  struct timeval timeout; //timeout for waiting for an input in STDIN with getchar
  timeout.tv_sec = 0; //seconds
  timeout.tv_usec = 0; //microseconds


  //All this is to avoid pressing enter after introducing a character. Reference: http://stackoverflow.com/questions/1798511/how-to-avoid-press-enter-with-any-getchar
  static struct termios oldt, newt;

  /*tcgetattr gets the parameters of the current terminal
  STDIN_FILENO will tell tcgetattr that it should write the settings
  of stdin to oldt*/
  tcgetattr( STDIN_FILENO, &oldt);
  /*now the settings will be copied*/
  newt = oldt;

  /*ICANON normally takes care that one line at a time will be processed
  that means it will return if it sees a "\n" or an EOF or an EOL*/
  newt.c_lflag &= ~(ICANON | ECHO);

  /*Those new settings will be set to STDIN
  TCSANOW tells tcsetattr to change attributes immediately. */
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  
  float time = 0;
  float accel = 10;
  float elapsedTime = 1/LOOP_FREQ;

  while (ros::ok()) {
    controlSignal = zeroSignal;
    
    //set timeout:
    timeout.tv_sec = 0; //seconds
    timeout.tv_usec = 0; //microseconds
    //initialize set
    FD_ZERO(&readfds); //clear set
    FD_SET(STDIN_FILENO, &readfds); //add stdin to the set
  
    res = select(STDIN_FILENO+1, &readfds, NULL, NULL, &timeout);
    
    if (res > 0) { //res > 0 there is something to read. res == 0 timeout expired
      a = getchar();
      time += elapsedTime;
      switch (a) {
        case 'f': //Take off ('F' for fly, close to awsd)
        {
          if (!flying) {
            takeoff.publish(empty_msg);
            flying = true;
          }
          break;
        }
        
        case 'h': //Land ('H' sign as a landing platform, close to jikl)
        {
          if (flying) {
            land.publish(empty_msg);
            flying = false;
          }
          break;
        }
        
        case 'w': //Pitch forward
        {
          if (flying) {
            controlSignal.linear.x = accel*time;
          }
          break;
        }
        
        case 's': //Pitch backward
        {
          if (flying) {
            controlSignal.linear.x = -accel*time;
          }
          break;
        }
        
        case 'd': //Roll right
        {
          if (flying) {
            controlSignal.linear.y = -accel*time;
          }
          break;
        }
        
        case 'a': //Roll left
        {
          if (flying) {
            controlSignal.linear.y = accel*time;
          }
          break;
        }
        
        case 'l': //Yaw right
        {
          if (flying) {
            controlSignal.angular.z = -SPEED;
          }
          break;
        }
        
        case 'j': //Yaw left
        {
          if (flying) {
            controlSignal.angular.z = SPEED;
          }
          break;
        }
        
        case 'i': //Throttle up
        {
          if (flying) {
            controlSignal.linear.z = accel*time;
          }
          break;
        }
        
        case 'k': //Throttle down
        {
          if (flying) {
            controlSignal.linear.z = -accel*time;
          }
          break;
        }
        
        default: //do nothing
        break;
      }
    }
    
    if (res == 0) {
	time = 0;
    }
    
    if (res == -1) {
      ROS_INFO("ERROR with select!");
    }
    
    if (flying) {
      controller.publish(controlSignal);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  /*restore the old settings*/
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
  
  return 0;
}
