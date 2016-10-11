#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "dynamic_reconfigure/server.h"
#include <Final_Project/takeoff_landConfig.h>
#include "Final_Project/takeoff.h"
#include "Final_Project/lander.h"
#include <actionlib/client/simple_action_client.h>
#include <Final_Project/aligningAction.h>
#include <Final_Project/gotowallAction.h>
#include <Final_Project/searchwallAction.h>
#include "Final_Project/camServ.h"
#include "geometry_msgs/Twist.h"


enum State {Landed, Takeoff, Landing, AlignFloorMark, AlignWallMark, SearchWallMark};

State currentState;
State previousState;
bool take_flag;
bool land_flag;
double time_cont;
double time_take;
double alignHeight;
float searchWaitTime;
float searchRotTime;
double rotSpeed;

ros::ServiceClient camClient; // Service client for toggle cam
Final_Project::camServ cam_serv;

void callback(Final_Project::takeoff_landConfig &config, uint32_t level) {
  
  take_flag = config.takeoff;
  config.takeoff = 0;
  land_flag = config.land;
  config.land = 0;

  if(config.bottom == 1){ // Directly toggles cam
    config.bottom = 0;
    cam_serv.request.bottom = 1;
    cam_serv.request.front = 0;
    bool res = camClient.call(cam_serv);
    if(!res) ROS_ERROR("Can't connect bottom");
  }

  if(config.front == 1){ // Directly toggles cam
    config.front = 0;
    cam_serv.request.bottom = 0;
    cam_serv.request.front = 1;
    bool res = camClient.call(cam_serv);
    if(!res) ROS_ERROR("Can't connect front");
  }

  time_cont = config.time_align;
  time_take = config.time_take;
  alignHeight = config.alignHeight;
  rotSpeed = config.rotSpeed;
  searchRotTime = config.searchRotTime;
  searchWaitTime = config.searchWaitTime;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "stateMachine");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  
  // Initializing states
  currentState = Landed;
  previousState = Landed;
  
  // Initializing flags
  take_flag = false;
  land_flag = false;

  // Initialize counter for alignController
  double begin;
  

  // For dynamic reconfigure (landing and takeoff).
  dynamic_reconfigure::Server<Final_Project::takeoff_landConfig> server;
  dynamic_reconfigure::Server<Final_Project::takeoff_landConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  
  // Client service for takeoff.
  ros::ServiceClient takeoff_client = n.serviceClient<Final_Project::takeoff>("take_off");
  Final_Project::takeoff takeoff_serv;
  
  // Client service for landing.
  ros::ServiceClient land_client = n.serviceClient<Final_Project::lander>("land");
  Final_Project::takeoff land_serv;

  // Client action aligning
  actionlib::SimpleActionClient<Final_Project::aligningAction> ac("align_controller", true);

  // Client action search wall mark
  actionlib::SimpleActionClient<Final_Project::searchwallAction> searchWallClient("search_wall", true);

  // Client action go to the wall mark
  actionlib::SimpleActionClient<Final_Project::gotowallAction> goToWallClient("goto_wall", true);

  // Client service for togglecam Server
  camClient = n.serviceClient<Final_Project::camServ>("cam_service");
  
  ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ROS_INFO("Waiting for aligning controller");
  ac.waitForServer(); 
  ROS_INFO("Connected to aligning controller");
  
  ROS_INFO("Waiting for search_wall_controller");
  searchWallClient.waitForServer(); 
  ROS_INFO("Connected to search_wall_controller");
  
  ROS_INFO("Waiting for goto_wall_controller");
  goToWallClient.waitForServer(); 
  ROS_INFO("Connected to goto_wall_controller");

  while (ros::ok()) {
    switch (currentState) {
      case Landed:
      {
        if (currentState != previousState) {
          // Entering state. Initialization.
          ROS_INFO("Landed!");
        }
        previousState = currentState;

        //Checking state exit conditions.
        if (take_flag) {
          currentState = Takeoff;
          take_flag = false;
        }
        
        if (land_flag) {
          ROS_WARN("I want to land!");
          currentState = Landing;
          land_flag = false;
        }
        break;
      }

      case Takeoff:
      {
        if(currentState != previousState) {
          previousState = currentState;
          
          geometry_msgs::Twist zero_signal;
          cmd_vel.publish(zero_signal); //set velocity reference to zero
          
          takeoff_serv.request.input = 1;
          bool res = takeoff_client.call(takeoff_serv);
          if (res) {
            ROS_INFO("Taking off!");
          } else {
            ROS_ERROR("Couldn't take off.");
          }
            begin = ros::Time::now().toSec();
        }

        // Checking state exit conditions.
        double now = ros::Time::now().toSec();
        if (now - begin > time_take) {
          currentState = AlignFloorMark;
        }
        if (land_flag) {
          ROS_WARN("I want to land!");
          currentState = Landing;
          land_flag = false;
        }
        
        break;
      }

      case AlignFloorMark: 
      {
        if (currentState != previousState) {
          // Sets the bottom camera.
          cam_serv.request.bottom = 1;
	      cam_serv.request.front = 0;
	      camClient.call(cam_serv);
          
          Final_Project::aligningGoal goal;
          goal.aligningTime = time_cont; //in seconds
          goal.tolerance = 800; //radius around the marker which we consider we are aligned
	      goal.refHeight = alignHeight;

          ac.sendGoal(goal);

          // Waits for 5 seconds to get a result.
          ROS_INFO("Waiting for the aligning controller to finish.");
        }
        previousState = currentState;
        
        // Checking state exit conditions.
        actionlib::SimpleClientGoalState state = ac.getState();
	
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_WARN("stateMachine: Finished aligning");
          currentState = SearchWallMark;
        }	
        
        if (state == actionlib::SimpleClientGoalState::ABORTED) {
          ROS_ERROR("stateMachine: MARKER LOST, ALIGNING ABORTED. LANDING");
          currentState = Landing;
        }	

        if (land_flag) {
          ROS_WARN("I want to land!");
          currentState = Landing;
          land_flag = false;
        }
        
        if (currentState != previousState) {
          ac.cancelGoal();
        }

        break;
      }

      case SearchWallMark:
      {
        if (currentState != previousState) {
          // Sets the front camera
		  cam_serv.request.bottom = 0;
		  cam_serv.request.front = 1;
		  camClient.call(cam_serv);

          Final_Project::searchwallGoal goal;
          goal.rotationSpeed = rotSpeed;
          goal.timeRot = searchRotTime;
          goal.timeWait = searchWaitTime;

          searchWallClient.sendGoal(goal);

          ROS_INFO("Waiting for the searching_wall_controller to finish.");
        }
        previousState = currentState;

        actionlib::SimpleClientGoalState state = searchWallClient.getState();

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("stateMachine: Found the wall marker");
          currentState = AlignWallMark;
        }

        if (land_flag) {
          currentState = Landing;
          land_flag = false;
        }

        if (currentState != previousState) {
          // Leaving state.
          searchWallClient.cancelGoal();          
        }

        break;
      }

      case AlignWallMark: // Will go to the wall marker and remain 1m in front
      {
        if (currentState != previousState) {
		  // Sets the front camera
		  cam_serv.request.bottom = 0;
		  cam_serv.request.front = 1;
		  camClient.call(cam_serv);
			
			
          Final_Project::gotowallGoal goal;

          goToWallClient.sendGoal(goal);

          ROS_INFO("Waiting for the goto_wall_controller to finish.");

        }
        previousState = currentState;

        actionlib::SimpleClientGoalState state = goToWallClient.getState();

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("stateMachine: In front of marker");
          currentState = Landing;
        }
              
        // Checking state exit conditions.
        if (land_flag) {
          currentState = Landing;
          land_flag = false;
        }
        
        if (currentState != previousState) {
          // Leaving state.
          goToWallClient.cancelGoal();
        }
        break;
      }

      case Landing:
      {
        previousState = currentState;
        
        // State action.
        ROS_INFO("Landing!");
        bool res = land_client.call(land_serv);
        if (!res) {
          ROS_ERROR("Landing service returned false.");
        }
        
        // Checking state exit conditions.
        if (res) {
          currentState = Landed;
        }
        break;
      }

      /*//Sample state code:
      case newState:
      {
        if (currentState != previousState) {
          // Entering state. Initialization.

        }
        previousState = currentState;
        
        // State action.
        //do something
        
        // Checking state exit conditions.
        if (true) {
          //currentState = anotherState;
        }
        
        if (currentState != previousState) {
          // Leaving state.
          
        }
        break;
      }*/

      default:
      {
        ROS_ERROR("ERROR! Default state. Shouldn't reach this point.");
      }
    }
    
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
