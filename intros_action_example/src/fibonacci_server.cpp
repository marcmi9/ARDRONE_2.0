#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <intros_action_example/FibonacciAction.h>

void FiboCallback(const intros_action_example::FibonacciGoalConstPtr &goal,actionlib::SimpleActionServer<intros_action_example::FibonacciAction>* as) {
  
  ROS_INFO("action_Server: New goal received");
  bool success = true;
  // create messages that are used to published feedback/result
  intros_action_example::FibonacciFeedback feedback;
  intros_action_example::FibonacciResult result;
  // push_back the seeds for the fibonacci sequence
  feedback.sequence.clear();
  feedback.sequence.push_back(0);
  feedback.sequence.push_back(1);
  // publish info to the console for the user
  ROS_INFO("Fibonacci action: Executing, creating fibonacci sequence of order %i with seeds %i, %i",goal->order, feedback.sequence[0], feedback.sequence[1]);

  // start executing the action
  for (int i = 1; i <= goal->order; i++) {
    // check that preempt has not been requested by the client
    if (as->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("Fibonacci action: Preempted");
      // set the action state to preempted
      as->setPreempted();
      success = false;
      break;
    }
    feedback.sequence.push_back(feedback.sequence[i] + feedback.sequence[i - 1]);
    // publish the feedback
    as->publishFeedback(feedback);
  }
  if (success){
    result.sequence = feedback.sequence;
    ROS_INFO("Fibonacci action: Succeeded");
    // set the action state to succeeded
    as->setSucceeded(result);
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "action_server");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  actionlib::SimpleActionServer<intros_action_example::FibonacciAction> as(nh,"action_server/fibonacci",boost::bind(&FiboCallback, _1, &as),false);

  as.start();

  while (ros::ok()) {
    //<your main code can go here (except callbacks)>
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
