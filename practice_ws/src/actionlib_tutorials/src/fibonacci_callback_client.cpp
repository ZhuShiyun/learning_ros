/**
 * @file fibonacci_callback_client.cpp
 * @author Birb
 * @brief  教程：用回调来避免调用waitForResult()来阻塞用于目标完成。
 * @version 0.1
 * @date 2023-08-28
 *
 * @copyright Copyright (c) 2023
 *      Copy from
 * https://wiki.ros.org/cn/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client
 */

#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <ros/ros.h>

using namespace actionlib_tutorials;
typedef actionlib::SimpleActionClient<FibonacciAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const FibonacciResultConstPtr& result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %i", result->sequence.back());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb() { ROS_INFO("Goal just went active"); }

// Called every time feedback is received for the goal
void feedbackCb(const FibonacciFeedbackConstPtr& feedback) {
  ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_fibonacci_callback");

  // Create the action client
  Client ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  // Send Goal
  FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  ros::spin();
  return 0;
}