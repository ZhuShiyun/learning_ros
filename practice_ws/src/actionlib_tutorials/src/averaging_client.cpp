/**
 * @file averaging_client.cpp
 * @author BIrb
 * @brief  This tutorial covers using the simple_action_client library to create
 * a averaging action client. This example program spins a thread, creates an
 * action client, and sends a goal to the action server.
 * @version 0.1
 * @date 2023-08-28
 *
 * @copyright Copyright (c) 2023
 * Copy from
 * https://wiki.ros.org/cn/actionlib_tutorials/Tutorials/SimpleActionClient%28Threaded%29
 */

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/AveragingAction.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

void spinThread() { ros::spin(); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_averaging");

  // create the action client
  actionlib::SimpleActionClient<actionlib_tutorials::AveragingAction> ac(
      "averaging");
  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::AveragingGoal goal;
  goal.samples = 100;
  ac.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else
    ROS_INFO("Action did not finish before the time out.");

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  // exit
  return 0;
}
/**
 * @brief  --------------
 *
 */

// #include <ros/ros.h>
// #include <std_msgs/Float32.h>
// #include <random>

// void gen_number() {
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<std_msgs::Float32>("random_number",
//     10); ROS_INFO("Generating random numbers");

//     while (ros::ok()) {
//         std_msgs::Float32 msg;
//         std::random_device rd;
//         std::mt19937 gen(rd());
//         std::normal_distribution<float> distribution(5.0, 1.0);
//         msg.data = distribution(gen);
//         pub.publish(msg);
//         ros::Duration(0.05).sleep();
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "random_number_generator");
//     try {
//         gen_number();
//     } catch (const std::exception& e) {
//         ROS_ERROR("An error occurred: %s", e.what());
//     }
//     return 0;
// }
