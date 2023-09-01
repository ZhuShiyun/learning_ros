/**
 * @file generate_primes_client.cpp
 * @author birb
 * @brief  action通信-client:
 * 当使用命令 $ rosrun prime_actionlib generate_primes_server <number> 时，
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <actionlib/client/simple_action_client.h>
#include <prime_actionlib/GeneratePrimesAction.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "generate_primes_client");
  if (argc != 2) {
    ROS_ERROR("Usage: generate_primes_client <max_number>");
    return 1;
  }

  int max_number = atoi(argv[1]);

  actionlib::SimpleActionClient<prime_actionlib::GeneratePrimesAction> ac(
      "generate_primes", true);
  ac.waitForServer();

  prime_actionlib::GeneratePrimesGoal goal;
  goal.max_number = max_number;

  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    prime_actionlib::GeneratePrimesResultConstPtr result = ac.getResult();
    ROS_INFO("Generated prime numbers: ");
    for (int prime : result->primes) {
      ROS_INFO("%d", prime);
    }
  } else {
    ROS_ERROR("Action did not succeed.");
  }

  return 0;
}
