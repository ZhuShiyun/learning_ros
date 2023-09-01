/**
 * @file tenth_power_client.cpp
 * @author Birb
 * @brief  action通信-client:
 * 使用命令$ rosrun tenth_power tenth_power_client <number>
 * 得到<number>的十次方
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tenth_power/TenthPowerAction.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tenth_power_client");
  if (argc != 2) {
    ROS_INFO_STREAM(
        "\033[1;32m Usage: rosrun tenth_power tenth_power_client <number> "
        " \033[0m");
    return 1;
  }

  int input_number = atoi(argv[1]);

  actionlib::SimpleActionClient<tenth_power::TenthPowerAction> ac("tenth_power",
                                                                  true);

  ac.waitForServer();

  // 创建Action目标
  tenth_power::TenthPowerGoal goal;
  goal.input_number = input_number;

  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    tenth_power::TenthPowerResultConstPtr result = ac.getResult();
    ROS_INFO("The result is: %d", result->output_number[10]);  // ？

    // ROS_INFO("Hey, here is  result:");
    // for (int i = 0; i < result->output_number.size(); i++) {
    //   ROS_INFO("%d", result->output_number[i]);
    // }
  }

  else {
    ROS_ERROR("Action did not succeed.");
  }

  return 0;
}
