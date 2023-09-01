/**
 * @file tenth_power_server.cpp
 * @author Birb
 * @brief  action通信应用-server: 输出<input number>的十次方
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <tenth_power/TenthPowerAction.h>

class TenthPowerServer {
 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tenth_power::TenthPowerAction> as_;
  std::string action_name_;  //?

 public:
  TenthPowerServer(std::string name)
      : as_(nh_, name, boost::bind(&TenthPowerServer::executeCB, this, _1),
            false),
        action_name_(name) {
    as_.start();
  }

  ~TenthPowerServer(void) {}

  void executeCB(const tenth_power::TenthPowerGoalConstPtr &goal) {
    int32_t input_number = goal->input_number;

    tenth_power::TenthPowerFeedback feedback;
    tenth_power::TenthPowerResult result;

    for (int i = 0; i < 10; i++) {
      result.output_number.push_back(pow(input_number, i));
      feedback.percentage = (i * 10) / 100.0;
      as_.publishFeedback(feedback);
      ros::Duration(1.0).sleep();
    }

    as_.setSucceeded(result);
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tenth_power_server");

  TenthPowerServer server("tenth_power");

  ros::spin();

  return 0;
}
