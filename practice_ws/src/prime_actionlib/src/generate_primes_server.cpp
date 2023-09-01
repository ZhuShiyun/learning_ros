/**
 * @file generate_primes_server.cpp
 * @author Birb
 * @brief  action通信应用-server：输出从1到<number>的质数
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <actionlib/server/simple_action_server.h>
#include <prime_actionlib/GeneratePrimesAction.h>
#include <ros/ros.h>

bool isPrime(int num) {
  if (num <= 1) return false;
  for (int i = 2; i * i <= num; ++i) {
    if (num % i == 0) return false;
  }
  return true;
}

class GeneratePrimesServer {
 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<prime_actionlib::GeneratePrimesAction> as_;

 public:
  GeneratePrimesServer()
      : as_(nh_, "generate_primes",
            boost::bind(&GeneratePrimesServer::execute, this, _1), false) {
    as_.start();
  }

  void execute(const prime_actionlib::GeneratePrimesGoalConstPtr& goal) {
    prime_actionlib::GeneratePrimesFeedback feedback;
    prime_actionlib::GeneratePrimesResult result;

    int max_number = goal->max_number;
    int primes_count = 0;

    for (int i = 1; i <= max_number; ++i) {
      if (isPrime(i)) {
        result.primes.push_back(i);
        primes_count++;
      }
      feedback.percentage = (float)i / max_number * 100;
      as_.publishFeedback(feedback);
    }

    ROS_INFO("Generated %d prime numbers.", primes_count);
    as_.setSucceeded(result);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "generate_primes_server");
  GeneratePrimesServer server;
  ros::spin();
  return 0;
}
