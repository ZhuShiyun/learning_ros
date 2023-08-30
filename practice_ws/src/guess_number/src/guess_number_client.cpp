/**
 * @file guess_number_server.cpp
 * @author Birb
 * @brief
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "guess_number/GuessNumber.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "guess_number_client");
  if (argc != 2) {
    ROS_ERROR("Usage: guess_number_client <guess>");
    return 1;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<guess_number::GuessNumber>("guess_number");

  guess_number::GuessNumber srv;
  srv.request.guess = atoi(argv[1]);

  if (client.call(srv)) {
    if (srv.response.is_correct) {
      ROS_INFO("Server: %s", srv.response.message.c_str());
    } else {
      ROS_INFO("Server: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Failed to call service guess_number");
    return 1;
  }

  return 0;
}
