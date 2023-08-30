#include "get_prime/GetPrime.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "get_prime");
  if (argc != 2) {
    ROS_INFO("Usege: get_prime <number>");
    return 1;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<get_prime::GetPrime>("get_prime");

  get_prime::GetPrime srv;
  srv.request.ipt = atoi(argv[1]);

  /**
   * @brief  监修中。。
   *
   */
  if (client.call(srv)) {
    ROS_INFO("Show get_prime result: ");
    for (const auto &prime : srv.response.primes) {
      ROS_INFO("%ld", prime);
    }
  } else {
    ROS_ERROR("Failed to call service get_prime");
    return 1;
  }

  return 0;
}
