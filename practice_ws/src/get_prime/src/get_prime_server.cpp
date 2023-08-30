#include "get_prime/GetPrime.h"
#include "ros/ros.h"

bool isPrime(int ipt) {
  for (int i = 2; i * i <= ipt; i++) {
    if (ipt % i == 0) return false;
  }

  return true;
}

bool primeCallback(get_prime::GetPrime::Request &req,
                   get_prime::GetPrime::Response &res) {
  for (int n = 2; n <= req.ipt; n++) {
    if (isPrime(n)) res.primes.push_back(n);
  }
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "get_prime_server");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("get_prime", primeCallback);

  ROS_INFO("Get Prime Server is ready.");
  ros::spin();

  return 0;
}
