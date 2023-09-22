#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <random>

void gen_number() {
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32>("random_number", 10);
  ROS_INFO("Generating random numbers");

  while (ros::ok()) {
    std_msgs::Float32 msg;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> distribution(5.0, 1.0);
    msg.data = distribution(gen);
    pub.publish(msg);
    ros::Duration(0.05).sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_number_generator");
  try {
    gen_number();
  } catch (const std::exception& e) {
    ROS_ERROR("An error occurred: %s", e.what());
  }
  return 0;
}
