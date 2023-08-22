#include <ros/ros.h>
#include <std_msgs/String.h>

void tree_callback(std_msgs::String msg) {
  ROS_INFO(msg.data.c_str());  // 加时间戳
  // printf("\n");
}

void cockatoo_callback(std_msgs::String msg) {
  ROS_WARN(msg.data.c_str());  // 会显示成黄色，和tree的信息区分
  // printf("\n");
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tree_node");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("MDAE", 10, tree_callback);
  ros::Subscriber sub_1 = nh.subscribe("NoisyDonkey", 10, cockatoo_callback);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}

// #include "ros/ros.h"
// #include "std_msgs/String.h"

// void chatterCallback(const std_msgs::String::ConstPtr& msg) {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

// int main(int argc, char* argv[]) {
//   ros::init(argc, argv, "listener");

//   ros::NodeHandle n;
//   ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);
//   ros::spin();

//   return 0;
// }
