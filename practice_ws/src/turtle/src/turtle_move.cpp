/**
 * @file turtle_move.cpp
 * @author Birb
 * @brief sub 和pub
 * 当海龟模拟器的角度大于1.0弧度时，海龟会以角速度1.0逆时针旋转，同时以线速度0.7向前移动。
 * 当角度小于等于1.0弧度时，海龟只会以角速度1.0逆时针旋转。
 * @version 0.1
 * @date 2023-08-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "turtlesim/Pose.h"

ros::Publisher car_pub;

void callback(const turtlesim::Pose::ConstPtr& data) {
  ROS_INFO_STREAM(data->theta);
  geometry_msgs::Twist car_cmd;

  car_cmd.angular.z = 1.0;
  car_pub.publish(car_cmd);

  if (data->theta > 1.0) {
    car_cmd.linear.x = 0.7;
    car_pub.publish(car_cmd);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  car_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  ros::Subscriber sub = n.subscribe("/turtle1/pose", 10, callback);

  ros::spin();

  return 0;
}
