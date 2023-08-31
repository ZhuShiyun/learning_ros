/**
 * @file turtle_rectangle.cpp
 * @author Birb
 * @brief  turtle画一个矩形-速度控制
 * @version 0.1
 * @date 2023-08-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#define PI 3.1415926

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "turtle_rectangle_vel");
  ros::NodeHandle nh;

  ros::Publisher pub =
      nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  ros::Rate rate(2);

  geometry_msgs::Twist twist;

  float linear_x;
  float angular_z;
  int count = 0;
  int countz = 0;

  twist.linear.x = 0;
  twist.angular.z = 0;

  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;

  while (ros::ok()) {
    count++;
    linear_x = 0.6;
    angular_z = 0;

    if (count == 5) {
      angular_z = PI;
      count = 0;
      countz++;
    }

    if (countz >= 9) {
      linear_x = 0;
      angular_z = 0;
    }
    twist.linear.x = linear_x;
    twist.angular.z = angular_z;
    pub.publish(twist);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}