/**
 * @brief 这个对了(原始代码在turtle_move.cpp)
 * 复盘：之前是在main里先发布了一个角速度消息，再订阅姿态主题，
 * 这样会导致在订阅姿态主题之前，就已经发布了一个初始的角速度命令，从而影响后续的逻辑。
 * 注释里的代码都是错的..
 *
 * 效果：
 * 当海龟模拟器的角度大于1.0弧度时，海龟会以角速度1.0逆时针旋转，同时以线速度0.7向前移动。
 * 当角度小于等于1.0弧度时，海龟只会以角速度1.0逆时针旋转。
 */
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <turtlesim/Pose.h>

// 将pub设置为全局变量，可以确保在main函数中初始化它，然后在回调函数中使用它来发布消息。
ros::Publisher velocity_publisher;

void poseCallback(const turtlesim::Pose::ConstPtr& turtle_pose) {
  geometry_msgs::Twist vel_msg;

  // 设置一个角速度并发布
  vel_msg.angular.z = 1.0;
  velocity_publisher.publish(vel_msg);

  // 检查turtle的角度是不是大于1.0,如果大于则给他一个0.7的线速度和1.0的角速度
  if (turtle_pose->theta > 1.0) {
    vel_msg.linear.x = 0.7;
    vel_msg.angular.z = 1.0;
    velocity_publisher.publish(vel_msg);
  }
}

int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "turtle_subscriber_publisher");
  // 创建节点句柄
  ros::NodeHandle nh;

  // Advertise the velocity topic "/turtle1/cmd_vel"
  velocity_publisher =
      nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  // Subscribe to the turtle's pose topic
  ros::Subscriber pose_subscriber =
      nh.subscribe("/turtle1/pose", 10, poseCallback);

  ros::spin();

  return 0;
}

/**
 * @file sub_pub.cpp
 * @author Birb
 * @brief 订阅turtle朝向，在合适朝向时发布线速度指令(错误代码)
 * @version 0.1
 * @date 2023-08-31
 *
 * @copyright Copyright (c) 2023
 * step: 在终端中输入：$ rosmsg show turtlesim/Pose
 *
 */

// #include <geometry_msgs/Twist.h>
// #include <ros/ros.h>
// #include <turtlesim/Pose.h>

// ros::Publisher pub;
// // ros::Subscriber pose_subscriber;

// geometry_msgs::Twist vel_msg;

// // 接收到订阅的消息后，会进入消息回调函数
// void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
//   // 将接收到的消息打印出来
//   ROS_INFO("Turtle pose: x:%0.6f, y:%0.6f, theta: %f", msg->x, msg->y,
//            msg->theta);

//   // 判断朝向是否大于1，然后发布线速度指令
//   if (msg->theta > 1.0) {
//     // geometry_msgs::Twist vel_msg;
//     vel_msg.linear.x = 0.5;
//     pub.publish(vel_msg);
//     ROS_INFO(
//         "Publish turtle velocity command [if theta > 1.0, linear.x = 0.5
//         m/s]");
//   }
// }

// int main(int argc, char* argv[]) {
//   // 初始化ROS节点
//   ros::init(argc, argv, "sub_pub");

//   // 创建节点句柄
//   ros::NodeHandle nh;

//   // 创建一个Subscriber，订阅名为/turtle1/pose 的topic，
//   // 注册回调函数poseCallback
//   ros::Subscriber pose_subscriber =
//       nh.subscribe("/turtle1/pose", 10, poseCallback);

//   //
//   创建一个Publiser，发布名为/turtle1/cmd_vel的topic,消息类型为geometry::Twist,
//   //  队列长度10
//   ros::Publisher pub =
//       nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

//   // 设置循环的频率
//   ros::Rate loop_rate(10);

//   vel_msg.angular.z = 1;
//   pub.publish(vel_msg);

//   // ROS_INFO("Publish turtle angular velocity command [angular.z
//   = 1.0rad/s]");

//   // 等待一段时间让turtle自转
//   ros::Duration(2.0).sleep();

//   // int count = 0;
//   while (ros::ok()) {
//     // 循环等待回调函数
//     ros::spinOnce();

//     // 按照循环频率延时
//     loop_rate.sleep();
//   }

//   return 0;
// }

/**
 * @brief
 *
 */
// #include <geometry_msgs/Twist.h>
// #include <ros/ros.h>
// #include <turtlesim/Pose.h>

// ros::Publisher car_pub;
// geometry_msgs::Twist car_cmd;

// void callback(const turtlesim::Pose::ConstPtr& data) {
//   ROS_INFO("%f", data->theta);

//   car_cmd.angular.z = 1.0;
//   car_pub.publish(car_cmd);

//   if (data->theta > 1.0) {
//     car_cmd.linear.x = 0.7;
//     car_pub.publish(car_cmd);
//   }
// }

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "turtle_subscriber_publisher");
//   ros::NodeHandle nh;

//   car_pub = nh.advertise<geometry_msgs::Twist>("/turtle/cmd_vel", 10);

//   ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, callback);

//   ros::spin();

//   return 0;
// }
