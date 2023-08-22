/**
 * @file velocity_publisher.cpp
 * @author your name (you@domain.com)
 * @brief 该例程将发布/person_info话题，消息类型learning_topic::Person
 * @version 0.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <learning_topic/Person.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  // ROS节点初始化
  ros::init(argc, argv, "person_publisher");

  // 创建节点句柄
  ros::NodeHandle n;

  // 创建一个Publiser，发布名为/person_info的topic,消息类型为learning_topic::Person,
  // 队列长度10
  ros::Publisher person_info_pub =
      n.advertise<learning_topic::Person>("/person_info", 10);

  // 设置循环的频率
  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok()) {
    // 初始化learning_topic::Person类型的消息
    learning_topic::Person person_msg;
    person_msg.name = "Wendy";
    person_msg.age = 50;
    person_msg.gender = learning_topic::Person::female;

    // 发布消息
    person_info_pub.publish(person_msg);
    ROS_INFO("Publish Person Info: name: %s  age:%d  gender: %d",
             person_msg.name.c_str(), person_msg.age, person_msg.gender);

    // 按照循环频率延时
    loop_rate.sleep();
  }

  return 0;
}
