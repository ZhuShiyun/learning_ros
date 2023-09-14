/*
 * @Author: your name
 * @Date: 2023-09-13 14:55:10
 * @LastEditTime: 2023-09-13 15:29:01
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_test/src/test01_pub_twist.cpp
 */

/*
    编写 ROS 节点，控制小乌龟画圆

    准备工作:
        1.获取topic(已知: /turtle1/cmd_vel)
        2.获取消息类型(已知: geometry_msgs/Twist)
        3.运行前，注意先启动 turtlesim_node 节点

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建发布者对象
        4.循环发布运动控制消息
*/
// 1.包含头文件
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //  2.初始化 ROS 节点
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    //  3.创建发布者对象
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);
    //  4.循环发布运动控制消息
    // 4.1 组织消息
    geometry_msgs::Twist msg;
    msg.linear.x =2.0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 1.0;     
    // 4-2.设置发送频率
    ros::Rate r(10);
    //4-3.循环发送
    while (ros::ok())
    {
       pub.publish(msg);
       ros::spinOnce();
       ROS_INFO("转圈圈～");
    }

    
    return 0;
}

