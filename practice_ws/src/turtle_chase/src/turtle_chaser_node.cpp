/*
 * @Author: your name
 * @Date: 2023-09-07 14:04:45
 * @LastEditTime: 2023-09-07 17:08:10
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/turtle_chase/src/turtle_chaser_node.cpp
 */
// #include <ros/ros.h>
// #include <turtlesim/Pose.h>
// #include <std_msgs/String.h>
// #include <sstream>
// #include <cstdlib>

// // turtlesim::Pose turtle1_pose;
// // turtlesim::Pose turtle2_pose;
// // ros::Publisher pub;

// // void turtle1Callback(const turtlesim::Pose::ConstPtr& msg)
// // {
// //     turtle1_pose = *msg;
// // }

// // void turtle2Callback(const turtlesim::Pose::ConstPtr& msg)
// // {
// //     turtle2_pose = *msg;

// //     // Calculate the distance between turtle1 and turtle2
// //     double distance = sqrt(pow(turtle1_pose.x - turtle2_pose.x, 2) + pow(turtle1_pose.y - turtle2_pose.y, 2));

// //     // Set a threshold for catching the thief
// //     double catch_threshold = 1.0;

// //     if (distance < catch_threshold)
// //     {
// //         ROS_INFO("Thief has been caught!");
        
// //         // Generate random positions for turtle2
// //         turtle2_pose.x = rand() % 11;
// //         turtle2_pose.y = rand() % 11;

// //         std_msgs::String message;
// //         message.data = "MoveTurtle2";
// //         pub.publish(message);

// //         // // Publish a message to move turtle1 to the new position of turtle2
// //         // geometry_msgs::Twist cmd_vel;
// //         // cmd_vel.linear.x = turtle2_pose.x;
// //         // cmd_vel.linear.y = turtle2_pose.y;
// //         // pub.publish(cmd_vel);
// //     }
// // }

// // int main(int argc, char **argv)
// // {
// //     ros::init(argc, argv, "turtle_chaser");
// //     ros::NodeHandle nh;

// //     // Subscribe to turtle1 and turtle2's positions
// //     ros::Subscriber sub1 = nh.subscribe("turtle1/pose", 1000, turtle1Callback);
// //     ros::Subscriber sub2 = nh.subscribe("turtle2/pose", 1000, turtle2Callback);

// //     // Publish message to control turtle2's movement
// //     pub = nh.advertise<std_msgs::String>("turtle2/cmd_vel", 1000);

// //     ros::spin();

// //     return 0;
// // }

// #include <ros/ros.h>
// #include <turtlesim/Pose.h>
// #include <std_msgs/String.h>
// #include <geometry_msgs/Twist.h>
// #include <sstream>
// #include <cstdlib>

// turtlesim::Pose turtle1_pose;
// turtlesim::Pose turtle2_pose;
// ros::Publisher pub;

// void turtle1Callback(const turtlesim::Pose::ConstPtr& msg)
// {
//     turtle1_pose = *msg;

//     // Calculate the distance between turtle1 and turtle2
//     double distance = sqrt(pow(turtle1_pose.x - turtle2_pose.x, 2) + pow(turtle1_pose.y - turtle2_pose.y, 2));

//     // Set a threshold for catching the thief
//     double catch_threshold = 0.5;

//     if (distance < catch_threshold)
//     {
//         ROS_INFO("Thief has been caught!");
        
//         // Generate random positions for turtle2
//         turtle2_pose.x = rand() % 7;
//         turtle2_pose.y = rand() % 7;
        
//         // Publish a message to move turtle1 to the new position of turtle2
//         geometry_msgs::Twist cmd_vel;
//         cmd_vel.linear.x = turtle2_pose.x;
//         cmd_vel.linear.y = turtle2_pose.y;
//         pub.publish(cmd_vel);
//     }
// }

// void turtle2Callback(const turtlesim::Pose::ConstPtr& msg)
// {
//     turtle2_pose = *msg;
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "turtle_chaser");
//     ros::NodeHandle nh;

//     // Subscribe to turtle1 and turtle2's positions
//     ros::Subscriber sub1 = nh.subscribe("turtle1/pose", 1000, turtle1Callback);
//     ros::Subscriber sub2 = nh.subscribe("turtle2/pose", 1000, turtle2Callback);

//     // Publish twist messages to control turtle1's movement
//     pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

//     ros::spin();

//     return 0;
// }

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


ros::Publisher pub;
tf2_ros::Buffer tfBuffer;

void checkThiefDistance()
{
    try
    {
        // 获取turtle1相对于turtle2的变换
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("turtle1", "turtle2", ros::Time(0));
        
        // 计算距离
        double distance = sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));

        // 设置一个捕捉阈值
        double catch_threshold = 2;

        if (distance < catch_threshold)
        {
            ROS_INFO("Thief has been caught!");

            // 重新生成小偷的位置
            double newX = rand() % 11 + 1; // turtlesim的世界大小默认是[0, 11] x [0, 11]
            double newY = rand() % 11 + 1;

            // 使用服务调用来重新设置turtle2的位置
            turtlesim::Spawn srv;
            srv.request.x = newX;
            srv.request.y = newY;

            ros::service::call("turtle2/teleport_absolute", srv);
        }
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "turtle_chaser");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
// tf2_ros::TransformListener tfListener(tfBuffer);

// Wait for the transforms to become available
ros::Duration(1.0).sleep();  // give some time for the transforms to be published
while (!tfBuffer.canTransform("turtle1", "world", ros::Time(0)) || 
       !tfBuffer.canTransform("turtle2", "world", ros::Time(0))) {
    ros::Duration(0.1).sleep();
}

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&){ checkThiefDistance(); });

    ros::spin();

    return 0;
}

