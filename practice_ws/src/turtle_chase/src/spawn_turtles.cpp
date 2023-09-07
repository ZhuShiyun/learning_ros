/*
 * @Author: your name
 * @Date: 2023-09-07 14:28:57
 * @LastEditTime: 2023-09-07 16:54:07
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/turtle_chase/src/spawn_turtles.cpp
 */

#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spawn_turtles");
    ros::NodeHandle nh;

    // Create a service client for spawning turtles
    ros::ServiceClient spawnTurtleClient = nh.serviceClient<turtlesim::Spawn>("/spawn");

    // turtlesim::Spawn turtle1;
    // turtle1.request.x = 2.0;
    // turtle1.request.y = 2.0;
    // turtle1.request.theta = 0.0;
    // turtle1.request.name = "turtle1";

    turtlesim::Spawn turtle2;
    turtle2.request.x = 8.0;
    turtle2.request.y = 8.0;
    turtle2.request.theta = 0.0;
    turtle2.request.name = "turtle2";

    // Call the service to spawn the turtles
    if (spawnTurtleClient.call(turtle2))
    {
        ROS_INFO("Turtles spawned successfully!");
    }
    else
    {
        ROS_ERROR("Failed to spawn turtles!");
    }

    // ros::spin();

    return 0;
}
