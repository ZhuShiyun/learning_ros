#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "turtle2";
  spawner.call(turtle);

  ros::Publisher turtle_vel =
      node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()) {
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped =
          tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    /**
     * @brief 这段代码计算了turtle2机器人的线速度 (vel_msg.linear.x) 和角速度
(vel_msg.angular.z)，以便根据turtle1和turtle2之间的坐标变换来控制turtle2。
     * 具体来说：
atan2 函数用于计算机器人的角速度 (vel_msg.angular.z)。它使用
transformStamped.transform.translation.y 和
transformStamped.transform.translation.x
作为参数，计算出一个角度，这个角度可以使turtle2的朝向朝向turtle1。 sqrt 和 pow
函数用于计算机器人的线速度
(vel_msg.linear.x)，它根据turtle1和turtle2之间的距离来确定线速度。这里使用了欧几里得距离的计算公式。
这些计算确保了turtle2会根据其与turtle1之间的相对位置调整其线速度和角速度，从而试图一直跟随turtle1。
如果turtle1移动，turtle2会尽力跟随，并在turtle1周围保持相对位置。这就是代码中使turtle2尝试一直跟随turtle1的关键部分。
但请注意，实际效果可能会受到环境和控制参数的影响，可能不会完美地跟随。
     *
     */
    geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x =
        0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                   pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};