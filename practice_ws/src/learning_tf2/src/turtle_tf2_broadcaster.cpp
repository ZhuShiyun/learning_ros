/**
 * @file turtle_tf2_broadcaster.cpp
 * @author Birb
 * @brief
 * @version 0.1
 * @date 2023-09-04
 *
 * @copyright Copyright (c) 2023
 * copy from
 * https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29
 */

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
/**
 * tf2包提供了transformbroadcast的实现，以帮助简化发布转换的任务。
 * 要使用transformbroadcast，我们需要包含tf2_ros/transform_broadcast
 .h头文件。
 */
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg) {
  // 在这里，我们创建了一个transformbroadcast对象，稍后我们将使用它通过网络发送转换。
  static tf2_ros::TransformBroadcaster br;

  /**
   * 这里我们创建一个Transform对象并为其提供适当的元数据。

        1.我们需要为被发布的转换提供一个时间戳，我们将使用当前时间ros::
   time::now()来标记它。

        2.然后，我们需要设置所创建链接的父框架的名称，在本例中为“world”

        3.
   最后，我们需要设置正在创建的链接的子节点的名称，在本例中，这是海龟本身的名称。
   *
   */
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = turtle_name;
  // Here we copy the information from the 3D turtle pose into the 3D
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // This is where the real work is done. Sending a transform with a
  // TransformBroadcaster requires passing in just the transform itself.
  br.sendTransform(transformStamped);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  if (!private_node.hasParam("turtle")) {
    if (argc != 2) {
      ROS_ERROR("need turtle name as argument");
      return -1;
    };
    turtle_name = argv[1];
  } else {
    private_node.getParam("turtle", turtle_name);
  }

  ros::NodeHandle node;
  ros::Subscriber sub =
      node.subscribe(turtle_name + "/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};
