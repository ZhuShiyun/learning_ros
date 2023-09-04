// #include <geometry_msgs/TransformStamped.h>
// #include <ros/ros.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/static_transform_broadcaster.h>

// #include <cstdio>

// std::string static_turtle_name;  // ?

// int main(int argc, char *argv[]) {
//   ros::init(argc, argv, "my_static_tf2_broadcaster");
//   if (argc != 8) {
//     ROS_ERROR(
//         "Invalid number of parameters\nusage: static_turtle_tf2_broadcaster "
//         "child_frame_name x y xz roll pitch yaw");
//     return -1;
//   }
//   if (strcmp(argv[1], "world") == 0) {
//     ROS_ERROR("Your static turtle name cannot be 'world'");
//     return -1;
//   }
//   static_turtle_name = argv[1];

//   //
//   这里创建了一个TransformStamped对象，它将是我们在填充后发送的消息。在填充实际的转换值之前，我们需要给它适当的元数据。
//   static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//   geometry_msgs::TransformStamped static_transformStamped;

//   static_transformStamped.header.stamp = ros::Time::now();
//   static_transformStamped.header.frame_id = "world";#include
//   <geometry_msgs/TransformStamped.h>
// #include <ros/ros.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/static_transform_broadcaster.h>

// #include <cstdio>

// std::string static_turtle_name;  // ?

// int main(int argc, char *argv[]) {
//   ros::init(argc, argv, "my_static_tf2_broadcaster");
//   if (argc != 8) {
//     ROS_ERROR(
//         "Invalid number of parameters\nusage: static_turtle_tf2_broadcaster "
//         "child_frame_name x y xz roll pitch yaw");
//     return -1;
//   }
//   if (strcmp(argv[1], "world") == 0) {
//     ROS_ERROR("Your static turtle name cannot be 'world'");
//     return -1;
//   }
//   static_turtle_name = argv[1];

//   //
//   这里创建了一个TransformStamped对象，它将是我们在填充后发送的消息。在填充实际的转换值之前，我们需要给它适当的元数据。
//   static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//   geometry_msgs::TransformStamped static_transformStamped;

//   static_transformStamped.header.stamp = ros::Time::now();
//   static_transformStamped.header.frame_id = "world";
//   static_transformStamped.child_frame_id = static_turtle_name;
//   static_transformStamped.transform.translation.x = atof(argv[2]);
//   static_transformStamped.transform.translation.y = atof(argv[3]);
//   static_transformStamped.transform.translation.z = atof(argv[4]);
//   tf2::Quaternion quat;
//   quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
//   static_transformStamped.transform.rotation.x = quat.x();
//   static_transformStamped.transform.rotation.y = quat.y();
//   static_transformStamped.transform.rotation.z = quat.z();
//   static_transformStamped.transform.rotation.w = quat.w();
//   static_broadcaster.sendTransform(static_transformStamped);
//   ROS_INFO("Spinning until killed publishing %s to world",
//            static_turtle_name.c_str());
//   ros::spin();
//   return 0;
// };

//   static_transformStamped.child_frame_id = static_turtle_name;
//   static_transformStamped.transform.translation.x = atof(argv[2]);
//   static_transformStamped.transform.translation.y = atof(argv[3]);
//   static_transformStamped.transform.translation.z = atof(argv[4]);
//   tf2::Quaternion quat;
//   quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
//   static_transformStamped.transform.rotation.x = quat.x();
//   static_transformStamped.transform.rotation.y = quat.y();
//   static_transformStamped.transform.rotation.z = quat.z();
//   static_transformStamped.transform.rotation.w = quat.w();
//   static_broadcaster.sendTransform(static_transformStamped);
//   ROS_INFO("Spinning until killed publishing %s to world",
//            static_turtle_name.c_str());
//   ros::spin();
//   return 0;
// };
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = turtle_name;
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
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
