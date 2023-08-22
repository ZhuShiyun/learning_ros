#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cockatoo_node");
  printf("Ahhhhh \n");

  // 发布话题
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>(
      "NoisyDonkey", 10);  // nh.advertise("话题名称", 缓存长度)

  // 设置频率
  ros::Rate loop_rate{10};

  // 发布消息
  while (ros::ok()) {
    printf("I'm hungry!!!\n");
    std_msgs::String msg;
    msg.data = "Give me french fries!";
    pub.publish(msg);
    loop_rate.sleep();  // 短时间阻塞
  }

  return 0;
}
