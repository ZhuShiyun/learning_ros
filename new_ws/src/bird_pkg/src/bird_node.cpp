#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "bird_node");
  printf("Mad dog and englishmen go out in the midday sun..\n");

  // 发布话题
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>(
      "MDAE", 10);  // nh.advertise("话题名称", 缓存长度)

  // 设置频率
  ros::Rate loop_rate{10};

  // 发布消息
  while (ros::ok()) {
    printf("I'm going to enjoy the sun shine! \n");
    std_msgs::String msg;
    msg.data = "Come on & go with me!";
    pub.publish(msg);
    loop_rate.sleep();  // 短时间阻塞
  }

  return 0;
}
