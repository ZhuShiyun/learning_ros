/*
 * @Author: your name
 * @Date: 2023-09-13 16:06:40
 * @LastEditTime: 2023-09-14 10:29:45
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_pub_sub/src/demo01_pub.cpp
 */

/*
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)

         PS: 二者需要设置相同的话题


    消息发布方:
        循环发布信息:HelloWorld 后缀数字编号

    实现流程:
        1.包含头文件 
            ROS中文本类型 ---> std_msgs/String
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 发布者 对象
        5.组织被发布的数据，并编写逻辑发布数据

*/
// 1.包含头文件
#include<ros/ros.h>
#include<std_msgs/String.h>
// 实现需求中的字符串拼接数字
#include<sstream>

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");
    //  2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"sayHi");
    // 3.实例化 ROS 句柄
    ros::NodeHandle nh;

    // 4.实例化 发布者 对象
    /*
        inline ros::Publisher ros::NodeHandle::advertise<std_msgs::String>(const std::__cxx11::string &topic, uint32_t queue_size, bool latch = false)
        还有 3 个重载

        Advertise a topic, simple version This call connects to the master to publicize that the node will be publishing messages on the given topic. This method returns a Publisher that allows you to publish a message on this topic. This version of advertise is a templated convenience function, and can be used like so ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);

        参数:
            topic – Topic to advertise on
            queue_size – Maximum number of outgoing messages to be queued for delivery to subscribers
            latch – (optional) If true, the last message published on this topic will be saved and sent to new subscribers when they connect
    */
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatRoom",10);

    // 5.组织被发布的数据，并编写逻辑发布数据
    std_msgs::String msg;
    // msg.data = "Good morning!";

    ros::Rate rate(10);
    // 循环，发布数据 
    // 设置编号
    int count = 0;

    ros::Duration(3.0).sleep();

    while (ros::ok())
    {
        count++;

        // 实现需求中的字符串拼接数字
        std::stringstream ss;
        ss << "Good morning! - - - >" << count;
        msg.data = ss.str();

        // 发布话题
        pub.publish(msg);
        ROS_INFO("Data:%s",ss.str().c_str());
        rate.sleep();

        // 官方建议，用于处理回调函数
        ros::spinOnce();
        
    }
    

    return 0;
}

