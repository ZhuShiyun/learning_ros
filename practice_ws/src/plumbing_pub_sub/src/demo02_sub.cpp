/*
 * @Author: your name
 * @Date: 2023-09-14 09:34:25
 * @LastEditTime: 2023-09-14 13:48:19
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_pub_sub/src/demo02_sub.cpp
 */
/*
    订阅方实现：
        1.包含头文件 
            ROS中文本类型 ---> std_msgs/String
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 订阅者 对象
        5.处理订阅的数据
        6. spin
*/

// 1.包含头文件 
//     ROS中文本类型 ---> std_msgs/String
#include<ros/ros.h>
#include<std_msgs/String.h>

void dm02Callback(const std_msgs::String::ConstPtr &msg){
    // 通过msg获取并操作订阅到的数据
    ROS_INFO("demo02_sub 订阅的数据：%s", msg->data.c_str());
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
     // 2.初始化 ROS 节点:命名(唯一)
    ros::init(argc, argv, "demo02_sub");

    // 3.实例化 ROS 句柄
    ros::NodeHandle nh;

    // 4.实例化 订阅者 对象
    /**
     * ros::Subscriber subscribe<M>(const std::__cxx11::string &topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr<const M> &), const ros::TransportHints &transport_hints = ros::TransportHints())

    */
    ros::Subscriber sub = nh.subscribe("chatRoom", 10, dm02Callback);

    // 5.处理订阅到的数据

    ros::spin();

    return 0;
}

