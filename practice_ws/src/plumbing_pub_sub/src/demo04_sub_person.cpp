/*
 * @Author: your name
 * @Date: 2023-09-14 13:34:30
 * @LastEditTime: 2023-09-14 13:56:15
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_pub_sub/src/demo04_sub_person.cpp
 */

/*
    订阅方实现：
        1.包含头文件 
            #include "plumbing_pub_sub/Person.h"
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 订阅者 对象
        5.处理订阅的数据
        6. spin
*/
//  1.包含头文件 
#include "ros/ros.h"
#include "plumbing_pub_sub/Person.h"

void dm04Callback(const plumbing_pub_sub::Person::ConstPtr &person){
    ROS_INFO("demo04订阅的数据: name:%s, age:%d, height:%0.2f", person->name.c_str(), person->age, person->height);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc, argv, "demo04_sub_person");
    // 3.实例化 ROS 句柄
    ros::NodeHandle nh;
    // 4.实例化 订阅者 对象
    ros::Subscriber sub = nh.subscribe("chatRoom02", 10, dm04Callback);
    
    ros::spin();
    return 0;
}

