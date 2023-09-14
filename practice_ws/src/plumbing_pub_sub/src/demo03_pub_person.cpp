/*
 * @Author: your name
 * @Date: 2023-09-14 11:24:51
 * @LastEditTime: 2023-09-14 13:33:53
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_pub_sub/src/demo03_pub_person.cpp
 */

/*
    1.包含头文件 
            #include "plumbing_pub_sub/Person.h"
    2.初始化 ROS 节点:命名(唯一)
    3.实例化 ROS 句柄
    4.实例化 发布者 对象
    5.组织被发布的数据，并编写逻辑发布数据

*/

#include"ros/ros.h"
#include "plumbing_pub_sub/Person.h"

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");
    // 2. 初始化 ROS 节点；
    ros::init(argc, argv, "demo03_pub_person");
    // 3. 创建节点句柄；
    ros::NodeHandle nh;
    // 4. 实例化发布者对象；
    ros::Publisher pub = nh.advertise<plumbing_pub_sub::Person>("chatRoom02",10);
    // 5.编写发布逻辑，发布数据
    // 5-1. 创建被发布的数据
    plumbing_pub_sub::Person person;
    person.name = "Ally";
    person.age = 20;
    person.height = 159.7;
    // 5-2. 设置发布频率
    ros::Rate rate(1);
    // 5-3. 循化发布数据
    while (ros::ok())
    {   
        // 让年龄递增（下次还是身高吧..）
        person.age += 1;
        // biu!
        pub.publish(person);
        rate.sleep();
        // “建议持有”
        ros::spinOnce();
    }
    
    
    return 0;
}

