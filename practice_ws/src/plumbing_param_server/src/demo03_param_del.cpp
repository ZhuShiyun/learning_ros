/*
 * @Author: your name
 * @Date: 2023-09-15 16:19:15
 * @LastEditTime: 2023-09-15 16:42:17
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_param_server/src/demo03_param_del.cpp
 */
/* 
    参数服务器操作之删除_C++实现:

    ros::NodeHandle
        deleteParam("键")
        根据键删除参数，删除成功，返回 true，否则(参数不存在)，返回 false

    ros::param
        del("键")
        根据键删除参数，删除成功，返回 true，否则(参数不存在)，返回 false


*/

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "demo03_param_del");
    ros::NodeHandle nh;

    // ros::NodeHandle----------------------------------
    bool r1 = nh.deleteParam("type_param");
    ROS_INFO("nh 删除结果:%d",r1);

    // ros::param---------------------------------------
    bool r2 = ros::param::del("type");
    if (r2)
    {
        ROS_INFO("Deleted successfully!");
    } else {
        ROS_INFO("Deleted failed!");
    }
    
    

    return 0;
}


