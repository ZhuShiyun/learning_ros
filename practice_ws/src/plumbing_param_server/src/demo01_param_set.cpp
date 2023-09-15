/*
 * @Author: your name
 * @Date: 2023-09-15 11:36:20
 * @LastEditTime: 2023-09-15 14:09:38
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_param_server/src/demo01_param_set.cpp
 */

/*
    需要实现参数的新增与修改
    需求：首先设置机器人的共享参数，类型，半径（0.15m)
            再修改半径（0.2m）
    实现：
    ros::NodeHandle
        setParam("键","值")
    ros::param
        set("键","值")
    修改：只需要继续调用setParam或者set，保证 键 是已经存在的，值 会覆盖。
*/

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    // 初始化ros节点
    ros::init(argc, argv, "demo01_param_set");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 参数增--------------------------
    // 方案1：nh
    nh.setParam("type", "yellowCar");
    nh.setParam("radius", 0.15);

    // 方案2：ros::param
    ros::param::set("type_param","whiteCar");
    ros::param::set("radius_param",0.16);

    // 参数改--------------------------
    // 方案1：nh
    nh.setParam("radius", 0.2);

    // 方案2：ros::param
    ros::param::set("radius_param", 0.25);

    return 0;
}

