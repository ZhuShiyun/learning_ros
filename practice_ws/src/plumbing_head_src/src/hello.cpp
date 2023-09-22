/*
 * @Author: your name
 * @Date: 2023-09-19 13:24:35
 * @LastEditTime: 2023-09-19 13:42:48
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/home/b-zhushiyun/04practice_ws/src/plumbing_head_src/src/hello.cpp
 */

#include "plumbing_head_src/hello.h"
#include "ros/ros.h"

namespace hello_ns
{
    void MyHello::run(){
        ROS_INFO("源文件中的run函数");
    };
} // namespace hello_ns

