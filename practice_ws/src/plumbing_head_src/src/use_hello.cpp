/*
 * @Author: your name
 * @Date: 2023-09-19 13:28:32
 * @LastEditTime: 2023-09-19 13:42:55
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/home/b-zhushiyun/04practice_ws/src/plumbing_head_src/src/use_hello.cpp
 */

#include "ros/ros.h"
#include "plumbing_head_src/hello.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "hello_head_src");

    hello_ns::MyHello myHello;
    myHello.run();
    
    return 0;
}

