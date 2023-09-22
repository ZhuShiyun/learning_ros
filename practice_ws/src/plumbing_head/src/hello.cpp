/*
 * @Author: your name
 * @Date: 2023-09-19 11:03:44
 * @LastEditTime: 2023-09-19 11:40:28
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/home/b-zhushiyun/04practice_ws/src/plumbing_head/src/hello.cpp
 */

#include"ros/ros.h"
#include"plumbing_head/hello.h"

namespace hello_ns
{
    void MyHello::run(){
        std::cout << "run函数执行......" << std::endl;
    }
} // namespace hello_ns


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "hello_head");
    hello_ns::MyHello myHello;
    myHello.run();

    return 0;
}

