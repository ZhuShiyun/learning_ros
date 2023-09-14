/*
    需求: 
        编写两个节点实现服务通信，客户端节点需要提交两个整数到服务器
        服务器需要解析客户端提交的数据，相加后，将结果响应回客户端，
        客户端再解析

    服务器实现:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建 服务 对象
        5.回调函数处理请求并产生响应
        6.由于请求有多个，需要调用 ros::spin()

*/
#include "ros/ros.h"
#include "plumbing_server_client/AddInts.h"

bool doReq(plumbing_server_client::AddInts::Request &request,
                         plumbing_server_client::AddInts::Response &response){
    //1.处理请求
    int num1 = request.num1;
    int num2 = request.num2;
    ROS_INFO("收到请求数据: num1 = %d, num2 = %d", num1, num2);

    //2.组织响应
    int sum = num1 + num2;
    response.sum = sum;
    ROS_INFO("sum = ", sum);

    return true;
}

int main(int argc, char *argv[])
{   
     // 2.初始化 ROS 节点
    ros::init(argc, argv, "demo01_server.cpp");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh; 
    // 4.创建 服务 对象
    ros::ServiceServer server = nh.advertiseService("AddInts", doReq);
    ROS_INFO("demo01_server已启动...");
    // 5.回调函数处理请求并产生响应
    // 6.由于请求有多个，需要调用 ros::spin()
    ros::spin();
    return 0;
}

