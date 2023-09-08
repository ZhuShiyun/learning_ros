
/*  
创建随机数生成器，判断turtle2是否被抓到，一旦抓到，给turtle2一个新的随机坐标并让turtle2到达新坐标。

    实现流程:
        1.包含头文件
        2.初始化 ros 节点
        3.创建 ros 句柄
        4.创建 TF 订阅对象
        5.处理订阅到的 TF
        6.spin

*/
//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include<turtlesim/TeleportAbsolute.h>
#include<random>


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"catch_turtle");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
    // 4.创建 TF 订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    // 5.处理订阅到的 TF

    // 需要创建发布 /turtle2/cmd_vel 的 publisher 对象

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",1000);
    
        // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 11.0);  // 生成0到11之间的随机实数

   // 定义客户端
    ros::ServiceClient client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle2/teleport_absolute");
    turtlesim::TeleportAbsolute srv;

    ros::Rate rate(10);
    while (ros::ok())
    {
        try
        {
            //5-1.先获取 turtle1 相对 turtle2 的坐标信息
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("turtle2","turtle1",ros::Time(0));

            //5-2.根据坐标信息生成速度信息 -- geometry_msgs/Twist.h
            geometry_msgs::Twist twist;

             float distent  = sqrt(pow(tfs.transform.translation.x,2) + pow(tfs.transform.translation.y,2));

            if (distent< 2)
            {

                srv.request.x = dist(gen);
                srv.request.y = dist(gen);
                srv.request.theta = dist(gen);

                ros::service::waitForService("/turtle2/teleport_absolute");

               if(client.call(srv)) 
                {
                    ROS_INFO("小偷被抓住了！");
                }
                else
                {
                    ROS_INFO("Failed to call /turtle2/teleport_absolute service");
                }
                
            }
            else
            {
                twist.linear.x =0;
                twist.angular.z = 0;
            }
            //5-3.发布速度信息 -- 需要提前创建 publish 对象
            pub.publish(twist);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("错误提示:%s",e.what());
        }
        rate.sleep();
        // 6.spin
        ros::spinOnce();
    }

    return 0;
}

