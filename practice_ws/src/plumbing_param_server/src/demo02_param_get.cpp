/*
 * @Author: your name
 * @Date: 2023-09-15 14:19:00
 * @LastEditTime: 2023-09-15 16:17:47
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_param_server/src/demo02_param_get.cpp
 */

/*
    演示参数查询
    实现：
    ros::NodeHandle------------------------------------------
        1. param(键,默认值) 
            存在，返回对应结果，否则返回默认值

        2. getParam(键,存储结果的变量)
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        3. getParamCached键,存储结果的变量)--提高变量获取效率
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        4. getParamNames(std::vector<std::string>)
            获取所有的键,并存储在参数 vector 中 

        5. hasParam(键)
            是否包含某个键，存在返回 true，否则返回 false

        6. searchParam(参数1，参数2)
            搜索键，参数1是被搜索的键，参数2存储搜索结果的变量

    ros::param-----------------------------------------------
*/

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    // 设置编码
    setlocale(LC_ALL, "");
    // 初始化ROS
    ros::init(argc, argv, "demo02_param_get");
    // 句柄
    ros::NodeHandle nh;

    // ros::NodeHandle------------------------------------------
    // 1. param(键,默认值)
    double radius = nh.param("radius", 0.5);
    ROS_INFO("param:radius = %.2f", radius);

    // 2. getParam(键,存储结果的变量)
    double radius2 = 0.0;
    bool result = nh.getParam("radius", radius2);
    // 当取一个不存在的值："Param does not exist!"
    // bool result = nh.getParam("radiusxxx", radius2);
    if (result)
    {
        ROS_INFO("getParam:r = %.2f", radius2);
     } else {
        ROS_INFO("Param does not exist!");
    }

    // 3. getParamCached键,存储结果的变量)--提高变量获取效率 cached：意思是从缓存里面取。
    /*
        原理：
            前面讲过，参数服务器通过RPC通信，而且这种通信方式效率不高；
            getParamCached的原理是看一下本地缓存里有没有获取过，如果获取过，直接从本地缓存里取，没有再远程调用。
    */ 
    std::string type;
    bool result2 = nh.getParamCached("type",type);
    if (result2)
    {
        ROS_INFO("getParamCached:type = %s", type.c_str());
     } else {
        ROS_INFO("Param does not exist!");
    }

    // 4. getParamNames(std::vector<std::string>) 获取所有的键
    std::vector<std::string> names;
    nh.getParamNames(names);
    for (auto &&name : names)
    {
        ROS_INFO("getParamNames:遍历到的元素：%s", name.c_str());
    }

    // 5. hasParam(键) 判断某个键是否存在    
    bool flag1 = nh.hasParam("radius");
    bool flag2 = nh.hasParam("radiusxxxx");
    ROS_INFO("radius 存在吗？ %d", flag1);
    ROS_INFO("radiusxxxx 存在吗？ %d", flag2);  

    // 6. searchParam(参数1，参数2) 查询参数
    std::string key;
    nh.searchParam("type", key);
    ROS_INFO("searchParam:搜索结果：%s", key.c_str()); 
    //      若找不到;
    std::string key2;
    nh.searchParam("typeeeee", key2);
    ROS_INFO("searchParam:搜索结果：%s", key2.c_str()); 

    // ros::param-----------------------------------------------和API ROS::NodeHandle差不多，键名有细微区别
    // 1. param(键,默认值)
    double radius_param = ros::param::param("radius", 100.5);
    ROS_INFO("param:radius = %.2f", radius_param);
    // 2. getParam(键,存储结果的变量)
    double radius_param2 = 0.0;
    bool result_param = ros::param::get("radius", radius_param2);
    if (result_param)
    {
        ROS_INFO("getParam:r = %.2f", radius_param2);
     } else {
        ROS_INFO("Param does not exist!");
    }
    // 3. getParamCached键,存储结果的变量)--提高变量获取效率
    std::string type_param;
    bool result_param2 = ros::param::getCached("type",type_param);
    if (result_param2)
    {
        ROS_INFO("getParamCached:type = %s", type_param.c_str());
     } else {
        ROS_INFO("Param does not exist!");
    }
    // 4. getParamNames(std::vector<std::string>) 获取所有的键
    std::vector<std::string> names_param;
    ros::param::getParamNames(names_param);
    for (auto &&name : names)
    {
        ROS_INFO("getParamNames:遍历到的元素：%s", name.c_str());
    }
    // 5. hasParam(键) 判断某个键是否存在
    bool flag_param1 = ros::param::has("radius");
    bool flag_param2 = ros::param::has("radiusxxxx");
    ROS_INFO("radius 存在吗？ %d", flag_param1);
    ROS_INFO("radiusxxxx 存在吗？ %d", flag_param2);
    // 6. searchParam(参数1，参数2) 查询参数
    std::string key_param;
    ros::param::search("type", key_param);
    ROS_INFO("searchParam:搜索结果：%s", key_param.c_str()); 
    //      若找不到;
    std::string key_param2;
    ros::param::search("typeeeee", key_param2);
    ROS_INFO("searchParam:搜索结果：%s", key_param2.c_str());

    return 0;
}
