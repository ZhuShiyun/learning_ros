# 第 2 章 ROS通信机制

## 2.1 话题通信

### 2.1.2 话题通信基本操作A(C++)

#### 1.发布方 (注释版)

```c++
/*
 * @Author: your name
 * @Date: 2023-09-13 16:06:40
 * @LastEditTime: 2023-09-14 09:54:49
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_pub_sub/src/demo01_pub.cpp
 */

/*
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)

         PS: 二者需要设置相同的话题


    消息发布方:
        循环发布信息:HelloWorld 后缀数字编号

    实现流程:
        1.包含头文件 
            ROS中文本类型 ---> std_msgs/String
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 发布者 对象
        5.组织被发布的数据，并编写逻辑发布数据

*/
// 1.包含头文件
#include<ros/ros.h>
#include<std_msgs/String.h>
// 实现需求中的字符串拼接数字
#include<sstream>

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");
    //  2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"sayHi");
    // 3.实例化 ROS 句柄
    ros::NodeHandle nh;

    // 4.实例化 发布者 对象
    /*
        inline ros::Publisher ros::NodeHandle::advertise<std_msgs::String>(const std::__cxx11::string &topic, uint32_t queue_size, bool latch = false)
        还有 3 个重载

        Advertise a topic, simple version This call connects to the master to publicize that the node will be publishing messages on the given topic. This method returns a Publisher that allows you to publish a message on this topic. This version of advertise is a templated convenience function, and can be used like so ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);

        参数:
            topic – Topic to advertise on
            queue_size – Maximum number of outgoing messages to be queued for delivery to subscribers
            latch – (optional) If true, the last message published on this topic will be saved and sent to new subscribers when they connect
    */
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatRoom",10);

    // 5.组织被发布的数据，并编写逻辑发布数据
    std_msgs::String msg;
    // msg.data = "Good morning!";

    ros::Rate rate(10);
    // 循环，发布数据 
    // 设置编号
    int count = 0;

    while (ros::ok())
    {
        count++;

        // 实现需求中的字符串拼接数字
        std::stringstream ss;
        ss << "Good morning! - - - >" << count;
        msg.data = ss.str();

        // 发布话题
        pub.publish(msg);
        ROS_INFO("Data:%s",ss.str().c_str());
        rate.sleep();

        // 官方建议，用于处理回调函数
        ros::spinOnce();
        
    }
    

    return 0;
}
```

#### 2.订阅方 (注释版)

```c++
/*
 * @Author: your name
 * @Date: 2023-09-14 09:34:25
 * @LastEditTime: 2023-09-14 09:57:13
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/plumbing_pub_sub/src/demo02_sub.cpp
 */
/*
    订阅方实现：
        1.包含头文件 
            ROS中文本类型 ---> std_msgs/String
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 订阅者 对象
        6. spin
*/

// 1.包含头文件 
//     ROS中文本类型 ---> std_msgs/String
#include<ros/ros.h>
#include<std_msgs/String.h>

void dm02Callback(const std_msgs::String::ConstPtr &msg){
    // 通过msg获取并操作订阅到的数据
    ROS_INFO("demo02_sub 订阅的数据：%s", msg->data.c_str());
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
     // 2.初始化 ROS 节点:命名(唯一)
    ros::init(argc, argv, "demo02_sub");

    // 3.实例化 ROS 句柄
    ros::NodeHandle nh;

    // 4.实例化 订阅者 对象
    /**
     * ros::Subscriber subscribe<M>(const std::__cxx11::string &topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr<const M> &), const ros::TransportHints &transport_hints = ros::TransportHints())

    */
    ros::Subscriber sub = nh.subscribe("chatRoom", 10, dm02Callback);

    // 5.处理订阅到的数据

    ros::spin();

    return 0;
}

```

注意看**补充说明**

在初始化ROS节点时会给节点命名，像这样：

```c++
ros::init(argc,argv,"sayHi");
```

这个"sayHi"可以在rqt_graph中看到。

![](https://github.com/ZhuShiyun/learning_ros/blob/main/NotebookPictures/rosgraph_zXuzuo_1.png?raw=true)

解藕合的体现之一：

不同语言写的node可以交互。

### 2.1.4 话题通信自定义msg

#### 2.编辑配置文件

理解：

CMakeList.txt文件里各项配置的用途，见：[【Autolabor初级教程】ROS机器人入门】 【精准空降到 06:20】](https://www.bilibili.com/video/BV1Ci4y1L7ZZ/?p=54&share_source=copy_web&vd_source=ce7db4602809160957ede06f23353169&t=380)

-----> 也可以简单粗暴地将:

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```

理解为**编译时依赖**；

将:

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plumbing_pub_sub
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```

理解为**运行时依赖**。

### 2.1.5 话题通信自定义msg调用A(C++)

比普通pub\sub多的这一句`add_dependencies(demo03_pub_person ${PROJECT_NAME}_generate_messages_cpp)`是为了避免编译时cpp源文件在msg文件编译前编译而报错。

## 2.2 服务通信

### 2.2.3 服务通信自定义srv调用A(C++)

#### 1.服务端

测试：

```shell
$ rosservice call /AddInts [num1] [mun2]
```



#### 2.客户端

#### 3.配置 CMakeLists.txt

注意，笔记中为官网/推荐写法，实际上默认写法也可以：

```cmake
# add_dependencies(demo01_server ${PROJECT_NAME}_gencpp)
add_dependencies(demo01_server  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

#### 4.执行

**流程:**

- 需要先启动服务:`rosrun 包名 服务`
- 然后再调用客户端 :`rosrun 包名 客户端 参数1 参数2`

**结果:**

会根据提交的数据响应相加后的结果。

**注意:**

如果先启动客户端，那么会导致运行失败

**优化:**

在客户端发送请求前添加:`client.waitForExistence();`

或:`ros::service::waitForService("AddInts");`

这是一个阻塞式函数，只有服务启动成功后才会继续执行

此处可以使用 launch 文件优化，但是需要注意 args 传参特点

**----- Note -----**

- 为什么不用cin作为输入？

> 在这段代码中，没有使用 `cin >>` 的原因是它是标准输入流（stdin）的一部分，而不是命令行参数。`cin` 通常用于从终端（用户输入）读取数据，而不是用于获取命令行参数。
>
> 在这个特定的ROS节点示例中，参数似乎是在命令行上提供的，而不是通过标准输入（`cin`）来获取的。这种方式更适合在命令行中指定参数，特别是当你希望将参数作为程序的输入而不是通过交互式输入时。
>
> 如果你想要在运行程序时通过交互式输入获取参数，那么你可以使用 `cin >>`，但在这种情况下，用户必须在程序运行时手动输入参数，而不是在命令行上提供它们。这通常用于需要与用户进行交互的程序，而不是通过命令行自动化脚本。

- `ros::service::waitForService("AddInts");`和 `client.waitForExistence();`有什么区别？

> `ros::service::waitForService("AddInts");` 和 `client.waitForExistence();` 都用于等待ROS服务的存在，但它们的使用方式略有不同，具体区别如下：
>
> 1. `ros::service::waitForService("AddInts");`：
>    - 这是一个全局函数，不是特定于服务客户端的成员函数。
>    - 它用于等待名为 "AddInts" 的服务出现，并且不依赖于特定的服务客户端对象。你可以在任何地方调用这个函数，只要你知道服务的名称。
>    - 通常在程序的启动阶段使用，用于确保所需的服务在程序执行之前已经存在。
> 2. `client.waitForExistence();`：
>    - 这是一个服务客户端对象的成员函数，是针对特定服务客户端的操作。
>    - 它用于等待与服务客户端对象关联的服务存在。这意味着你必须先创建一个服务客户端对象，然后调用 `waitForExistence` 来等待与该客户端相关的服务的出现。
>    - 这通常在服务客户端的初始化阶段使用，用于确保所需的服务在客户端对象的上下文中已经存在。
>
> 总之，两者都用于等待ROS服务的存在，但根据你的需求和上下文，你可以选择使用全局函数 `ros::service::waitForService("AddInts");` 或服务客户端对象的成员函数 `client.waitForExistence();`。前者更通用，而后者更与特定的服务客户端对象相关。

## 2.3 参数服务器

### 2.3.1 参数服务器理论模型

#### 3.ROS Master 向 Listener 发送参数值

ROS Master 根据步骤2请求提供的参数名查找参数值，并将查询结果通过 RPC 发送给 Listener。

------

参数可使用数据类型:

- 32-bit integers
- booleans
- strings
- doubles
- iso8601 dates : 日期和时间的表示方法
- lists
- base64-encoded binary data : 以base64进行编码的二进制数据。
- 字典

> 注意:参数服务器不是为高性能而设计的，因此最好用于存储静态的非二进制的简单数据

### 2.3.2 参数操作A(C++)

#### 1.参数服务器新增(修改)参数 

```c++
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
```

**调试：**

```shell
$ rosparam list
/radius
/radius_param
/rosdistro
/roslaunch/uris/host_zhushiyun__35795
/rosversion
/run_id
/type
/type_param
$ rosrun plumbing_param_server demo01_param_set
$ rosparam get /type
yellowCar
$ rosparam get /radius
0.15
# 添加修改语句后：
$ rosrun plumbing_param_server demo01_param_set
$ rosparam get /radius
0.2

```

#### 2.参数服务器获取参数

```c++
/*
 * @Author: your name
 * @Date: 2023-09-15 14:19:00
 * @LastEditTime: 2023-09-15 16:14:25
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

```



#### 3.参数服务器删除参数

```c++
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

```



## 2.4 常用命令




# 第10章 ROS进阶

## 10.1 Action通信

### 10.1.1 自定义action文件

### 10.1.2 action通信自定义action文件调用A(C++) 

#### 1.服务端 代码 （注释版）

```c++
/*
 * @Author: your name
 * @Date: 2023-09-11 09:30:45
 * @LastEditTime: 2023-09-11 13:28:03
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/practice_action/src/AddInts_server.cpp
 */

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h" // 创建action服务对象必须有
#include "practice_action/AddIntsAction.h"
/*  
    需求:
        创建两个ROS节点，服务器和客户端，
        客户端可以向服务器发送目标数据N（一个整型数据）
        服务器会计算1到N之间所有整数的和，这是一个循环累加的过程，返回给客户端，
        这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，每累加一次耗时0.1s，
        为了良好的用户体验，需要服务器在计算过程中，
        每累加一次，就给客户端响应一次百分比格式的执行进度，使用action实现。

    流程:
        1.包含头文件;
        2.初始化ROS节点;
        3.创建NodeHandle;
        4.创建action服务对象;
        5.处理请求,产生反馈与响应(a.解析提交的目标值；b.产生连续反馈；c.最终结果相应) --- 回调函数；
        6.spin()回旋.

*/
//重命名actionlib::SimpleActionServer<practice_action::AddIntsAction> 成 Server, 简化代码。
typedef actionlib::SimpleActionServer<practice_action::AddIntsAction> Server;

//5.处理请求,产生反馈与响应(a.解析提交的目标值；b.产生连续反馈；c.最终结果相应) --- 回调函数；
//cb:参数一：client端提交的goal，参数二：server对象自身。
void cb(const practice_action::AddIntsGoalConstPtr &goal,Server* server){
    //a.解析提交的目标值；
    int num = goal -> num;
    ROS_INFO("客户端提交的目标值是:%d",num);

    //b.产生连续反馈；
    int result = 0;
    practice_action::AddIntsFeedback feedback;//连续反馈
    ros::Rate rate(10);//通过频率设置休眠时间 “每累加一次耗时0.1s”
    for (int i = 1; i <= num; i++)
    {
        //累加
        result += i;
        //组织连续数据并发布
        /*
            //void publishFeedback(const practice_action::AddIntsFeedback &feedback)
            practice_action::AddIntsFeedback fb;
            fb.prograss_bar = i / (double)num;
            server->publishFeedback(fb);
        */
        feedback.progress_bar = i / (double)num;
        server->publishFeedback(feedback);
        rate.sleep();
    }

    //c.最终结果相应
    practice_action::AddIntsResult r;
    r.result = result;
    server->setSucceeded(r);
    ROS_INFO("最终结果:%d",r.result);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ROS_INFO("action服务端实现");
    // 2.初始化ROS节点;
    ros::init(argc,argv,"addInts_server");
    // 3.创建NodeHandle;
    ros::NodeHandle nh;
    // 4.创建action服务对象;
    /*
    SimpleActionServer(ros::NodeHandle n, 
                        std::string name, 
                        boost::function<void (const practice_action::AddIntsGoalConstPtr &)> execute_callback, 
                        bool auto_start)
        
    参数1：NodeHandle
    参数2：话题名称
    参数3：回调函数
    参数4：是否自动启动
    
    */
    // actionlib::SimpleActionServer<practice_action::AddIntsAction> server(....); 这里在开头被简化成Server了
    Server server(nh, "addInts",boost::bind(&cb, _1, &server), false );
    //Server server(nh,  );
    server.start(); // 如果auto start为false，那么需要手动调用该函数启动服务
    // 5.处理请求,产生反馈与响应;

    // 6.spin().   
    ros::spin();
    return 0;
}



```

**PS:**

> 可以先配置CMakeLists.tx文件并启动上述action服务端，然后通过 rostopic 查看话题，向action相关话题发送消息，或订阅action相关话题的消息。

**详细步骤：**

```shell
#新终端1
$ roscore
#新终端2
$ rosnode list
/addInts_server
/rosout
$ rostopic list
/addInts/cancel
/addInts/feedback
/addInts/goal
/addInts/result
/addInts/status
/rosout
/rosout_agg
#新终端3
$ rosrun practice_action addInts_server
#新终端4 ---输入goal
$ rostopic pub /addInts/goal practice_action/AddIntsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  num: 500"   #修改这个goal
#新终端5 ---查看进度
$ rostopic echo /addInts/feedback
#新终端6 ---查看结果
$ rostopic echo /addInts/result
```

#### 2.客户端 代码 （注释版）

```c++
/*
 * @Author: your name
 * @Date: 2023-09-11 10:39:04
 * @LastEditTime: 2023-09-11 14:36:34
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/practice_action/src/addInts_client.cpp
 */

#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h> // 创建action客户端对象
#include"practice_action/AddIntsAction.h"

/*  
    需求:
        创建两个ROS节点，服务器和客户端，
        客户端可以向服务器发送目标数据N（一个整型数据）
        服务器会计算1到N之间所有整数的和，这是一个循环累加的过程，返回给客户端，
        这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，每累加一次耗时0.1s，
        为了良好的用户体验，需要服务器在计算过程中，
        每累加一次，就给客户端响应一次百分比格式的执行进度，使用action实现。

    流程:
        1.包含头文件;
        2.初始化ROS节点;
        3.创建NodeHandle;
        4.创建action客户端对象;
        5.发送请求:
            a.  连接建立；        --- 回调函数
            b. 处理连续反馈；--- 回调函数
            c. 处理最终相应；--- 回调函数
        6.spin()回旋.

*/
// 重命名
// typedef actionlib::SimpleActionClient<practice_action::AddIntsAction>Client;

//参数2：响应成功时的回调
void done_cb(const actionlib::SimpleClientGoalState &state, const practice_action::AddIntsResultConstPtr &result){
    // 判断响应状态是否成功
    if (state.state_ == state.SUCCEEDED)
    {
        ROS_INFO("响应成功，最终结果：%d",result->result);
    } else {
        ROS_INFO("任务失败！");
    }
    
}

//参数3：激活时的回调
void active_cb(){
    ROS_INFO("客户端与服务端连接建立......");
}   

//参数4：连续反馈时的回调
void feedback_cb(const practice_action::AddIntsFeedbackConstPtr &feedback){
    ROS_INFO("当前进度: %.2f", feedback->progress_bar);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //2.初始化ROS节点;
    ros::init(argc, argv, "addInts_client");
    //3.创建NodeHandle;
    ros::NodeHandle nh;
    //4.创建action客户端对象; 需要包含头文件actionlib/client/simple_action_client.h
    // Client client(nh, "addInts", true); 
    actionlib::SimpleActionClient<practice_action::AddIntsAction> client(nh, "addInts", true);
    //5.发送请求:
    // 注意：（否则可能会抛异常）
    //先等待服务启动
    ROS_INFO("等待服务器启动......");
    // 这个不行 ros::service::waitForService("addInts");
    client.waitForServer();
            // a.  连接建立；        --- 回调函数
            // b. 处理连续反馈；--- 回调函数
            // c. 处理最终相应；--- 回调函数

    /*
    void actionlib::SimpleActionClient<...>::sendGoal(const practice_action::AddIntsGoal &goal, 
                                                                                                           boost::function<...> done_cb, 
                                                                                                           boost::function<...> active_cb, 
                                                                                                           boost::function<...> feedback_cb)


    参数:
    done_cb – Callback that gets called on transitions to Done
    active_cb – Callback that gets called on transitions to Active
    feedback_cb – Callback that gets called whenever feedback for this goal is received
    */
    // 参数1：设置目标值
    practice_action::AddIntsGoal goal;
    goal.num = 100;
    client.sendGoal(goal, &done_cb, &active_cb,&feedback_cb); //参数1-4
    // 6. spin(回旋)
    ros::spin();
    return 0;
}

```

**PS:**

> 等待服务启动，只可以使用`client.waitForServer();`,之前服务中等待启动的另一种方式`ros::service::waitForService("addInts");`不适用

把`client.waitForServer();`替换成 `ros::service::waitForService("addInts");`后，提示：

```
[ INFO] [1694414132.444830888]: waitForService: Service [/addInts] has not been advertised, waiting...
```

但实际上这个API并不可用。**因为action通信底层实现上是基于topic的，而非service。**

**总结：**

1. vscode配置，可以避免不抛异常以及可以出现代码提示，推荐；
2. 服务端实现时注意两点：
   1. 关于回调函数：在创建服务端时，参数3是回调函数，用boost::bind绑定回调函数，回调函数实现时涉及到3步：a. 解析目标值；b.产生连续反馈；c.设置最终响应结果。
   2. 服务端完成后，尚未编写客户端时若想**测试服务端**，可以用rostopic相关命令测试，参考10.1.2 服务端 PS下方的内容（具体看“详细步骤”）。
3. 客户端实现时注意两点：
   1. 调用API：`client.waitForServer();` 
   2. 调用函数`sendGoal()`发送目标值，它有四个参数，第一个是封装好的目标值goal, 第2-3-4是三个回调函数，详见客户端代码注释。

## 10.2 动态参数 

动态参数：dynamic reconfigure

和**参数服务器**作比较，对比“海龟背景色修改”案例，在 2.5.4实操04_参数设置 。

动态参数应用举例：

​	1. 机器人标定时，有时需要调试 轮胎半径 和 轮间距

​	2. 导航，见[【【Autolabor初级教程】ROS机器人入门】 【精准空降到 04:37】](https://www.bilibili.com/video/BV1Ci4y1L7ZZ/?p=348&share_source=copy_web&vd_source=ce7db4602809160957ede06f23353169&t=277) 

### 10.2.1 动态参数客户端

#### 1. 建包 

```shell
$ catkin_create_pkg demo02_dr roscpp rospy std_msgs dynamic_reconfigure
```

#### 2. 添加.cfg文件

因为.cfg文件是python文件，所以需要添加可执行权限：

（在cfg文件下打开终端，输入 ll:可以看到dr.cfg文件名是白色的）

```shell
~/your_workspace/src/demo02_dr/cfg：$ chmod +x *.cfg
```

（添加可执行权限后dr.cfg文件名变绿色且名字变成dr.cfg*）



**修改.cfg文件时，建议临时将.cfg改成.py，获得自动补齐代码buff。**

mini版

```python
#! /usr/bin/env python
# -*- coding: utf-8 -*-


from dynamic_reconfigure.parameter_generator_catkin import ParameterGenerator
"""  
    动态参数客户端：
        1.导包；
        2. 创建一个参数生成器；
        3. 向参数生成器内添加参数；
        4. 配置节点， 并退出。

"""
# 2. 创建一个参数生成器；
gen = ParameterGenerator()
# 3. 向参数生成器内添加参数；
# add(name: Any, paramtype: Any, level: Any, description: Any, default: Any | None = None, min: Any | None = None, max: Any | None = None, edit_method: str = "") -> None
gen.add("int_param", int_t, 0, "整数参数", 10, 1, 100) #level: Any 是个掩码，在回调函数中使用，用来标记汉书是否被修改过，一般set 0.
# 4. 配置节点， 并退出。
# (pkgname: Any, nodename: Any, name: Any) -> None
exit(gen.generate("demo02_dr", "dr_client", "dr"))
```



#### 3.配置 CMakeLists.txt

#### 4.编译

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(demo02_dr)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
)
generate_dynamic_reconfigure_options(
  cfg/dr.cfg
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

```

