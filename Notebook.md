# ROS笔记

资料来源：

> 古月居《ROS入门21讲》;
>
> tf部分把tf1换成了tf2;

## Topic 主题/话题

### 话题消息的定义与使用

#### 如何自定义话题消息

- 定义msg文件;

```shell
$ cd catkin_ws/src/learning_topic
$ mkdir msg
$ touch Person.msg
```

		在Person.msg中输入：注意是**uint**不是unit

```html
string name
uint8 gender
uint age

uint unknown = 0
uint male = 1
uint female = 2
```

- 在package.xml中添加功能包依赖

  *添加同台生成程序功能包的依赖*

```html
<build_depend>message_generation</build_depend>   <!-- 编译依赖 -->
<exec_depend>message_runtime</exec_depend> <!-- 执行依赖 -->
```

- 在XMakeList.txt中添加编译选项

  - 在find_package的()里添加

    ```
    message generation
    ```

  - 在 Declare ROS messages, services and actions 下方添加以下配置

    目的：用来将.msg文件编译成对应的程序文件

    ```cmake
    add_message_files(FILES Person.msg)  #会将Person.msg作为消息接口
    generate_messages(DEPENDENCIES std_msgs) #确定编译前者时依赖的包
    ```

  - 将catkin specific configuration中catkin_package(...)内的CATKIN_DEPENDS前面的注释打开，改成这样：

    目的：创建运行的依赖，对应package.xml中的执行依赖

    ```cmake
    catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES learning_topic
    CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs turtlesim message_runtime
    #  DEPENDS system_lib
    )
    ```

- 编译

  ```shell
  $ cd ~/catkin_ws
  $ catkin_make
  ```

  之后会在~/catkin_ws/devel/include/learning_topic内看到Person.h

#### 如何调用自定义话题消息Person.h

- 首先创建一个Publisher和一个Subscriber

  - Publisher:

    ```c++
    #include <learning_topic/Person.h>
    #include <ros/ros.h>
    
    int main(int argc, char *argv[]) {
      // ROS节点初始化
      ros::init(argc, argv, "person_publisher");
    
      // 创建节点句柄
      ros::NodeHandle n;
    
      // 创建一个Publiser，发布名为/person_info的topic,消息类型为learning_topic::Person,
      // 队列长度10
      ros::Publisher person_info_pub =
          n.advertise<learning_topic::Person>("/person_info", 10);
    
      // 设置循环的频率
      ros::Rate loop_rate(1);
    
      int count = 0;
      while (ros::ok()) {
        // 初始化learning_topic::Person类型的消息
        learning_topic::Person person_msg;
        person_msg.name = "Wendy";
        person_msg.age = 50;
        person_msg.gender = learning_topic::Person::female;
    
        // 发布消息
        person_info_pub.publish(person_msg);
        ROS_INFO("Publish Person Info: name: %s  age:%d  gender: %d",
                 person_msg.name.c_str(), person_msg.age, person_msg.gender);
    
        // 按照循环频率延时
        loop_rate.sleep();
      }
    
      return 0;
    }
    ```

  - Subscriber:

    ```c
    #include <ros/ros.h>
    #include "learning_topic/Person.h"
    
    // 接收到订阅的消息后，会进入消息回调函数
    void personInfoCallback(const learning_topic::Person::ConstPtr& msg) {
      // 将接收到的消息打印出来
      ROS_INFO("Subscribe Person Info: name: %s  age: %d  gender: %d",
               msg->name.c_str(), msg->age, msg->gender);
    }
    
    int main(int argc, char* argv[]) {
      // 初始化ROS节点
      ros::init(argc, argv, "person_subscriber");
    
      // 创建节点句柄
      ros::NodeHandle n;
    
      // 创建一个Subscriber，订阅名为/person_info的topic，
      // 注册回调函数personInfoCallback
      ros::Subscriber person_info_sub =
          n.subscribe("/person_info", 10, personInfoCallback);
    
      // 循环等待回调函数
      ros::spin();
    
      return 0;
    }
    ```

- 配置CMakeLists.txt中的编译规则

  > 设置需要编译的代码和生成的可执行文件;
  >
  > 设置链接库;
  >
  > 添加依赖项。

```cmake
add_executable(person_publisher src/person_publisher.cpp)
target_link_libraries(person_publisher${catkin_LIBRARIES})
add_dependencies(person_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(person_subscriber src/person_subscriber.cpp)
target_link_libraries(person_subscriber${catkin_LIBRARIES})
add_dependencies(person_subscriber ${PROJECT_NAME}_generate_messages_cpp)
```

- 编译运行

  ```shell
  $ cd ~/catkin_ws
  $ catkin_make
  # 没在bashrc中配置的话记得source
  $ roscore
  $ rosrun learning_topic person_subscriber
  $ rosrun learning_topic person_publisher
  ```

------

## Service 服务

创建功能包：

```shell
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_service roscpp rospy std_msgs geometry_msgs turtlesim
```

### 客户端 Client

```shell
$ cd ~/catkin_ws/src/learning_service/src
$ touch turtle_spawn.cpp
```

如何实现一个客户端：

- 初始化ROS节点
- 创建一个Client实例
- 发布服务请求数据
- 等待Server处理之后的应答结果

```c++
/**
* 该例程将请求spawn服务，服务数据类型：turtlesim::Spawn
*/

#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char *argv[]) {
  // 初始化ros节点。
  ros::init(argc, argv, "turtle_spawn");

  // 创建节点句柄。
  ros::NodeHandle node;

  // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service。
  ros::service::waitForService("/spawn");
  ros::ServiceClient add_turtle =
      node.serviceClient<turtlesim::Spawn>("/spawn");

  // 初始化turtlesim::Spawn的请求数据。
  turtlesim::Spawn srv;  // 封装请求数据
  srv.request.x = 2.0;
  srv.request.y = 2.0;
  srv.request.name = "turtle2";

  // 请求服务调用。
  ROS_INFO("Call service to spawn turtle[x:%0.6f, y:%0.6f, name:%s]",
           srv.request.x, srv.request.y, srv.request.name.c_str());

  add_turtle.call(srv); //call是一个阻塞型的方法

  // 显示服务调用结果。
  ROS_INFO("Spawn turtle successfully [name:%s]", srv.response.name.c_str());

  return 0;
}
```

配置CMakeLists.txt中的编译规则：

> 设置需要编译的代码和生成的可执行文件;
>
> 设置链接库;

```cmake
add_executable(turtle_spawn src/turtle_spawn.cpp)
target_link_libraries(turtle_spawn
  ${catkin_LIBRARIES}
)
```

编译：

```shell
$ cd ~/catkin_ws
$ catkin_make
```

即可在~/catkin_ws/devel/lib/learning_service中看到编译生成的可执行文件turtle_spawn

```shell
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun learning_service turtle_spawn
```



### 服务端 Server

```shell
$ cd ~/catkin_ws/src/learning_service/src
$ touch turtle_command_server.cpp
```

如何实现一个服务器

- 初始化ROS节点;
- 创建Server实例;
- 循环等待服务请求，进入回调函数;
- 在回调函数中完成服务功能的处理，并反馈应答结果。

```c++
/**
*该例程将执行/turtle_command服务，服务数据类型std_srvs/Trigger
*/
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

ros::Publisher turtle_vel_pub; // 因为会在回调函数中用到，所以创建成全局的
bool pubCommand = false; // flag, 标志位，标志海龟运动/停止，默认false,即停止

// service回调函数，输入参数req,输出参数res
bool commandCallback(std_srvs::Trigger::Request &req,
                     std_srvs::Trigger::Response &res) {
  pubCommand = !pubCommand; 

  // 显示请求数据
  ROS_INFO("Publish turtle velocity command [%s]",
           pubCommand == true ? "Yes" : "No");

  // 设置反馈数据
  res.success = true;
  res.message = "Change turtle command state!";

  return true;
}

int main(int argc, char *argv[]) {
  // 初始化ROS节点
  ros::init(argc, argv, "turtle_command_server");

  // 创建节点句柄
  ros::NodeHandle n;

  // 创建一个名为/turtle_command的server,注册回调函数commandCallback
  ros::ServiceServer command_service =
      n.advertiseService("/turtle_command", commandCallback); //server端收到请求会立马跳到回调函数

  // 创建一个Publisher ,
  // 发布名为/tuetle1/cmd_vel的topic,消息类型为geometry_msgs::Twist, 队列长度10
  turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10); //发个指令让海龟动起来～

  // 循环等待回调函数
  ROS_INFO("Ready to receive turtle command.");

  // 设置循环的频率
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // 查看一次回调函数队列
    ros::spinOnce();

    // 如果标志为true,则发布速度指令
    if (pubCommand) {
      geometry_msgs::Twist vel_msg;
      vel_msg.linear.x = 0.5;
      vel_msg.angular.z = 0.2;
      turtle_vel_pub.publish(vel_msg);
    }

    // 按照循环频率延时
    loop_rate.sleep();
  }

  return 0;
}
```

配置CMakeLists.txt中的编译规则：

> 设置需要编译的代码和生成的可执行文件;
>
> 设置链接库。

```cmake
add_executable(turtle_command_server src/turtle_command_server.cpp)
target_link_libraries(turtle_command_server ${catkin_LIBRARIES})
```

编译并运行服务器：

```shell
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun learning_service turtle_command_server
$ rosservice call /turtle_command "{}"
```

### 服务数据的定义和使用

#### 如何自定义服务数据

- 定义srv文件;

  一个Person.srv文件内会有以下内容：

```
string name
uint8 age
uint8 gender

uint8 unknown = 0
uint8 male = 1
uint8 fenmale = 2
---
string result
```

关于 ---：

在编译中会产生对应的头文件

上半部分会放到PersonRequest.h里

下半部分会放到PersonResponse.h里

同时，Person.h会包括以上两者

- 在package.xml中添加功能包依赖

  *添加同台生成程序功能包的依赖*

```html
<build_depend>message_generation</build_depend>   <!-- 编译依赖 -->
<exec_depend>message_runtime</exec_depend> <!-- 执行依赖 -->
```

- 在XMakeList.txt中添加编译选项(过程参考配置msg文件)

  ```
  find_packsge( ...... message_generation)
  
  add_service_files(FILES Person.srv)
  generate_messages(DEPENDENCIES std_msgs)
  
  catkin_package( ...... message_runtime)
  ```

- 编译

之后会在~/catkin_ws/devel/include/learning_topic内看到Person.h 、PersonRequest.h 和 PersonResponse.h

(中间省略，链接：【【古月居】古月·ROS入门21讲 | 一学就会的ROS机器人入门教程】 【精准空降到 07:19】 https://www.bilibili.com/video/BV1zt411G7Vn/?p=15&share_source=copy_web&vd_source=ce7db4602809160957ede06f23353169&t=439 )

#### 如何调用自定义服务数据

编译并运行 Client和Server:

```shell
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun learning_service person_server
$ rosrun learning_service person_client
```

---

## Parameter server 参数服务器

创建功能包：

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_parameter roscpp rospy std_srvs
```

### 参数命令行使用

**rosparam**

- 列出当前所有参数

  $ rosparam list

- 显示某个数值

  $ rosparam get param_key

- 设置某个参数值

  $ rosparam set param_key param_value

- 保存参数到文件

  $ rosparam dump file_name

- 从文件读取参数

  $ rosparam load file_name

- 删除参数

  $ rosparam delete param_key

### **YAML参数文件**

（/ˈjæməl/，尾音类似*camel*骆驼, "Yet Another Markup Language"）是一个可读性高，用来表达数据[序列化](https://baike.baidu.com/item/序列化?fromModule=lemma_inlink)的格式。

```yaml
rosdistro: 'melodic

  '
roslaunch:
  uris: {host_xxz__33509: 'http://xxz:33509/'}
rosversion: '1.14.13

  '
run_id: d361776a-42e8-11ee-a791-3c91802f7611
turtlesim: {background_b: 255, background_g: 0, background_r: 192}
```

**例1.** 修改turtle背景-使用rosparam命令行：

```shell
$ rosparam set /turtlesim/background_r 192
$ rosparam set /turtlesim/background_g 0
$ rosparam set /turtlesim/background_b 255
$ rosservice call /clear "{}" 
```

**例2.** 修改turtle背景-使用YAML文件：

```shell
$ rosparam dump param.yaml  # 保存参数到文件
$ rosparam load param.yaml 
$ rosservice call /clear "{}" 
```

### 如何设置/获取参数：C++实现

> 初始化ROS节点;
>
> get获取参数;
>
> set设置参数。

以turtle为例：

```c++
/**
 * 该例程设置/读取海龟例程中的参数
 */
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <string>

int main(int argc, char **argv) {
  int red, green, blue;

  // ROS节点初始化
  ros::init(argc, argv, "parameter_config");

  // 创建节点句柄
  ros::NodeHandle node;

  // 读取背景颜色参数
  ros::param::get("/turtlesim/background_r", red);
  ros::param::get("/turtlesim/background_g", green);
  ros::param::get("/turtlesim/background_b", blue);

  ROS_INFO("Get Backgroud Color[%d, %d, %d]", red, green, blue);

  // 设置背景颜色参数
  ros::param::set("/turtlesim/background_r", 255);
  ros::param::set("/turtlesim/background_g", 255);
  ros::param::set("/turtlesim/background_b", 255);

  ROS_INFO("Set Backgroud Color[255, 255, 255]");

  // 读取背景颜色参数
  ros::param::get("/turtlesim/background_r", red);
  ros::param::get("/turtlesim/background_g", green);
  ros::param::get("/turtlesim/background_b", blue);

  ROS_INFO("Re-get Backgroud Color[%d, %d, %d]", red, green, blue);

  // 调用服务，刷新背景颜色
  ros::service::waitForService("/clear");
  ros::ServiceClient clear_background =
      node.serviceClient<std_srvs::Empty>("/clear");
  std_srvs::Empty srv;
  clear_background.call(srv);

  sleep(1);

  return 0;
}
```

配置CMakeLists.txt

```cmake
add_executable(parameter_config src/parameter_config.cpp)
target_link_libraries(parameter_config
  ${catkin_LIBRARIES}
)
```

编译并运行发布者：

```shell
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun learning_parameter parameter_config
```



---

## ROS中的坐标管理系统 - TF功能包

TF: transfer

**工具一：**`$ rosrun tf view_frames`

例：turtle跟随实验

```shell
$ sudo apt-get install ros-melodic-turtle-tf
$ roslaunch turtle_tf turtle_tf_demo.launch
$ rosrun turtlesim turtle_teleop_key
$ rosrun tf view_frames
```

- 会在主目录下生成一个frames.pdf

**工具二：**`$ rosrun tf tf_echo turtle1 turtle2`

- 可以查看两个turtle的坐标关系。

**工具三：可视化工具：**  `$ rosrun rviz rviz -d rospack find turtle_tf /rviz/turtle_rviz.rviz`

- 调出**RViz**

- 把**Displays**里的**Fixed Frame**中的设置改成**world**

## Actionlib 动作

> From ros.org

### **[actionlib_tutorials](https://wiki.ros.org/actionlib_tutorials)**

- Begineer

  - [Writing a Simple Action Server using the Execute Callback](https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29#CA-4489f75655c29f8fd70e22d2b43906c08f386f4e_1)

  - [Writing a Simple Action Client](https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient)

- Intermediate

  - [Writing a Simple Action Server using the Goal Callback Method](https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer(GoalCallbackMethod))
  - [Writing a Threaded Simple Action Client](https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient(Threaded))
  - [Running an Action Server and Client with Other Nodes](https://wiki.ros.org/actionlib_tutorials/Tutorials/RunningServerAndClientWithNodes)  ***没搞懂***

- Advanced

  - [Writing a Callback Based SimpleActionClient](https://wiki.ros.org/actionlib_tutorials/Tutorials/Writing a Callback Based Simple Action Client)  ***也没搞懂***

### Action通信接口图解：

![](https://img-blog.csdnimg.cn/3841225ccae0484eb22d779cdfdb7c12.png)

建包：

注：官网的[教程](https://wiki.ros.org/cn/actionlib_tutorials/Tutorials)没加message_runtime, 但我这边好像不加不行。。

```shell
$ catkin_create_pkg actionlib_tutorial rospy roscpp actionlib actionlib_msgs message_generation std_msgs message_runtime
```



### .action文件结构：

Goal(目标), Feedback(反馈), Result(结果)，例如：

```cmake
#goal definition
int32 samples
---
#result definition
float32 mean
float32 std_dev
---
#feedback
int32 sample
float32 data
float32 mean
float32 std_dev
```

按教程操作后，跑一下`$ rostopic echo /fibonacci/feedback`可以看到feedback里包裹的内容，部分如下：

```
---
header: 
  seq: 79
  stamp: 
    secs: 1692956026
    nsecs: 637127295
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1692956007
      nsecs: 636301473
    id: "/test_fibonacci-1-1692956007.636301473"
  status: 1
  text: "This goal has been accepted by the simple action server"
feedback: 
  sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946]
---

```

也可以用rostopic查看一下然后找找

### 如何写Action服务器

服务端负责执行Action请求，并发送反馈和结果给客户端。下面是服务端的示例代码：

```C++
//需要替换your_package和YourAction为实际Action消息的包名和消息类型。
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <your_package/YourAction.h> // 替换为你的Action消息类型

class YourActionServer {
public:
  YourActionServer(ros::NodeHandle nh, std::string name)
    : nh_(nh), as_(nh_, name, boost::bind(&YourActionServer::executeCB, this, _1), false), action_name_(name) {
    as_.start();
  }

  void executeCB(const your_package::YourGoalConstPtr& goal) {
    // 执行Action任务的代码放在这里
    // 可以通过goal对象获取客户端发送的目标信息

    // 在执行过程中发送反馈（feedback）
    your_package::YourFeedback feedback;
    feedback.status = "Executing...";
    as_.publishFeedback(feedback);

    // 在任务完成后发送结果（result）
    your_package::YourResult result;
    result.success = true;
    result.message = "Task completed successfully";
    as_.setSucceeded(result);
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<your_package::YourAction> as_;
  std::string action_name_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "your_action_server");
  ros::NodeHandle nh;

  YourActionServer server(nh, "your_action");
  ros::spin();

  return 0;
}

```

### 如何写Action客户端

客户端用于向服务端发送Action请求，并接收反馈和结果。下面是客户端的示例代码：

```c++
// 需要替换your_package和YourAction为的实际Action消息的包名和消息类型，还可以根据需要设置目标参数。
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <your_package/YourAction.h> // 替换为你的Action消息类型

int main(int argc, char** argv) {
  ros::init(argc, argv, "your_action_client");
  ros::NodeHandle nh;

  // 创建Action客户端
  actionlib::SimpleActionClient<your_package::YourAction> client("your_action", true);

  // 等待Action服务器启动
  ROS_INFO("Waiting for action server to start...");
  client.waitForServer();

  // 创建Action目标
  your_package::YourGoal goal;
  goal.target = 42; // 设置目标参数

  // 发送Action目标并等待结果
  client.sendGoal(goal);

  // 等待Action执行完成
  client.waitForResult();

  // 处理Action的结果
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Action completed successfully: %s", client.getResult()->message.c_str());
  } else {
    ROS_ERROR("Action did not complete successfully");
  }

  return 0;
}

```



## TF2

> 《古月ROS21讲》中的TF1已经不用啦！
>
> [官网]()：tf 已被弃用，取而代之的是[tf2](https://wiki.ros.org/tf2)。[tf2](https://wiki.ros.org/tf2)提供了 tf 功能的超集，实际上现在是底层的实现。如果您现在刚刚学习，强烈建议使用[tf2/Tutorials](https://wiki.ros.org/tf2/Tutorials)。

wiki: https://wiki.ros.org/tf2

tutorials: https://wiki.ros.org/tf2/Tutorials

### tf2能做什么?为什么要用tf2?

You want to **see** what tf can do instead of just reading about it? Check out the [tf2 introduction demo](https://wiki.ros.org/tf2/Tutorials/Introduction to tf2).

> 你想看看它能做什么，而不是只是阅读它?查看[tf2介绍演示](https://wiki.ros.org/tf2/Tutorials/Introduction to tf2)。

A robotic system typically has many 3D [coordinate frames](https://wiki.ros.org/geometry/CoordinateFrameConventions) that change over **time**, such as a world frame, base frame, gripper frame, head frame, etc. tf2 keeps track of all these frames over time, and allows you to ask questions like:

> 机器人系统通常有许多随**时间**变化的3D[坐标框架](https://wiki.ros.org/geometry/CoordinateFrameConventions)，如世界框架、基础框架、抓手框架、头部框架等。tf2会随时间跟踪所有这些框架，并允许你问这样的问题:

- Where was the head frame relative to the world frame, 5 seconds ago?

  > 5秒前，头部坐标系相对于世界坐标系在哪里?

- What is the pose of the object in my gripper relative to my base?

  > 我的抓手中物体相对于我的底座的姿势是什么?

- What is the current pose of the base frame in the map frame?

  > 基本框架在地图框架中的当前姿态是什么?

tf2 can operate in a **distributed system**. This means all the information about the coordinate frames of a robot is available to all ROS components on any computer in the system. Tf2 can operate with a central server that contains all transform information, or you can have every component in your distributed system build its own transform information database.

> Tf2可以在**分布式系统**中运行。这意味着关于机器人坐标框架的所有信息对于系统中任何计算机上的所有ROS组件都是可用的。Tf2可以与包含所有转换信息的中央服务器一起操作，或者您可以让分布式系统中的每个组件构建自己的转换信息数据库。

For more information on the design see [design](https://wiki.ros.org/tf2/Design)

### Tutorials

There are essentially two tasks that any user would use tf2 for, **listening for transforms** and **broadcasting transforms**.

Anyone using tf2 will need to **listen for transforms**:

- **Listening for transforms** - Receive and buffer all coordinate frames that are broadcasted in the system, and query for specific transforms between frames. Check out the writing a tf2 listener tutorial [(Python)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2 listener (Python)) [(C++)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2 listener (C%2B%2B)).

  > **监听转换** -接收并缓冲所有在系统中广播的坐标帧，并查询帧之间的特定转换。查看编写tf2侦听器教程[(Python)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2侦听器(Python)) [(c++)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2侦听器(C%2B%2B))。

To extend the *capabilities*(功能) of a robot you will need to start broadcasting transforms.

- **Broadcasting transforms** - Send out the relative pose of coordinate frames to the rest of the system. A system can have many broadcasters that each provide information about a different part of the robot. Check out the writing a tf2 broadcaster tutorial [(Python)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2 broadcaster (Python)) [(C++)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2 broadcaster (C%2B%2B)).

  > **广播变换** -发送坐标系的相对姿态到系统的其余部分。一个系统可以有许多广播器，每个广播器提供关于机器人不同部分的信息。查看编写tf2广播器教程[(Python)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2广播器(Python)) [(c++)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2广播器(C%2B%2B))。

Once you are finished with the basic tutorials, you can move on to learn about tf2 and time. The tf2 and time tutorial [(Python)](https://wiki.ros.org/tf2/Tutorials/tf2 and time (Python)) [(C++)](https://wiki.ros.org/tf2/Tutorials/tf2 and time (C%2B%2B)) teaches the basic principles of tf2 and time. The advanced tutorial about tf2 and time [(Python)](https://wiki.ros.org/tf2/Tutorials/Time travel with tf2 (Python)) [(C++)](https://wiki.ros.org/tf2/Tutorials/Time travel with tf2 (C%2B%2B)) teaches the principles of time traveling with tf2.

> 一旦你完成了基本教程，你可以继续学习tf2和time。**tf2和time教程**:[(Python)](https://wiki.ros.org/tf2/Tutorials/tf2和time (Python)) [(c++)](https://wiki.ros.org/tf2/Tutorials/tf2和time (C%2B%2B))教授tf2和time的基本原理。关于tf2和时间的**高级教程**[(Python)](https://wiki.ros.org/tf2/Tutorials/Time travel with tf2 (Python)) [(c++)](https://wiki.ros.org/tf2/Tutorials/Time travel with tf2 (C%2B%2B))教授使用tf2进行时间旅行的原理。

If you are looking for an easy tool to manually tweak tf transforms, such as for quick calibration-by-eye tuning, try [Manual TF Calibration Tools](https://wiki.ros.org/tf_keyboard_cal)

> 如果你正在寻找一个简单的工具来手动调整tf转换，如快速校准眼调，尝试[手动tf校准工具](https://wiki.ros.org/tf_keyboard_cal)

#### 静态的广播 Writing a tf2 static broadcaster (C++)

In this tutorial we will write code to publish static transforms to tf2. This is a standalone tutorial covering the basics of static transforms.

> 在本教程中，我们将编写代码向tf2发布静态转换。这是一个独立的教程，介绍静态转换的基础知识

In the next two tutorials we will write the code to reproduce the demo from the [tf2 introduction](https://wiki.ros.org/tf2/Tutorials/Introduction to tf2) tutorial. After that, the following tutorials focus on extending the demo with more advanced tf2 features.

> 在接下来的两个教程中，我们将编写代码来再现tf2介绍教程中的演示。之后，下面的教程将重点介绍使用更高级的tf2功能扩展演示。

- **建包**

```shell
$ catkin_create_pkg learning_tf2 tf2 tf2_ros roscpp rospy turtlesim
```

##### **如何broadcast transforms(广播变换)**

This tutorial teaches you how to broadcast coordinate frames to tf2. In this case, we want to broadcast the changing coordinate frames of the turtles, as they move around.

> 本教程教你如何广播坐标帧到tf2。在本例中，我们想要广播海龟移动时坐标帧的变化。

Let's first create the source files. Go to the package we just created:

> 创建source files, 在刚才创建的package的目录下：

```shell
 $ roscd learning_tf2
```

##### **Code**

Fire up your favourite editor to paste the following code into a new file called **`src/static_turtle_tf2_broadcaster.cpp`**.

```c++
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


std::string static_turtle_name;

int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_static_tf2_broadcaster");
  if(argc != 8)
  {
    ROS_ERROR("Invalid number of parameters\nusage: static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw");
    return -1;
  }
  if(strcmp(argv[1],"world")==0)
  {
    ROS_ERROR("Your static turtle name cannot be 'world'");
    return -1;

  }
  static_turtle_name = argv[1];
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = static_turtle_name;
  static_transformStamped.transform.translation.x = atof(argv[2]);
  static_transformStamped.transform.translation.y = atof(argv[3]);
  static_transformStamped.transform.translation.z = atof(argv[4]);
  tf2::Quaternion quat;
  quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
  ROS_INFO("Spinning until killed publishing %s to world", static_turtle_name.c_str());
  ros::spin();
  return 0;
};
```

##### **运行**

CMakeLists.txt

```cmake
add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
target_link_libraries(static_turtle_tf2_broadcaster  ${catkin_LIBRARIES} )
```

然后

```shell
$ catkin_make
# 新终端
$ . /opt/ros/$ROS-DISTRO/setup.bash
$ roscore
# 新终端
$ rosrun learning_tf2 static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
```

##### **检查结果**

```shell
 $ rostopic echo /tf_static
```

结果be like:

```shell
transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1459282870
        nsecs: 126883440
      frame_id: world
    child_frame_id: mystaticturtle
    transform:
      translation:
        x: 0.0
        y: 0.0
        z: 1.0
      rotation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
---
```

##### **发布静态转换的正确方式**

This tutorial aimed to show how `StaticTransformBroadcaster` can be used to publish static transforms. In your real development process you shouldn't have to write this code yourself and should privilege the use of the dedicated [tf2_ros](https://wiki.ros.org/tf2_ros) tool to do so. [tf2_ros](https://wiki.ros.org/tf2_ros) provides an executable named `static_transform_publisher` that can be used either as a commandline tool or a node that you can add to your launchfiles.

```
static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
```

- Publish a static coordinate transform to tf2 using an x/y/z offset in meters and yaw/pitch/roll in radians. (yaw is rotation about Z, pitch is rotation about Y, and roll is rotation about X).

```
static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
```

- Publish a static coordinate transform to tf2 using an x/y/z offset in meters and quaternion.

Unlike in tf, there is no period argument, and a latched topic is used.

`static_transform_publisher` is designed both as a command-line tool for manual use, as well as for use within [roslaunch](https://wiki.ros.org/roslaunch) files for setting static transforms. For example:

```xml
<launch>
<node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 1 link1_parent link1" />
</launch>
```

#### 编写tf2广播器(c++)Writing a tf2 broadcaster (C++)

已做则略：

```shell
$ catkin_create_pkg learning_tf2 tf2 tf2_ros roscpp rospy turtlesim
$ roscd learning_tf2
```

##### **Code**

Create a folder called src/ and fire up your favorite editor to paste the following code into a new file called **src/turtle_tf2_broadcaster.cpp.**

```c++
/**
 * @file turtle_tf2_broadcaster.cpp
 * @author Birb
 * @brief
 * @version 0.1
 * @date 2023-09-04
 *
 * @copyright Copyright (c) 2023
 * copy from
 * https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29
 */

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
/**
 * tf2包提供了transformbroadcast的实现，以帮助简化发布转换的任务。
 * 要使用transformbroadcast，我们需要包含tf2_ros/transform_broadcast .h头文件。
 */
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg) {
  // 在这里，我们创建了一个transformbroadcast对象，稍后我们将使用它通过网络发送转换。
  static tf2_ros::TransformBroadcaster br;

  /**
   * 这里我们创建一个Transform对象并为其提供适当的元数据。

        1.我们需要为被发布的转换提供一个时间戳，我们将使用当前时间ros::
   time::now()来标记它。

        2.然后，我们需要设置所创建链接的父框架的名称，在本例中为“world”

        3.
   最后，我们需要设置正在创建的链接的子节点的名称，在本例中，这是海龟本身的名称。
   *
   */
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = turtle_name;
  // Here we copy the information from the 3D turtle pose into the 3D transform.
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.x = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // This is where the real work is done. Sending a transform with a
  // TransformBroadcaster requires passing in just the transform itself.
  br.sendTransform(transformStamped);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  if (!private_node.hasParam("turtle")) {
    if (argc != 2) {
      ROS_ERROR("need turtle name as argument");
      return -1;
    };
    turtle_name = argv[1];
  } else {
    private_node.getParam("turtle", turtle_name);
  }

  ros::NodeHandle node;
  ros::Subscriber sub =
      node.subscribe(turtle_name + "/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

```

##### **CMakeList.txt**

修改CMakeLists.txt文件

```cmake
add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
target_link_libraries(turtle_tf2_broadcaster
 ${catkin_LIBRARIES}
)
```

然后

```shell
$ catkin_make
```

##### **创建launch文件**

If everything went well, you should have a binary file called **`turtle_tf2_broadcaster`** in your `bin` folder. If so, we're ready to create a launch file for this demo. With your text editor, create a new file called **`start_demo.launch`**, and add the following lines:

```xml
<launch>
     <!-- Turtlesim Node-->
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>

    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
    <!-- Axes -->
    <param name="scale_linear" value="2" type="double"/>
    <param name="scale_angular" value="2" type="double"/>

    <node pkg="learning_tf2" type="turtle_tf2_broadcaster"
          args="/turtle1" name="turtle1_tf2_broadcaster" />
    <node pkg="learning_tf2" type="turtle_tf2_broadcaster"
          args="/turtle2" name="turtle2_tf2_broadcaster" />

  </launch>
```

And mark the file for installation: 打开CMakeList.txt

```cmake
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
 start_demo.launch
 # myfile2
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

##### **roslaunch运行**

```shell
$ roslaunch learning_tf2 start_demo.launch
```

##### **使用tf_echo检查结果** 

```shell
$ rosrun tf tf_echo /world /turtle1
```

This should show you the pose of the first turtle. Drive around the turtle using the arrow keys (make sure your terminal window is active, not your simulator window). If you run `tf_echo` for the transform between the world and turtle 2, you should not see a transform, because the second turtle is not there yet. However, as soon as we add the second turtle in the next tutorial, the pose of turtle 2 will be broadcast to tf2.

> 这应该向你展示第一只海龟的姿势。使用箭头键驱动海龟(确保您的终端窗口处于活动状态，而不是您的模拟器窗口)。如果对世界和海龟2之间的转换运行tf_echo，则不应该看到转换，因为第二只海龟还没有出现。但是，当我们在下一个教程中添加第二只海龟时，海龟2的姿势将被广播到tf2。

To actually use the transforms broadcast to tf2, you should move on to the next tutorial about creating a tf2 listener [(Python)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2 listener (Python)) [(C++)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2 listener (C%2B%2B))

#### 添加框架（C++）Adding a frame (C++)

> **Note:** This tutorial assumes you **have completed the writing a tf2 listener tutorial** [(Python)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2 listener (Python)) [(C++)](https://wiki.ros.org/tf2/Tutorials/Writing a tf2 listener (C%2B%2B)). 

##### Why adding frames

对于许多任务来说，在局部框架内思考更容易，例如，在激光扫描仪中心的框架内对激光扫描进行推理是最容易的。Tf2允许你为系统中的每个传感器、链路等定义一个本地帧。tf2会处理所有引入的额外的帧变换。

##### Where to add frames

tf2建立一个树结构的框架;它不允许在框架结构中出现闭环。这意味着一个框架只有一个父框架，但它可以有多个子框架。目前我们的tf2树包含三个框架:world, turtle1和turtle2。这两只乌龟是世界的孩子。如果我们想要在tf2中添加一个新框架，那么三个现有框架中的一个需要成为父框架，而新框架将成为子框架。

![](https://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28C%2B%2B%29?action=AttachFile&do=get&target=tree.png)

##### How to add a frame

```shell
$ roscd learning_tf2
```

##### Code:

`src/frame_tf2_broadcaster.cpp`(和tf2 broadcaster很像)

```c++
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

   tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  
  transformStamped.header.frame_id = "turtle1";
  transformStamped.child_frame_id = "carrot1";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 2.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
        q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(10.0);
  while (node.ok()){
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }

};
```

##### CMakeLists.txt

```cmake
add_executable(frame_tf2_broadcaster src/frame_tf2_broadcaster.cpp)
target_link_libraries(frame_tf2_broadcaster
 ${catkin_LIBRARIES}
)
```

**edit the** `start_demo.launch`

```xml
<launch>
    ...
    <node pkg="learning_tf2" type="frame_tf2_broadcaster"
          name="broadcaster_frame" />
  </launch>
```

**start the turtle broadcaster demo**

```shell
$ roslaunch learning_tf2 start_demo.launch
```

##### Checking the results

So, if you drive the first turtle around, you notice that the behavior  didn't change from the previous tutorial, even though we added a new  frame. That's because adding an extra frame does not affect the other  frames, and our listener is still using the previously defined frames.  So, let's change the behavior of the listener. 

> 因此，如果您驾驶第一只海龟，您会注意到，尽管我们添加了一个新帧，但它的行为与上一个教程相比并没有改变。这是因为添加一个额外的帧不会影响其他帧，我们的监听器仍然使用先前定义的帧。所以，让我们改变听众的行为。

Open the `src/turtle_tf2_listener.cpp` file, and simple replace `"/turtle1"` with `"/carrot1"` in lines 32-33: 

```c++
transformStamped = listener.lookupTransform("/turtle2", "/carrot1",
                           ros::Time(0));
```

And now the good part: just rebuild and restart the turtle demo, and  you'll see the second turtle following the carrot instead of the first  turtle! 

```shell
$ roslaunch learning_tf2 start_demo.launch
```

执行：

```shell
$ rosrun rqt_tf_tree rqt_tf_tree
```

可以看到坐标系之间的关系如：![](/home/b-zhushiyun/birb/NotebookPictures/frames_turtle_1.png)

##### 广播移动帧Broadcasting a moving frame

The extra frame we published in this tutorial is a fixed frame that  doesn't change over time in relation to the parent frame. However, if  you want to publish a moving frame you can change the broadcaster to  change over time. Let's modify the `/carrot1` frame to change relative to `/turtle1` over time. Don't forget to put these lines within the while loop, so that the updated values get sent. 

> 我们在本教程中发布的额外框架是一个固定的框架，它不会随着时间的推移而改变与父框架的关系。但是，如果要发布移动帧，可以将广播器更改为随时间更改。让我们修改/carrot1框架，使其随时间相对于/turtle1发生变化。不要忘记将这些行放入while循环中，以便发送更新后的值。

```c++
  transformStamped.transform.translation.x = 2.0*sin(ros::Time::now().toSec());
  transformStamped.transform.translation.y = 2.0*cos(ros::Time::now().toSec());
```

完整代码为：

```c++
/**
 * @file frame_tf2_broadcaster.cpp
 * @author Birb
 * @brief
 * @version 0.1
 * @date 2023-09-06
 *
 * @copyright Copyright (c) 2023
 */

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "turtle1";
  transformStamped.child_frame_id = "carrot1";

  ros::Rate rate(10.0);
  while (node.ok()){
    // 在每个循环迭代中更新carrot1的位置
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = 2.0 * sin(ros::Time::now().toSec());
    transformStamped.transform.translation.y = 2.0 * cos(ros::Time::now().toSec());

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }

  return 0;
}

```

And now the good part: just rebuild and restart the turtle demo, and you'll see the second turtle following a moving carrot. 

```shell
 $ roslaunch learning_tf2 start_demo.launch
```



#### Learning about tf2 and time (C++)

##### tf2 and Time

In the previous tutorials we learned about how tf2 keeps track of a tree of coordinate frames. This tree changes over time, and tf2 stores a  time snapshot for every transform (for up to 10 seconds by default).  Until now we used the `lookupTransform()` function to get access to the **latest available transforms** in that tf2 tree, without knowing at what time that transform was  recorded. This tutorial will teach you how to get a transform **at a specific time**. 

> 在之前的教程中，我们了解了tf2如何跟踪坐标帧树。该树随着时间的推移而变化，tf2为每个转换存储时间快照(默认情况下最长为10秒)。到目前为止，我们使用' lookupTransform() '函数来访问tf2树中最新的可用转换，而不知道该转换是在什么时间被记录的。本教程将教你如何在特定时间获得变换。

**迷惑：**不知道为什么根据教程走下来以后，每次终端中的提示都和教程不一样：

tutorial like:

```
[ERROR] 1253918454.307455000: Extrapolation Too Far in the future: target_time is 1253918454.307, but the closest tf2  data is at 1253918454.300 which is 0.007 seconds away.Extrapolation Too Far in the future: target_time is 1253918454.307, but the closest tf2  data is at 1253918454.301 which is 0.006 seconds away. See http://pr.willowgarage.com/pr-docs/ros-packages/tf2/html/faq.html for more info. When trying to transform between /turtle1 and /turtle2. See http://www.ros.org/wiki/tf2#Frequently_Asked_Questions
```

mine like:

```
[ WARN] [1693982230.901933166]: Could NOT transform turtle2 to turtle1: Lookup would require extrapolation into the past.  Requested time 1693982227.878927864 but the earliest data is at time 1693982228.383314290, when looking up transform from frame [turtle1] to frame [turtle2]
```

但是海龟运动效果应该是一样的。

#### Time travel with tf2 (C++)

开头第一步对不上，先不管了，消化一下。

### 小练习：警察抓小偷

#### 题目：

> 1. 创建两个小海龟的TF2 Broadcaster：分别为警察和小偷创建TF2 broadcaster来广播它们的坐标变换。
> 2. 创建一个TF2 Listener：创建一个TF2 listener来监听警察和小偷的坐标变换，并计算它们之间的距离。
> 3. 实时跟踪小偷：在监听器中，通过TF2获取警察和小偷的坐标变换，并计算它们之间的距离。
> 4. 判断小偷是否被捉住：当警察距离小偷一定距离时，触发一个条件，输出信息：“小偷已被捉住！”。
> 5. 重新生成小偷的位置：一旦小偷被捉住，重新随机生成小偷的位置，并继续游戏。

cue:

```
turtle1,turtle2初始各有一个位置；turtle1假扮警察，turtle2假扮小偷；
turtle1可以操控， 当turtle1靠近turtle2时，提示“抓到小偷了！”，turtle2随机刷新一个位置，玩家可以操控turtle1可以继续追赶。
```

#### 实现步骤：

##### 建包：

```shell
$ catkin_create_pkg catch_turtle tf2 tf2_ros tf2_geometry_msgs roscpp rospy std_msgs geometry_msgs
```

##### 创建一个publisher_tf.cpp：
Tips:

可以用rosservice 相关命令探索调用一个服务需要的参数。

```c++
/*  
    实现流程:
        1.包含头文件
        2.初始化 ros 节点
        3.解析传入的命名空间
        4.创建 ros 句柄
        5.创建订阅对象
        6.回调函数处理订阅的 pose 信息
            6-1.创建 TF 广播器
            6-2.将 pose 信息转换成 TransFormStamped
            6-3.发布
        7.spin

*/
//1.包含头文件
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
//保存乌龟名称
std::string turtle_name;


void doPose(const turtlesim::Pose::ConstPtr& pose){
    //  6-1.创建 TF 广播器 ---------------------------------------- 注意 static
    static tf2_ros::TransformBroadcaster broadcaster;
    //  6-2.将 pose 信息转换成 TransFormStamped
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = turtle_name;
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0.0;
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    //  6-3.发布
    broadcaster.sendTransform(tfs);

} 

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"pub_tf");
    // 3.解析传入的命名空间
    if (argc != 2)
    {
        ROS_ERROR("请传入正确的参数");
    } else {
        turtle_name = argv[1];
        ROS_INFO("乌龟 %s 坐标发送启动",turtle_name.c_str());
    }

    // 4.创建 ros 句柄
    ros::NodeHandle nh;
    // 5.创建订阅对象
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>(turtle_name + "/pose",1000,doPose);
    //     6.回调函数处理订阅的 pose 信息
    //         6-1.创建 TF 广播器
    //         6-2.将 pose 信息转换成 TransFormStamped
    //         6-3.发布
    // 7.spin
    ros::spin();
    return 0;
}
```

##### 创建一个create_turtle.cpp生成第二只海龟：

```c++
/* 
    创建第二只小乌龟
 */
#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[])
{

    setlocale(LC_ALL,"");

    //执行初始化
    ros::init(argc,argv,"create_turtle");
    //创建节点
    ros::NodeHandle nh;
    //创建服务客户端
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    ros::service::waitForService("/spawn");
    turtlesim::Spawn spawn;
    spawn.request.name = "turtle2";
    spawn.request.x = 1.0;
    spawn.request.y = 2.0;
    spawn.request.theta = 3.12415926;
    bool flag = client.call(spawn);
    if (flag)
    {
        ROS_INFO("乌龟%s创建成功!",spawn.response.name.c_str());
    }
    else
    {
        ROS_INFO("乌龟2创建失败!");
    }

    ros::spin();

    return 0;
}

```

##### 创建catch_turtle.cpp:

```c++
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


```

##### 创建catch.launch:

用launch文件启动会方便一些

```xml
<!--
    tf2实现警察抓小偷案例
-->
<launch>
     <!-- 启动乌龟节点与键盘控制节点 -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtle1" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key" name="key_control" output="screen"/>
      <!-- 启动创建第二只乌龟的节点 -->
    <node pkg="catch_turtle" type="create_turtle" name="turtle2" output="screen" />
    <!-- 启动两个坐标发布节点 -->
    <node pkg="catch_turtle" type="publisher_tf" name="police" output="screen" args="turtle1" />
    <node pkg="catch_turtle" type="publisher_tf" name="thief" output="screen" args="turtle2" />
    <!-- 启动坐标转换节点 -->
     <node pkg="catch_turtle" type="catch_turtle" name="listener" output="screen" />
</launch>
```

##### 配置CMakeLists.txt:

```cmake
add_executable(create_turtle src/create_turtle.cpp)
target_link_libraries(create_turtle ${catkin_LIBRARIES})

add_executable(catch_turtle src/catch_turtle.cpp)
target_link_libraries(catch_turtle ${catkin_LIBRARIES})

add_executable(publisher_tf src/publisher_tf.cpp)
target_link_libraries(publisher_tf ${catkin_LIBRARIES})
```

##### 编译并在终端中运行：

```shell
$ roslaunch catch_turtle catch.launch
```

如果一切正常，现在可以达到cue里提到的效果了。

#### 其他版本-1 敌进我退

这个运用了tf2静态坐标转换等内容，并让turtle2监听turtle1的坐标，当turtle2发现turtle1靠近时，给turtle2一个相反的方向和速度, 尽量保持和turtle1相同距离，见 [practice_ws/src/tf_static](practice_ws/src/tf_static)。

#### 其他版本-2  警察躲小偷

这个只是计算了警察与小偷之间的距离并给警察一个线速度，所以每次警察遇到小偷时都会以差不多的方向快速跑开，如果把警察和小偷的行为互换，小偷最终会憋死在墙角里（注：其他版本1 敌进我退 也是这样），没有用到tf坐标转换。见 [practice_ws/src/turtle_chase](practice_ws/src/turtle_chase)


------

## Launch文件

Launch文件：通过XML文件实现多节点的配置和启动（可自动启动ROS Master）

Launch文件基础语法

```
<launch>
		<node pkg = "packsge-name" name = "node-name" type = "excutable-name" /node>		
</launch>
```

例如：simple.launch

```
<launch>
    <node pkg="learning_topic" type="person_subscriber" name="listener" output="screen" />
    <node pkg="learning_topic" type="person_publisher" name="talker" output="screen" /> 
</launch>
```

创建launch文件：

```
~/practice_ws/src$ catkin_create_pkg practice_launch
mkdir launch
gedit simple.launch
# 把simple.launch丢进去。
~/practice_ws$ catkin_make
roslaunch launch simple.launch
```

## 彩蛋内容1：学完21讲之后的路线

建议一-阅读经典T_T(真的可以读下来吗)

*此处省略经典尊名。*

建议二-练手小项目：

1. TurtleBot: https://wiki.ros.org/Robots/TurtleBot
2. Husky: https://wiki.ros.org/Robots/Husky
3. 一起从0手写URDF模型
4. 其他，如GitHub上的SmartCar等

建议三-找个真家伙

*拼个小车什么的*

建议四-找个方向挖一挖：

机器人学四个核心领域（卡大）：

- 感知：视觉传感器、图像传感器、触觉和力传感器、惯导等。
- 认知：人工智能、知识表达、规划、任务调度、机器学习等。
- 行为：运动动力学、控制、manipulation和locomotion等。
- 数学基础：最优估计、微分几何、计算几何、运筹学等。
