# ROS笔记

---

资料来源：

> 古月居《ROS入门21讲》;

---



## Topic 主题/话题

### 话题消息的定义与使用

#### 如何自定义话题消息

- 定义msg文件;

```shell
$ cd catkin_ws/src/learning_topic
$ mkdir msg
$ touch Person.msg
```

​		在Person.msg中输入：注意是**uint**不是unit

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

  -  将catkin specific configuration中catkin_package(...)内的CATKIN_DEPENDS前面的注释打开，改成这样：

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

## Actionlib

> From ros.org

[actionlib_tutorials](https://wiki.ros.org/actionlib_tutorials)

- [Writing a Simple Action Server using the Execute Callback](https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29#CA-4489f75655c29f8fd70e22d2b43906c08f386f4e_1)
- [Writing a Simple Action Client](https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient)

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

- 感知：视觉传感器、图像传感器、触觉和力、惯导等。
- 认知：人工智能、知识表达、规划、任务调度、机器学习等。
- 行为：运动动力学、控制、manipulation和locomotion等。
- 数学基础：最优估计、微分几何、计算几何、运筹学等。
