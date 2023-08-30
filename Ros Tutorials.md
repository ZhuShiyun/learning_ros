# ROS教程

> (Cinese) Copy from https://wiki.ros.org/cn/ROS/Tutorials
>
> For English please delete "/cn" in the link.
>
> Tutorial code exists /catkin_ws

**对于非初学者**：如果你已经足够熟悉ROS [fuerte](https://wiki.ros.org/cn/fuerte)或早期版本，只是想了解新的构建系统[catkin](https://wiki.ros.org/cn/catkin)（在[groovy](https://wiki.ros.org/cn/groovy)中引入、[hydro](https://wiki.ros.org/cn/hydro)及更新版本中被使用），你可以阅读更深入的[catkin教程](https://wiki.ros.org/cn/catkin/Tutorials)。不过，还是建议所有使用者都通读所有基本的[新手入门教程](https://wiki.ros.org/cn/ROS/Tutorials#ROS.2BaDhfw2VZegs-)以便探索最新特性。

**对于Linux新手**：先学习下Linux常用命令行工具可能会更有帮助。[这里](http://www.ee.surrey.ac.uk/Teaching/Unix/)有个不错的快速教程。

> **目录**
>
> 1. ROS教程
>    1. ROS核心教程
>       1. [初级教程](https://wiki.ros.org/cn/ROS/Tutorials#A.2BUh1.2Bp2VZegs-)
>       2. [中级教程](https://wiki.ros.org/cn/ROS/Tutorials#A.2BTi1.2Bp2VZegs-)
>    2. [ROS标准](https://wiki.ros.org/cn/ROS/Tutorials#ROS.2BaAdRxg-)
>    3. [其他ROS库的教程](https://wiki.ros.org/cn/ROS/Tutorials#A.2BUXZO1g-ROS.2BXpN2hGVZegs-)
>    4. [提供ROS接口的库的教程](https://wiki.ros.org/cn/ROS/Tutorials#A.2BY9BPmw-ROS.2BY6VT43aEXpN2hGVZegs-)
>    5. 外部ROS资源
>       1. [外部教程](https://wiki.ros.org/cn/ROS/Tutorials#A.2BWRaQ6GVZegs-)
>       2. [外部研讨会和讲座](https://wiki.ros.org/cn/ROS/Tutorials#A.2BWRaQ6HgUi6hPGlSMi7Jepw-)
>    6. [在你自己的机器人上使用ROS](https://wiki.ros.org/cn/ROS/Tutorials#A.2BVyhPYIHqXfF2hGc6VmhOuk4KT391KA-ROS)

## 1. ROS核心教程

### 1.1 初级教程

1. [安装和配置ROS环境](https://wiki.ros.org/cn/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

   本教程将指导您在计算机上安装ROS和配置ROS环境。

2. [ROS文件系统导览](https://wiki.ros.org/cn/ROS/Tutorials/NavigatingTheFilesystem)

   本教程介绍ROS文件系统的概念，包括如何使用roscd、rosls和[rospack](https://wiki.ros.org/rospack)命令行工具。

3. [创建ROS软件包](https://wiki.ros.org/cn/ROS/Tutorials/CreatingPackage)

   本教程介绍如何使用[roscreate-pkg](https://wiki.ros.org/roscreate)或[catkin](https://wiki.ros.org/catkin)创建新的ROS软件包，并使用[rospack](https://wiki.ros.org/rospack)列出软件包的依赖关系。

4. [构建ROS软件包](https://wiki.ros.org/cn/ROS/Tutorials/BuildingPackages)

   本教程介绍了构建软件包及使用的工具链。

5. [理解ROS节点](https://wiki.ros.org/cn/ROS/Tutorials/UnderstandingNodes)

   该教程介绍了ROS[图](https://wiki.ros.org/cn/ROS/Concepts#ROS.2Bi6F7l1b.2BXEJrIQ-)的概念，并探讨了[roscore](https://wiki.ros.org/roscore)、[rosnode](https://wiki.ros.org/rosnode)和[rosrun](https://wiki.ros.org/rosrun)命令行工具的使用。

6. [理解ROS话题](https://wiki.ros.org/cn/ROS/Tutorials/UnderstandingTopics)

   本教程介绍了ROS话题，以及如何使用[rostopic](https://wiki.ros.org/rostopic)和[rqt_plot](https://wiki.ros.org/rqt_plot)命令行工具。

7. [理解ROS服务和参数](https://wiki.ros.org/cn/ROS/Tutorials/UnderstandingServicesParams)

   本教程介绍了ROS服务和参数的知识，以及命令行工具[rosservice](https://wiki.ros.org/rosservice)和[rosparam](https://wiki.ros.org/rosparam)的使用方法。

8. [使用rqt_console和roslaunch](https://wiki.ros.org/cn/ROS/Tutorials/UsingRqtconsoleRoslaunch)

   本教程介绍在ROS中使用[rqt_console](https://wiki.ros.org/rqt_console)和[rqt_logger_level](https://wiki.ros.org/rqt_logger_level)进行调试，以及使用[roslaunch](https://wiki.ros.org/roslaunch)同时启动多个节点。

9. [使用rosed在ROS中编辑文件](https://wiki.ros.org/cn/ROS/Tutorials/UsingRosEd)

   本教程展示了如何使用[rosed](https://wiki.ros.org/rosbash)来简化编辑过程。

10. [创建ROS消息和服务](https://wiki.ros.org/cn/ROS/Tutorials/CreatingMsgAndSrv)

    本教程介绍如何创建和构建msg和srv文件，以及[rosmsg](https://wiki.ros.org/rosmsg)、rossrv和roscp命令行工具的使用。

11. [编写简单的发布者和订阅者（C++）](https://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B))

    本教程介绍如何用C++编写发布者和订阅者节点。

12. [编写简单的发布者和订阅者（Python）](https://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber(python))）

    本教程介绍如何用Python编写发布者和订阅者节点。

13. [检验简单的发布者和订阅者](https://wiki.ros.org/cn/ROS/Tutorials/ExaminingPublisherSubscriber)

    本教程将介绍如何运行及测试发布者和订阅者。

14. [编写简单的服务和客户端（C++）](https://wiki.ros.org/cn/ROS/Tutorials/WritingServiceClient(c%2B%2B))

    本教程介绍如何用C++编写服务和客户端节点。

15. [编写简单的服务和客户端（Python）](https://wiki.ros.org/cn/ROS/Tutorials/WritingServiceClient(python))

    本教程介绍如何用Python编写服务和客户端节点。

16. [检验简单的服务和客户端](https://wiki.ros.org/cn/ROS/Tutorials/ExaminingServiceClient)本教程将介绍如何运行及测试服务和客户端。

17. [录制和回放数据](https://wiki.ros.org/cn/ROS/Tutorials/Recording and playing back data)

    教你如何将正在运行的ROS系统中的数据记录到一个bag文件中，然后通过回放这些数据来来重现相似的运行过程。

18. [从bag文件中读取消息](https://wiki.ros.org/cn/ROS/Tutorials/reading msgs from a bag file)

    了解从bag文件中读取所需话题的消息的两种方法，以及`ros_readbagfile`脚本的使用。

19. r[roswtf入门](https://wiki.ros.org/cn/ROS/Tutorials/Getting started with roswtf)

    简单介绍了[roswtf](https://wiki.ros.org/roswtf)工具的基本使用方法。

20. [探索ROS维基](https://wiki.ros.org/cn/ROS/Tutorials/NavigatingTheWiki)

    本教程介绍了ROS维基([wiki.ros.org](https://wiki.ros.org/Documentation))的组织结构以及使用方法。同时讲解了如何才能从ROS维基中找到你需要的信息。

21. [接下来做什么？](https://wiki.ros.org/cn/ROS/Tutorials/WhereNext)

    本教程将讨论获取更多知识的途径，以帮助你更好地使用ROS搭建真实或虚拟机器人。

**现在你已经完成了初级教程，有兴趣的话可以填写下这个简短的[问卷](http://spreadsheets.google.com/viewform?formkey=dGJVOVhyXzd0b0YxRHAxWDdIZmo4cGc6MA)。**

### 1.2 中级教程

更多的客户端API教程可以在相应的软件包中找到（[roscpp](https://wiki.ros.org/roscpp/Tutorials)，[rospy](https://wiki.ros.org/rospy/Tutorials)，[roslisp](https://wiki.ros.org/roslisp/Tutorials)）。

1. [手动创建ROS package](https://wiki.ros.org/cn/ROS/Tutorials/Creating a Package by Hand)

   本教程将展示如何手动创建ROS package

2. [管理系统依赖项](https://wiki.ros.org/cn/ROS/Tutorials/rosdep)

   本教程将展示如何使用[rosdep](https://wiki.ros.org/rosdep)安装系统依赖项.

3. [Roslaunch在大型项目中的使用技巧](https://wiki.ros.org/cn/ROS/Tutorials/Roslaunch tips for larger projects)

   本教程主要介绍roslaunch在大型项目中的使用技巧。重点关注如何构建launch文件使得它能够在不同的情况下重复利用。我们将使用 2dnav_pr2 package作为学习案例。

4. [ROS在多机器人上的使用](https://wiki.ros.org/cn/ROS/Tutorials/MultipleMachines)

   本教程将展示如何在两台机器上使用ROS系统，详述了使用`ROS_MASTER_URI`来配置多台机器使用同一个master。

5. [自定义消息](https://wiki.ros.org/cn/ROS/Tutorials/DefiningCustomMessages)

   本教程将展示如何使用ROS [Message Description Language](https://wiki.ros.org/ROS/Message_Description_Language)来定义你自己的消息类型.

6. [在python中使用C++类](https://wiki.ros.org/cn/ROS/Tutorials/Using a C%2B%2B class in Python)

   本教程阐述一种在python中使用C++类的方法。

7. [如何编写教程](https://wiki.ros.org/cn/WritingTutorials)

   （概述：）本教程介绍在编辑[ros.org](https://wiki.ros.org/Documentation)维基时可以用到的模板和宏定义，并附有示例以供参考。

## 2. ROS标准

- [ROS开发者指南](https://wiki.ros.org/cn/DevelopersGuide) 有关代码风格和软件包布局等相关准则
- [Standard Units of Measure and Coordinate Conventions](http://www.ros.org/reps/rep-0103.html) 标准计量单位和坐标约定

## 3. 其他ROS库的教程

- [Robot Model](https://wiki.ros.org/robot_model_tutorials)
- [Visualization](https://wiki.ros.org/visualization/Tutorials)
- [actionlib](https://wiki.ros.org/cn/actionlib_tutorials/Tutorials)
- [Pluginlib](https://wiki.ros.org/pluginlib/Tutorials)
- [Nodelets](https://wiki.ros.org/nodelet/Tutorials)
- [Navigation](https://wiki.ros.org/cn/navigation/Tutorials)
- [ROS-Industrial Tutorials](https://wiki.ros.org/Industrial/Tutorials)
- [Dynamixel Tutorials](https://wiki.ros.org/dynamixel_controllers/Tutorials)

## 4. 提供ROS接口的库的教程

- [Stage](https://wiki.ros.org/stage/Tutorials)
- [TF](https://wiki.ros.org/tf/Tutorials)
- [PCL with ROS](https://wiki.ros.org/pcl/Tutorials)

## 5.外部ROS资源

### 5.1外部教程

- [创客智造](https://www.ncnynl.com/) 中文机器人教程
- [New Course on Udemy: Milan Yadav, "ROS Tutorials"](https://www.udemy.com/course/ros-tutorials/?referralCode=CE60EFAB8BA458EC024C) (English)
- [Sıfırdan Uygulamalı ROS Eğitimi-Udemy](https://www.udemy.com/course/sifirdan-uygulamali-ros-egitimi/?referralCode=F2F90CAFA1EEF6F5D5E9) (Turkish Language)
- [RobotsForRobots Tutorials and ROS Explained Videos](https://www.youtube.com/channel/UCZT16dToD1ov6lnoEcPL6rw)
- [Temel ROS Eğitimi](https://www.udemy.com/course/temel-ros-egitimi/?referralCode=B859A5B86CFB048D9539) (Turkish Language)
- [ROS - Urdf ve Xacro ile Robot Modelleme](https://www.udemy.com/course/ros-ile-robot-modelleme/?referralCode=41BA44CA4E7906D0B6EC) (Turkish Language)
- [Uygulamalar ile ROS Eğitimi](https://www.udemy.com/course/uygulamalar-ile-ros-egitimi/?referralCode=3E1DB3387CAC346C4B74) (Turkish Language)
- [Course on Udemy: Anis Koubaa, "ROS for Beginners: Localization, Navigation, and SLAM"](http://www.riotu-lab.org/udemy.php#rosnav) (NEW)
- [Course on Udemy: Anis Koubaa, "ROS2 How To: Discover Next Generation ROS"](http://www.riotu-lab.org/udemy.php#ros2), the first online course on ROS2
- [Course on Udemy: Anis Koubaa, "ROS for Beginners: Basics, Motion, and OpenCV"](http://www.riotu-lab.org/udemy.php#ros) Highest Rated
- [ROS Online Courses Library](https://goo.gl/U8op5X)
- [ROS Weekly LIVE-Class](https://goo.gl/ZfWwkc)
- [Udemy Course on ROS: Video tutorials on learning to program robots from scratch](https://www.udemy.com/ros-basics-program-robots/)
- [Online ROS Tutorials:Learn ROS by programming online simulated robots](https://goo.gl/Nav9Vh)
- [ROS Q&A Videos Tutorials](https://goo.gl/fk2DNU)
- [ROS Tutorial Video Demos at ANU](http://www.youtube.com/playlist?list=PLDC89965A56E6A8D6)
- [NooTriX Step-by-Step ROS Tutorials](http://nootrix.com/category/robotics/robots-software/)
- [Clearpath Robotics' knowledge base](http://support.clearpathrobotics.com/)
- [Erle Robotics - Learning ROS](https://www.youtube.com/watch?v=d5YAJh6Z2B0&list=PL39WpgKDjDfVfiNVG47DBi93wsh2XHKVO)
- [ROS-Industrial Training Class Curriculum](http://ros-industrial.github.io/industrial_training)
- [Jonathan Bohren's ROS Tutorials](http://jbohren.com/tutorials/)
- [An Introduction to Robot Operating System (ROS)](https://www.allaboutcircuits.com/technical-articles/an-introduction-to-robot-operating-system-ros/)
- [Programming Robots Using ROS: An introduction](https://atadiat.com/ar/articles/programming-robot-operating-system-introduction/) (Arabic Language)
- [Learn ROS using a URDF simulation model from basics through SLAM - by Husarion](https://husarion.com/tutorials/ros-tutorials/1-ros-introduction/)
- [Learn and Develop for Robots using ROS](http://iranros.com/) (Persian Language)
- [ROS Tutorial for Beginners, a YouTube playlist](https://www.youtube.com/playlist?list=PLoGH52eUIHsfz-cz48haqCCMOpMpZyUtq) (Arabic Language)
- [How to Install ROS Melodic in Ubuntu](https://www.youtube.com/watch?v=WKlk_2EGfM4)
- [ROS2 on IBM Cloud Kubernetes](https://medium.com/@mahmoudmnasr95/running-ros2-on-ibm-cloud-1b1284cbd487)

### 5.2 外部研讨会和讲座

- [Short course on ROS programming 2020](http://mediawiki.isr.ist.utl.pt/wiki/Short_course_on_ROS_programming_2020) by [Institute for Systems and Robotics - Lisbon](https://welcome.isr.tecnico.ulisboa.pt/) of [Técnico](https://tecnico.ulisboa.pt/en/)
- [ROS Meetup](https://www.meetup.com/meetup-group-GLGBcAgn/) by [The Construct](http://www.theconstructsim.com/)
- [Free introductory seminar for enterprises](http://opensource-robotics.tokyo.jp/?p=355&lang=en) by [TORK](http://opensource-robotics.tokyo.jp/) in Tokyo

## 6. 在你自己的机器人上使用ROS

- [Create your own URDF file](https://wiki.ros.org/urdf/Tutorials) 创建自定义通用机器人描述格式（URDF）文件
- [ros_control](https://wiki.ros.org/ros_control) 使用ROS的标准控制器框架来与硬件交互
- [Using a URDF in Gazebo](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros) 添加必要的标签让你的机器人进入Gazebo机器人模拟器
- [Setting up MoveIt!](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) 创建配置包来使用[MoveIt!](https://wiki.ros.org/moveit)运动规划框架
- [Running ROS accross multiple REMOTE machines](https://wiki.ros.org/ROS/Tutorials/MultipleRemoteMachines) 适用于户外移动机器人的应用
- [Bringing ROS to real life: Barista](https://goo.gl/C4ZLA5) 世界上第一个为餐桌端咖啡的服务机器人
- [Pilz robot manipulator PRBT](https://wiki.ros.org/pilz_robots/Tutorials) 建模您的应用程序并控制一个pilz轻量级机械手模块PRBT6