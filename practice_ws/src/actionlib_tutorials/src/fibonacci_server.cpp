/**
 * @file fibonacci_server.cpp
 * @author Birb
 * @brief  copy from
 * https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29#CA-4489f75655c29f8fd70e22d2b43906c08f386f4e_1
 * @version 0.1
 * @date 2023-08-25
 *
 * @copyright Copyright (c) 2023
 *
 */

// /**
//  * @brief
//  * 从实现简单行为中使用的行为库actionlib/server/simple_action_server.h。
//  */
// #include <actionlib/server/simple_action_server.h>
// //
// 这个包含了从以上Fibonacci.action文件中生成的行为消息。这是从FibonacciAction.msg文件中自动生成的。
// // 对于更多关于消息定义的细节，查看msg界面.
// #include <actionlib_tutorials/FibonacciAction.h>
// #include <ros/ros.h>

// class FibonacciAction {
//   /*这些是行为类中受保护的变量。在创建行为的过程中，构造node
//    * handle并传递到行为服务器中。
//    *
//    在行为的构造函数中构造行为服务器，并且关于行为服务器将会在以下讲述。创建反馈和结果消息用于在行为中发布。
//    */
//  protected:
//   ros::NodeHandle nh_;
//   actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction>
//       as_;  // NodeHandle instance must be created before this line.
//       Otherwise
//             // strange error occurs.
//   std::string action_name_;
//   // create messages that are used to published feedback/result
//   actionlib_tutorials::FibonacciFeedback feedback_;
//   actionlib_tutorials::FibonacciResult result_;

//   /**
//    * @brief
//    * 在行为构造函数中，行为服务器会被创建。行为服务器会得到一个节点句柄（node
//    * handle）、行为名称和选择一个运行回调函数（executeCB）参数。
//    * 在这个例子中，创建的行为服务器将回调函数（executeCB）作为参数。
//    *
//    */
//  public:
//   FibonacciAction(std::string name)
//       : as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1),
//             false),
//         action_name_(name) {
//     as_.start();
//   }

//   ~FibonacciAction(void) {}

//   /**
//    * @brief
//    *
//    现在调用的executeCB函数以及在构造函数中创建。回调函数会传递一个指向目标消息的指针。
//    *  注意: 这是一个boost共享指针,
//    在目标消息类型最后附加一个给定的"ConstPtr".
//    * @param goal
//    */
//   void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr& goal) {
//     /**
//      * @brief 在行为中创建内部参数。
//      *  在这个例程中，发布ROS_INFO来让用户指定行为正在运行。
//      */
//     // helper variables
//     ros::Rate r(1);
//     bool success = true;

//     // push_back the seeds for the fibonacci sequence
//     feedback_.sequence.clear();
//     feedback_.sequence.push_back(0);
//     feedback_.sequence.push_back(1);

//     // publish info to the console for the user
//     ROS_INFO(
//         "%s: Executing, creating fibonacci sequence of order %i with seeds
//         %i, "
//         "%i",
//         action_name_.c_str(), goal->order, feedback_.sequence[0],
//         feedback_.sequence[1]);

//     /**
//      * @brief
//      * 一个行为服务器的一个重要组成部分是允许行为客户端请求取消当前目标执行。
//      * 当一个客户端请求抢占当前目标是，行为服务器应该取消目标，随后执行重要的清理，然后调用函数setPreempted()，该函数会发出该行为已经被用户请求抢占信号。
//      * 设置检查抢占请求服务器的等级到服务器系统。
//      *
//      * @param i
//      */
//     // start executing the action
//     for (int i = 1; i <= goal->order; i++) {
//       // check that preempt has not been requested by the client
//       if (as_.isPreemptRequested() || !ros::ok()) {
//         ROS_INFO("%s: Preempted", action_name_.c_str());
//         // set the action state to preempted
//         as_.setPreempted();
//         success = false;
//         break;
//       }
//       feedback_.sequence.push_back(feedback_.sequence[i] +
//                                    feedback_.sequence[i - 1]);

//       /**
//        * @brief
//        * 这里，Fibonacci序列赋值给feedback变量，然后通过行为服务器提供的反馈频道发布出去。随后行为继续循环和发布反馈。
//        *
//        */
//       // publish the feedback
//       as_.publishFeedback(feedback_);
//       // this sleep is not necessary, the sequence is computed at 1 Hz for
//       // demonstration purposes
//       r.sleep();
//     }

//     /**
//      * @brief
//      *  一旦行为完成计算Fibonacci序列，会通知行为客户端操作设置成功。
//      */
//     if (success) {
//       result_.sequence = feedback_.sequence;
//       ROS_INFO("%s: Succeeded", action_name_.c_str());
//       // set the action state to succeeded
//       as_.setSucceeded(result_);
//     }
//   }
// };

// /**
//  * @brief
//  *  最后main函数，创建行为然后spins节点。行为会运行并等待接收目标。
//  * @param argc
//  * @param argv
//  * @return int
//  */
// int main(int argc, char** argv) {
//   ros::init(argc, argv, "fibonacci");

//   FibonacciAction fibonacci("fibonacci");
//   ros::spin();

//   return 0;
// }

#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <ros/ros.h>

class FibonacciAction {
 protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction>
      as_;  // NodeHandle instance must be created before this line. Otherwise
            // strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;

 public:
  FibonacciAction(std::string name)
      : as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1),
            false),
        action_name_(name) {
    as_.start();
  }

  ~FibonacciAction(void) {}

  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr& goal) {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO(
        "%s: Executing, creating fibonacci sequence of order %i with seeds %i, "
        "%i",
        action_name_.c_str(), goal->order, feedback_.sequence[0],
        feedback_.sequence[1]);

    // start executing the action
    for (int i = 1; i <= goal->order; i++) {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] +
                                   feedback_.sequence[i - 1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for
      // demonstration purposes
      r.sleep();
    }

    if (success) {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}