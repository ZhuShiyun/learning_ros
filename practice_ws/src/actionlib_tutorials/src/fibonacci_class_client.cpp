/**
 * @file fibonacci_class_client.cpp
 * @author Birb
 * @brief
 * 当尝试注册类方法时，注册目标、活跃和反馈回调的语法并不是很方便。然而，这已经完成了，通过"超级有用但有时很难得到正确的语法"
 * boost::bind (查看boost 文档).
 * @version 0.1
 * @date 2023-08-28
 *
 * @copyright Copyright (c) 2023
 * Copy from
 * https://wiki.ros.org/cn/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client
 */

#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <ros/ros.h>

using namespace actionlib_tutorials;
typedef actionlib::SimpleActionClient<FibonacciAction> Client;

class MyNode {
 public:
  MyNode() : ac("fibonacci", true) {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
  }

  void doStuff(int order) {
    FibonacciGoal goal;
    goal.order = order;

    // Need boost::bind to pass in the 'this' pointer
    ac.sendGoal(goal, boost::bind(&MyNode::doneCb, this, _1, _2),
                Client::SimpleActiveCallback(),
                Client::SimpleFeedbackCallback());
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const FibonacciResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %i", result->sequence.back());
    ros::shutdown();
  }

 private:
  Client ac;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_fibonacci_class_client");
  MyNode my_node;
  my_node.doStuff(10);
  ros::spin();
  return 0;
}

// #include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib_tutorials/FibonacciAction.h>

// using namespace actionlib_tutorials;
// typedef actionlib::SimpleActionClient<FibonacciAction> Client;

// class MyNode
// {
// public:
//   MyNode() : ac("fibonacci", true)
//   {
//     ROS_INFO("Waiting for action server to start.");
//     ac.waitForServer();
//     ROS_INFO("Action server started, sending goal.");
//   }

//   void doStuff(int order)
//   {
//     FibonacciGoal goal;
//     goal.order = order;

//     // Need boost::bind to pass in the 'this' pointer
//     ac.sendGoal(goal,
//                 boost::bind(&MyNode::doneCb, this, _1, _2),
//                 Client::SimpleActiveCallback(),
//                 Client::SimpleFeedbackCallback());

//   }

//   void doneCb(const actionlib::SimpleClientGoalState& state,
//               const FibonacciResultConstPtr& result)
//   {
//     ROS_INFO("Finished in state [%s]", state.toString().c_str());
//     ROS_INFO("Answer: %i", result->sequence.back());
//     ros::shutdown();
//   }

// private:
//   Client ac;
// };

// int main (int argc, char **argv)
// {
//   ros::init(argc, argv, "test_fibonacci_class_client");
//   MyNode my_node;
//   my_node.doStuff(10);
//   ros::spin();
//   return 0;
// }