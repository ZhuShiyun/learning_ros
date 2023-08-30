/**
 * @file guess_number_server.cpp
 * @author Birb
 * @brief
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "guess_number/GuessNumber.h"
#include "ros/ros.h"

bool isGuessCorrect(int guess, int target) { return guess == target; }

bool guessNumberCallback(guess_number::GuessNumber::Request &req,
                         guess_number::GuessNumber::Response &res) {
  /**
   *   static int targetNumber = std::rand() % 100 + 1;
   * 这行代码的前面如果去掉 static 关键字，会导致每次调用 guessNumberCallback
   *  函数时都重新生成一个新的随机数。
   * 这将影响你猜测的结果。假设你在一个猜测回合中首先生成了一个随机数，比如
   * 50。然后，当你的猜测为 70时，由于你每次进入函数时都重新生成一个新的随机数，
   * 新的随机数可能是30，这就导致了 "Too big! Try again!" 的错误提示。
   * 同样的情况也适用于你在输入 69 时收到 "Too small! Try again!" 的错误提示。
   * 通过使用 static 关键字，你确保 targetNumber只被初始化一次，
   * 并且在函数调用之间保持相同的值。
   * 这样，你才能够正确地根据同一个随机数判断猜测的结果。在你的情况下，保留
   * static 关键字是正确的做法。
   */
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  static int targetNumber = std::rand() % 100 + 1;

  res.is_correct = isGuessCorrect(req.guess, targetNumber);

  if (req.guess < targetNumber) {
    res.message = "Too small! Try again!";
  } else if (req.guess > targetNumber) {
    res.message = "Too big! Try again!";
  } else {
    res.message = "Congratulations! You guessed correctly.";
  }

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "guess_number_server");
  ros::NodeHandle nh;

  ros::ServiceServer service =
      nh.advertiseService("guess_number", guessNumberCallback);

  ROS_INFO("Guess Number Server is ready.");
  ros::spin();

  return 0;
}
