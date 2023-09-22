/*
 * @Author: your name
 * @Date: 2023-09-21 16:04:20
 * @LastEditTime: 2023-09-21 16:31:08
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/home/b-zhushiyun/04practice_ws/src/cpp_practice/src/01switch_case.cpp
 */
#include <iostream>
using namespace std;
int main(){
   int number;
   cout << "请输入数字查询对应星期" <<endl;
   cin >> number;
   switch(number) {
      case 1: cout<<"Monday"<<endl;
      break;
      case 2: cout<<"Tuesday"<<endl;
      break;
      case 3: cout<<"Wednesday"<<endl;
      break;
      case 4: cout<<"Thursday"<<endl;
      break;
      default: cout<<"相信我，没有这一天。"<<endl; 
      break;
   }
   return 0;
}

