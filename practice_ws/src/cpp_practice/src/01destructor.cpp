/*
 * @Author: your name
 * @Date: 2023-09-21 09:42:48
 * @LastEditTime: 2023-09-21 11:01:19
 * @LastEditors: zhushiyun
 * @Description: 析构函数：析构函数(destructor) 与构造函数相反，当对象结束其生命周期，如对象所在的函数已调用完毕时，系统自动执行析构函数。
 * 析构函数往往用来做“清理善后” 的工作（例如在建立对象时用new开辟了一片内存空间，delete会自动调用析构函数后释放内存）。
 * @FilePath: /src/home/b-zhushiyun/04practice_ws/src/cpp_practice/src/xi.cpp
 */


#include<iostream>

using namespace std;

class Destructor
{
public:
    Destructor(){
        cout<<"Constructor(构造函数)"<<endl;
    }
    ~Destructor(){
        cout<<"Destructor(析构函数)"<<endl;
    }
}; 

int main()
{
    Destructor d;
    return 0;
}//在对象消失时，析构函数自动被调用，释放对象占用的空间



