/*
 * @Author: your name
 * @Date: 2023-09-21 16:15:56
 * @LastEditTime: 2023-09-21 16:24:57
 * @LastEditors: zhushiyun
 * @Description: 利用 if语句判断小明的成绩在班里的层次（满分100）：>=90分，优秀；>=80分，良；>=60分，及格；<60,分，不及格。
 * @FilePath: /src/home/b-zhushiyun/04practice_ws/src/cpp_practice/src/01if_else.cpp
 */

#include<iostream>
using namespace std;

int main()
{
    float score;
    cout<<"输入分数："<<endl;
    cin>>score;
    float max = 100;
    float hd = 80;
    float d = 70;
    float c = 60;
    float p = 50;
    float min = 0;

    if (score > max || score < min)
    {
        cout<<"输入错误"<<endl;
    }else if (score >= hd)
    {
        cout<<"HD!"<<endl;
    }else if (score >= d)
    {
        cout<<"D"<<endl;
    }else if (score >= c)
    {
        cout<<"C"<<endl;
    }else if (score >= p)
    {
        cout<<"P"<<endl;
    }else
    {
        cout<<"F"<<endl;
    }
    
    return 0;
}

