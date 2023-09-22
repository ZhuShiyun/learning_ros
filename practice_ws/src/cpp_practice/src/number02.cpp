/*
 * @Author: your name
 * @Date: 2023-09-21 15:37:59
 * @LastEditTime: 2023-09-21 15:45:06
 * @LastEditors: zhushiyun
 * @Description: C++中setiosflags( ) 的用法-练习3:setiosflags(ios::fixed|ios::showpoint) 
 * @FilePath: /src/home/b-zhushiyun/04practice_ws/src/cpp_practice/src/number02.cpp
 */

#include<iostream>

#include<iomanip>

#include<cmath>

using namespace std;

int main() {

    double s=20.7843000;

    cout << s << endl;

    cout << setiosflags( ios::fixed);

    cout << "setprecision( 0 )"<< setprecision( 0 )<< s << endl;

    cout << "setprecision( 1 )"<< setprecision( 1 )<< s << endl;

    cout << "setprecision( 2 )"<< setprecision( 2 )<< s << endl;

    cout << "setprecision( 7 )"<< setprecision( 7 )<< s << endl;

    cout << setiosflags( ios::fixed|ios::showpoint );

    cout << "setprecision( 0 )"<< setprecision( 0 )<< s << endl;

    cout << "setprecision( 1 )"<< setprecision( 1 )<< s << endl;

    cout << "setprecision( 2 )"<< setprecision( 2 )<< s << endl;

    cout << "setprecision( 3 )"<< setprecision( 3 )<< s << endl;

return 0;

}
