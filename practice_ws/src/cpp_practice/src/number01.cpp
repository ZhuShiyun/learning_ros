/*
 * @Author: your name
 * @Date: 2023-09-21 15:29:34
 * @LastEditTime: 2023-09-21 15:34:27
 * @LastEditors: zhushiyun
 * @Description: C++中setiosflags( ) 的用法-练习2 
 * `setprecision(n)`与`setiosflags(ios::fixed)`合用，可以控制小数点右边的数字个数。
 * @FilePath: /src/home/b-zhushiyun/04practice_ws/src/cpp_practice/src/number01.cpp
 */
#include<iostream>

#include<iomanip>

#include<cmath>

using namespace std;

int main() {

    double s=20.7843000;

    cout << s << endl;

    // cout << setiosflags( ios::fixed );

    cout << "setprecision( 1 )"<< setprecision( 1 )<< s << endl;

    cout << "setprecision( 2 )"<< setprecision( 2 )<< s << endl;

    cout << "setprecision( 3 )"<< setprecision( 3 )<< s << endl;

    

    cout << "setprecision( 4 )"<< setprecision( 4 )<< s << endl;

    cout << "setprecision( 5 )"<< setprecision( 5 )<< s << endl;

    cout << "setprecision( 6 )"<< setprecision( 6 )<< s << endl;
    
    cout << setiosflags( ios::fixed );

    cout << "setprecision( 7 )"<< setprecision( 7 )<< s << endl;

    cout << "setprecision( 8 )"<< setprecision( 8 )<< s << endl;

return 0;

}