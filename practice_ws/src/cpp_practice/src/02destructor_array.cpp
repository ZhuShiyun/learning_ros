/*
 * @Author: your name
 * @Date: 2023-09-21 13:25:13
 * @LastEditTime: 2023-09-21 13:35:43
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/cpp_practice/src/DestructorArray.cpp
 */

#include<iostream>

using namespace std;

class DestructorArray
{
public:
    DestructorArray(){
        
    };
    ~DestructorArray(){
        cout<<"Hello array!!"<<endl;
    };
};

int main()
{
    DestructorArray array[3];
    cout<<"end main"<<endl;
    return 0;
}


