/*
 * @Author: your name
 * @Date: 2023-09-21 14:00:35
 * @LastEditTime: 2023-09-21 14:06:52
 * @LastEditors: zhushiyun
 * @Description: 代码举例二：有new和delete
 * @FilePath: /src/cpp_practice/src/04destructor_delete.cpp
 */

#include<iostream>

using namespace std;
class DestructorDelete02
{
// private:
//     /* data */
public:
    DestructorDelete02(){
        cout<<"I am constructor.."<<endl;
    };
    ~DestructorDelete02(){
        cout<<"I am destructor.."<<endl;
    };
};

int main()
{
    DestructorDelete02 d; // 创造了DestructorDelete02的对象a,调用五参的构造函数，输出第一行的“构造函数
    DestructorDelete02 * p;
    p=new DestructorDelete02();
    delete p;
    cout<<"End od main"<<endl;
    return 0;
}
