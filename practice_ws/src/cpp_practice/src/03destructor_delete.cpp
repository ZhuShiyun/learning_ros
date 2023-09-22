/*
 * @Author: your name
 * @Date: 2023-09-21 13:47:10
 * @LastEditTime: 2023-09-21 14:00:04
 * @LastEditors: zhushiyun
 * @Description: 析构函数和运算符delete:
                    - delete运算导致析构函数调用
                    - 栈中内存由系统自动分配和释放；使用new创建的指针对象在堆中分配内存，不需要时，需要手动释放
                代码举例一：有new出来的对象，没有delete。
 * @FilePath: /src/cpp_practice/src/03destructor_delete.cpp
 */

#include<iostream>
using namespace std;

class DestructorDelate
{
// private:
//     /* data */
public:
    DestructorDelate(){
        cout<<"I am constructor.."<<endl;
    };
    ~DestructorDelate(){
        cout<<"I am destructor.."<<endl;
    };
};

int main()
{
    DestructorDelate d; //最后的析构函数是d的析构，d是在栈中分配的内存 
    DestructorDelate *p;
    p=new DestructorDelate();//new出来的对象，用构造函数DestructorDelate()初始化 
	// 最后没有delete,该对象就不会消亡，就不会引发析构函数的调用 
    cout<<"End of main"<<endl;
    return 0;
}

