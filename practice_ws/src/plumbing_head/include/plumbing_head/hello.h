/*
 * @Author: your name
 * @Date: 2023-09-19 10:52:35
 * @LastEditTime: 2023-09-19 11:40:59
 * @LastEditors: zhushiyun
 * @Description: In User Settings Edit
 * @FilePath: /src/home/b-zhushiyun/04practice_ws/src/plumbing_head/include/plumbing_head/hello.h
 */

// 先声明一下头文件保护(if not define...define...end if)
#ifndef _HELLO_H
#define _HELLO_H

/*
    声明 namespace
        |-- class
            |-- run
*/
namespace hello_ns{
    class MyHello{
        public:
            void run();
    };
};


#endif
