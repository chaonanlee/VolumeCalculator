//
// Created by louis on 7/10/19.
//

#ifndef DEMO_USBCONTROLLER_H
#define DEMO_USBCONTROLLER_H

#include <fstream>
#include <sys/time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <sys/types.h>  /**/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <string.h>
#include <iostream>

class usbRelayController {
    int file_descriptor = 0;
    const unsigned char SIGNAL_CLOSE = 0x21;
    const unsigned char SIGNAL_OPEN = 0x11;
public:
    bool initPort(const char *dev);

    void closePort();

    void openPort();

    virtual ~usbRelayController() {
        close(file_descriptor);
    }
};

#endif //DEMO_USBCONTROLLER_H
