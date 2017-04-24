#include "ecthread.h"

#include <stdio.h>   /*标准输入输出定义*/
#include <stdlib.h>  /*标准函数库定义*/
#include <unistd.h>  /*UNIX标准函数定义*/
#include <sys/types.h>  /*基本系统数据类型*/
#include <sys/stat.h>   /*unix/linux系统定义文件状态*/
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h>  /*PPSIX 终端控制定义*/
#include <errno.h>    /*错误号定义*/
#include <string.h>

ecThread::ecThread(QObject *parent) :
    QThread(parent)
{
}

int ecThread::openGPIO()
{
    int fd;
    /*int open（“dev_name”，int open_Status）   成功返回文件描述符，如果失败返回-1*/

    fd = open("/dev/Encoder", O_RDWR);      //读写方式打开GPIO

    if(fd == -1)  //g_iofd == -1 不能打开GPIO
    {
        perror("Can Not Open GPIO！\n");
        return -1;
    }

    printf("GPIO configure complete\n");

    return fd;
}

int ecThread::PluseCounting(int g_pulse, int g_direct)
{
    int result;

    if(g_direct == 1){
        result = g_pulse;
    }
    else
        result = -g_pulse;

    return result;
}

int ecThread::ClearCounter(int fd, int Num)
{
    int a;
    ec_temp[Num] = 0;
    a = write(fd,ec_temp,sizeof(ec_temp));

    return a;
}

void ecThread::run()
{
    ec_fd = openGPIO();

    while(1)
    {
        if(ec_fd >= 0)
        {
            if(read(ec_fd,ec_temp,sizeof(ec_temp)) == sizeof(ec_temp))
            {
                    g_LeftPulse += PluseCounting(ec_temp[LEFT_PLUSE],ec_temp[LEFT_DIRECTION]);

                    g_LeftDistance = (double)g_LeftPulse/PLUSES_PER_CIRCLE * PI * WHEEL_DIAMETER;

                    g_LeftPulseDisplay += PluseCounting(ec_temp[LEFT_PLUSE],ec_temp[LEFT_DIRECTION]);

                    ClearCounter(ec_fd,LEFT_PLUSE);


                    g_RightPulse += PluseCounting(ec_temp[RIGHT_PLUSE],ec_temp[RIGHT_DIRECTION]);

                    g_RightDistance = (double)g_RightPulse/PLUSES_PER_CIRCLE * PI * WHEEL_DIAMETER;

                    g_RightPulseDisplay += PluseCounting(ec_temp[RIGHT_PLUSE],ec_temp[RIGHT_DIRECTION]);

                    ClearCounter(ec_fd,RIGHT_PLUSE);

            }
        }

        usleep(200);
    }
}
