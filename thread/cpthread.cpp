#include "cpthread.h"

#include <stdio.h>   /*标准输入输出定义*/
#include <stdlib.h>  /*标准函数库定义*/
#include <unistd.h>  /*UNIX标准函数定义*/
#include <sys/types.h>  /*基本系统数据类型*/
#include <sys/stat.h>   /*unix/linux系统定义文件状态*/
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h>  /*PPSIX 终端控制定义*/
#include <errno.h>    /*错误号定义*/
#include <string.h>

cpThread::cpThread(QObject *parent) :
    QThread(parent)
{
}

int cpThread::openUart()
{
    int fd;
    struct termios opt;   /*定义指向termios 结构类型的指针opt*/

    /*int open（“dev_name”，int open_Status）   成功返回文件描述符，如果失败返回-1*/
    fd = open("/dev/ttySAC2", O_RDWR | O_NOCTTY);      //阻塞读写方式打开串口2

    if(fd == -1)  //fd == -1 不能打开串口
    {
        perror("Can Not Open ttySAC2！\n");
        return -1;
    }

    memset(&opt, 0, sizeof opt);  //在一段内存块中填充某个给定的值，它对较大的结构体或数组进行清零操作的一种最快方法
                                  //void *memset(void *s,  int c, size_t n)
                                  //把一个char a[20]清零, 是 memset(a, 0, 20)

    cfsetispeed(&opt, B9600);//设置波特率为9600
    cfsetospeed(&opt, B9600);


    /*tcsetattr函数用于设置终端参数。函数在成功的时候返回0，失败的时候返回-1
    参数fd为打开的终端文件描述符，参数 optional_actions用于控制修改起作用的时间，
    而结构体termios_p中保存了要修改的参数。optional_actions可以取如下的值。
        TCSANOW：不等数据传输完毕就立即改变属性。
        TCSADRAIN：等待所有数据传输结束才改变属性。
        TCSAFLUSH：清空输入输出缓冲区才改变属性。*/
    if(tcsetattr(fd, TCSANOW, &opt) != 0 )
    {
       perror("tcsetattr error");
       return -1;
    }

    /*opt.c_cflag控制模式标记*/
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;       //数据位8
    opt.c_cflag &= ~CSTOPB;   //停止位1
    /* c_iflag输入模式标记*/
    opt.c_iflag = IGNPAR;    //无校验位

    opt.c_cc[VTIME] = 0;   //设置超时0 seconds
    opt.c_cc[VMIN] = 11;    //define the minimum bytes data to be readed
                   // opt.c_cc[VMIN]=0:Update the options and do it NOW

    tcflush(fd, TCIOFLUSH);//刷清（扔掉）输入缓存或输出缓存

    printf("ttySAC2 configure complete\n");

    if(tcsetattr(fd, TCSANOW, &opt) != 0)
    {
        perror("serial error");
        return -1;
    }

    return fd;
}

void cpThread::SendAngleCommand(char angleOrder)
{
    if(cp_fd >= 0)
    {
        switch(angleOrder)
        {
            case SINGLE_ANGLE:      cp_UartTxdBuf[0] = '*';
                                    cp_UartTxdBuf[1] = 'P';
                                    write(cp_fd,cp_UartTxdBuf,2);
                                    break;
            case START_CALIBRATION: cp_UartTxdBuf[0] = 'P';
                                    write(cp_fd,cp_UartTxdBuf,1);
                                    break;
            case STOP_CALIBRATION:  cp_UartTxdBuf[0] = 'r';
                                    write(cp_fd,cp_UartTxdBuf,1);
                                    break;
            case SET_ZERO:          cp_UartTxdBuf[0] = '*';
                                    cp_UartTxdBuf[1] = 'z';
                                    write(cp_fd,cp_UartTxdBuf,2);
                                    break;
            default:break;
        }
    }
}

void cpThread::run()
{
    cp_fd = openUart();
    while(1)
    {
        SendAngleCommand(SINGLE_ANGLE);

        if(cp_fd >= 0) //[1]
        {
            if(read(cp_fd, cp_UartRcvBuf, 11) == 11)
            {
                    g_NewAngleMark = true;

                    g_cp_mutex.lock();


                    g_Angle_Ex  = g_Angle;              //将角度值存入g_Angle_Ex

                    g_Angle_Bai = cp_UartRcvBuf[3] - '0';  //角度值百位
                    g_Angle_Shi = cp_UartRcvBuf[4] - '0';  //角度值十位
                    g_Angle_Ge  = cp_UartRcvBuf[5] - '0';  //角度值个位

                    int temp;
                    g_Angle = g_Angle_Bai*100 + g_Angle_Shi*10 + g_Angle_Ge;   //当前角度值

                    /***angle modify*****/
                    if(g_Angle >= 90 && g_Angle < 225){
                        g_Angle -= ((float)45/135)*(g_Angle - 90);
                    }
                    else if(g_Angle >= 225 && g_Angle < 315){
                        g_Angle -= 45;
                    }
                    else if(g_Angle >= 315){
                        g_Angle -= ((float)45/45)*(360-g_Angle);
                    }

                    if(g_Angle > 180){
                        g_Angle -= 360;
                    }
                    /*****for kalman filter********/
                    temp = g_Angle;
                    temp += g_circle*360;

                    if((temp - g_Angle_Ex) < -200){
                        g_circle++;
                        temp += 360;
                    }
                    else if((temp - g_Angle_Ex) > 200){
                        g_circle--;
                        temp -= 360;
                    }

                    g_radian = (double)temp/180*PI;

                    g_cp_mutex.unlock();

          //判断角度变化值，向左为负，向右为正，绝对值不能超过180度
                    if(abs(g_Angle - g_Angle_Ex)>180)
                    {
                        if((g_Angle - g_Angle_Ex)>180)
                        {
                            g_Angle_Change = g_Angle - g_Angle_Ex - 360;
                        }
                        else
                            g_Angle_Change = g_Angle - g_Angle_Ex + 360;
                    }
                    else
                    {
                        g_Angle_Change = g_Angle - g_Angle_Ex;
                    }
            }
        }//[1]

        usleep(200);
    }
}
