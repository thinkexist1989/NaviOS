#include "vcthread.h"

#include <stdio.h>   /*标准输入输出定义*/
#include <stdlib.h>  /*标准函数库定义*/
#include <unistd.h>  /*UNIX标准函数定义*/
#include <sys/types.h>  /*基本系统数据类型*/
#include <sys/stat.h>   /*unix/linux系统定义文件状态*/
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h>  /*PPSIX 终端控制定义*/
#include <errno.h>    /*错误号定义*/
#include <string.h>

vcThread::vcThread(QObject *parent) :
    QThread(parent)
{
}

int vcThread::openUart()
{
    int fd;
    struct termios opt;   /*定义指向termios 结构类型的指针opt*/

    /*int open（“dev_name”，int open_Status）   成功返回文件描述符，如果失败返回-1*/
    fd = open("/dev/ttySAC3", O_RDWR | O_NOCTTY);      //阻塞读写方式打开串口2

    if(fd == -1)  //fd == -1 不能打开串口
    {
        perror("Can Not Open ttySAC3！\n");
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
    opt.c_cc[VMIN] = 5;    //define the minimum bytes data to be readed
                   // opt.c_cc[VMIN]=0:Update the options and do it NOW

    tcflush(fd, TCIOFLUSH);//刷清（扔掉）输入缓存或输出缓存

    printf("ttySAC3 configure complete\n");

    if(tcsetattr(fd, TCSANOW, &opt) != 0)
    {
        perror("serial error");
        return -1;
    }

    return fd;
}

void vcThread::SendVoiceCommand(char playOrder)
{
    if(vc_fd >= 0)
    {
        vc_UartTxdBuf[0] = 0xFA;
        vc_UartTxdBuf[1] = 0x5F;
        vc_UartTxdBuf[2] = 0xA5;
        vc_UartTxdBuf[3] = playOrder;
        write(vc_fd,vc_UartTxdBuf,4);
    }
}

void vcThread::run()
{
    vc_fd = openUart();

    while(1)
    {
        if(vc_fd >= 0) //[3]
        {
            if(read(vc_fd, vc_UartRcvBuf, 5) == 5)//[3.1]
            {
                g_VoiceOrder = vc_UartRcvBuf[3];  //Voice Module Order

                if(g_VoiceOrder != RECIEVED_CALL){
                    g_NewVoiceMark = true;                             

                    if(playingmark == false){
                        while(!playingmark);
                        sleep(3);
                    }
                    playingmark = false;

                    switch(g_VoiceOrder)
                    {
                    case CURRENT_POSITION: g_PlayOrder = g_currentposition;
                                           SendVoiceCommand(g_currentposition);
                                           break;
                    case GO_LAB1055: g_destination = LAB1055;
                                     break;
                    case GO_DATING: g_destination = DATING;
                                    break;

                    default:break;
                    }
                    sleep(3);
                    playingmark = true;
                }

            }//[3.1]
        }//[3]
    }
}
