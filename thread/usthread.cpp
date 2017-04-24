#include "usthread.h"

#include <stdio.h>   /*标准输入输出定义*/
#include <stdlib.h>  /*标准函数库定义*/
#include <unistd.h>  /*UNIX标准函数定义*/
#include <sys/types.h>  /*基本系统数据类型*/
#include <sys/stat.h>   /*unix/linux系统定义文件状态*/
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h>  /*PPSIX 终端控制定义*/
#include <errno.h>    /*错误号定义*/
#include <string.h>

usThread::usThread(QObject *parent) :
    QThread(parent)
{
}

int usThread::openUart()
{
    int fd;
    struct termios opt;   /*定义指向termios 结构类型的指针opt*/

    /*int open（“dev_name”，int open_Status）   成功返回文件描述符，如果失败返回-1*/
    fd = open("/dev/ttySAC1", O_RDWR | O_NOCTTY);      //阻塞读写方式打开串口1

    if(fd == -1)  //fd == -1 不能打开串口
    {
        perror("Can Not Open ttySAC1！\n");
        return -1;
    }

    memset(&opt, 0, sizeof opt);  //在一段内存块中填充某个给定的值，它对较大的结构体或数组进行清零操作的一种最快方法
                                  //void *memset(void *s,  int c, size_t n)
                                  //把一个char a[20]清零, 是 memset(a, 0, 20)

    cfsetispeed(&opt, B19200);//设置波特率为19200
    cfsetospeed(&opt, B19200);


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
    opt.c_cc[VMIN] = 15;    //define the minimum bytes data to be readed
                   // opt.c_cc[VMIN]=0:Update the options and do it NOW

    tcflush(fd, TCIOFLUSH);//刷清（扔掉）输入缓存或输出缓存

    printf("ttySAC1 configure complete\n");

    if(tcsetattr(fd, TCSANOW, &opt) != 0)
    {
        perror("serial error");
        return -1;
    }

    return fd;
}

void usThread::SendLocateCommand(int RcvNum)
{
    unsigned char sum = 0;
    us_UartTxdBuf[0] = 0x55;
    us_UartTxdBuf[1] = 0xAA;
    us_UartTxdBuf[2] = TXD_ADDR;
    us_UartTxdBuf[3] = MY_ADDR;
    us_UartTxdBuf[4] = 0x03;     //帧长
    us_UartTxdBuf[5] = SET_WORKMODE;
    sum += us_UartTxdBuf[5];
    us_UartTxdBuf[6] = SINGLE_MEASURE;
    sum += us_UartTxdBuf[6];
    us_UartTxdBuf[7] = RcvNum;
    sum += us_UartTxdBuf[7];
    us_UartTxdBuf[8] = ~sum;

    if(us_fd >= 0)
    {
        write(us_fd,us_UartTxdBuf,9);
    }
}

void usThread::run()
{
    us_fd = openUart();

    while(1)
    {
        if(us_fd >= 0) //[2]
        {
            if(read(us_fd, us_UartRcvBuf, 15) == 15)
            {

                    g_us_mutex.lock();
                    int i,k;

                    g_RcvNumCnt = 0;

                    g_GroupNum = us_UartRcvBuf[5];

                    i = 6;

                    for(k=0;k<4;k++)
                    {
                       //将收到的距离数据存入dist[4]中
                        g_dist[k] = us_UartRcvBuf[i];
                        i++;
                        g_dist[k] += us_UartRcvBuf[i]*256;     //单位:mm
                        i++;

                        if(g_dist[k] < MAXDIS)    //循环判断dist[k]中的数据是否超过允许最大值
                        {
                            g_ID[g_RcvNumCnt] = k;  //将符合条件的ID存入ID[]中
                            g_ValidDist[g_RcvNumCnt] = g_dist[k];
                            g_RcvNumCnt++;        //RcvNumCnt记录一组中共有多少有效数据
                        }
                        else
                        {
                            g_dist[k] = -1;
                        }

                     }

                    if(g_RcvNumCnt > 1)
                    {
                        g_NewLocMark = true;
                    };

                    for(int i=0;i<4;i++){
                        g_Group[i] = g_GroupNum;
                    }




                    //Calculation(g_RcvNumCnt,g_ValidDist,g_ID,g_Group,abs_obs,state_rel,R_obs);

                    //c.Update(c.GetX() + RELATIVENODE[g_GroupNum][RELATIVE_X],c.GetY() + RELATIVENODE[g_GroupNum][RELATIVE_Y],c.GetZ());

                    g_us_mutex.unlock();
            }
        }//[2]
    }
}
