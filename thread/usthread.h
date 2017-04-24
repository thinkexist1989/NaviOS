#ifndef USTHREAD_H
#define USTHREAD_H

#include <QThread>
#include "../constants.h"
#include "../global_var.h"
#include "../kalman/coordinate.h"



class usThread : public QThread
{
    Q_OBJECT
public:
    explicit usThread(QObject *parent = 0);

    int openUart();

    void SendLocateCommand(int RcvNum);   //Send Locating Command to UltraSonic Module
    
signals:
    
public slots:

private:
    int us_fd;
    unsigned char us_UartRcvBuf[15];	// 接收缓冲区
    unsigned char us_UartTxdBuf[9];  // 发射缓冲区

protected:
    void run();
    
};

#endif // USTHREAD_H
