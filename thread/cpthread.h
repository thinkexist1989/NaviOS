#ifndef CPTHREAD_H
#define CPTHREAD_H

#include <QThread>
#include "../constants.h"
#include "../global_var.h"

class cpThread : public QThread
{
    Q_OBJECT
public:
    explicit cpThread(QObject *parent = 0);

    int openUart();
    void SendAngleCommand(char angleOrder);  //Send Command to Compass Module
    
signals:
    
public slots:

private:
    int cp_fd;
    unsigned char cp_UartRcvBuf[11];	// 接收缓冲区
    unsigned char cp_UartTxdBuf[2];  // 发射缓冲区

protected:
    void run();
    
};

#endif // CPTHREAD_H
