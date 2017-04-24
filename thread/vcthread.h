#ifndef VCTHREAD_H
#define VCTHREAD_H

#include <QThread>
#include "../constants.h"
#include "../global_var.h"

class vcThread : public QThread
{
    Q_OBJECT
public:
    explicit vcThread(QObject *parent = 0);

    int openUart();

    void SendVoiceCommand(char playOrder);  //Send Command to Voice Module

    bool playingmark;
    
signals:
    
public slots:

private:
    int vc_fd;
    unsigned char vc_UartRcvBuf[5];	// 接收缓冲区
    unsigned char vc_UartTxdBuf[5];  // 发射缓冲区

protected:
    void run();
    
};

#endif // VCTHREAD_H
