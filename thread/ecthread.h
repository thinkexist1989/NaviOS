#ifndef ECTHREAD_H
#define ECTHREAD_H

#include <QThread>
#include "../constants.h"
#include "../global_var.h"

class ecThread : public QThread
{
    Q_OBJECT
public:
    explicit ecThread(QObject *parent = 0);

    int openGPIO();
    
signals:
    
public slots:

private:
    int ec_fd;
    int ec_temp[4];

    int PluseCounting(int g_pulse,int g_direct);

    int ClearCounter(int fd,int Num);

protected:
    void run();
    
};

#endif // ECTHREAD_H
