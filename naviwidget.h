#ifndef NAVIWIDGET_H
#define NAVIWIDGET_H

#include <QWidget>
#include "constants.h"
#include "global_var.h"
#include "thread/cpthread.h"
#include "thread/ecthread.h"
#include "thread/usthread.h"
#include "thread/vcthread.h"
#include "fstream"

using namespace std;

namespace Ui {
class NaviWidget;
}

class NaviWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit NaviWidget(QWidget *parent = 0);
    ~NaviWidget();

    int i;
    
private slots:
    void on_setzero_clicked();

    void on_calibration_clicked();

private:
    Ui::NaviWidget *ui;

    cpThread compass;
    ecThread encoder;
    usThread ultrasonic;
    vcThread voice;

    int ultrasonicID;

    int updateID;
    int locateID;


    void clearpluse();
    void timerEvent(QTimerEvent *event);
protected:
    void paintEvent(QPaintEvent *event);
};

#endif // NAVIWIDGET_H
