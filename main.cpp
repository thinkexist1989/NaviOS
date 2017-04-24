#include <QtGui/QApplication>
#include "naviwidget.h"

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    NaviWidget w;
    w.show();
    
    return a.exec();
}
