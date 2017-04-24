HEADERS += \
    thread/vcthread.h \
    thread/usthread.h \
    thread/ecthread.h \
    thread/cpthread.h \
    kalman/coordinate.h \
    kalman/matrix.h \
    naviwidget.h \
    global_var.h \
    constants.h

SOURCES += \
    thread/vcthread.cpp \
    thread/usthread.cpp \
    thread/ecthread.cpp \
    thread/cpthread.cpp \
    kalman/kalmanparameters.cpp \
    kalman/kalman.cpp \
    kalman/coordinate.cpp \
    kalman/calculation.cpp \
    kalman/matrix.cpp \
    naviwidget.cpp \
    main.cpp \
    global_var.cpp

FORMS += \
    naviwidget.ui

RESOURCES += \
    myImage.qrc
