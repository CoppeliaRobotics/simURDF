include(config.pri)

QT -= core
QT -= gui

TARGET = simExtURDF
TEMPLATE = lib

DEFINES -= UNICODE
CONFIG += shared plugin
INCLUDEPATH += "../include"
INCLUDEPATH += "../simMath"
greaterThan(QT_MAJOR_VERSION,4): QT += widgets

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs
}

INCLUDEPATH += $$BOOST_INCLUDEPATH

win32 {
    DEFINES += WIN_SIM
}

macx {
    DEFINES += MAC_SIM
}

unix:!macx {
    DEFINES += LIN_SIM
}

SOURCES += \
    simExtURDF.cpp \
    tinyxml2/tinyxml2.cpp \
    robot.cpp \
    link.cpp \
    joint.cpp \
    sensor.cpp \
    commonFunctions.cpp \
    ../simMath/3Vector.cpp \
    ../simMath/3X3Matrix.cpp \
    ../simMath/4Vector.cpp \
    ../simMath/4X4Matrix.cpp \
    ../simMath/MMatrix.cpp \
    ../simMath/7Vector.cpp \
    ../simMath/MyMath.cpp \
    ../common/simLib.cpp \
    ../common/scriptFunctionData.cpp \
    ../common/scriptFunctionDataItem.cpp \
    rospackagehelper.cpp

HEADERS +=\
    simExtURDF.h \
    tinyxml2/tinyxml2.h \
    robot.h \
    link.h \
    joint.h \
    sensor.h \
    commonFunctions.h \
    ../simMath/3Vector.h \
    ../simMath/3X3Matrix.h \
    ../simMath/4Vector.h \
    ../simMath/4X4Matrix.h \
    ../simMath/MMatrix.h \
    ../simMath/7Vector.h \
    ../simMath/MyMath.h \
    ../include/simLib.h \
    ../include/scriptFunctionData.h \
    ../include/scriptFunctionDataItem.h \
    rospackagehelper.h

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
