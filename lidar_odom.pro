TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    lidar.cpp
HEADERS += \
    lidar.h
INCLUDEPATH += rplidar/sdk/include
LIBS += -L ~/prj/lidar_odom/lidar_odom/rplidar/sdk/lib/ -lrplidar_sdk -lpthread



