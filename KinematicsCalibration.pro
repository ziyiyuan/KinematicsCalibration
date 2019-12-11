#-------------------------------------------------
#
# Project created by QtCreator 2019-08-16T15:57:43
#
#-------------------------------------------------

QT       += core gui

#include(./utility/utility.pri)

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = KinematicsCalibration
TEMPLATE = app
CONFIG += c++11


SOURCES += main.cpp\
        mainwindow.cpp \
    robotkinematics.cpp \
    kinematicscalibration.cpp \
    CalibrationType.cpp \
    CaliLine.cpp \
    CaliLeica.cpp

HEADERS  += mainwindow.h \
    robotkinematics.h \
    kinematicscalibration.h \
    CalibrationType.h \
    CaliLine.h \
    CaliLeica.h

FORMS    += mainwindow.ui \

INCLUDEPATH += /home/ziyi/Desktop/AUBO_ROBOTICS/aubo_robotics/math/include/
#INCLUDEPATH += /home/ziyi/AUBO_ROBOTICS/aubo_robotics/model/include/
LIBS += /home/ziyi/Desktop/AUBO_ROBOTICS/lib/lib64/libaubo_robotics.a
#    \dialog.ui

