#-------------------------------------------------
#
# Project created by QtCreator 2019-08-16T15:57:43
#
#-------------------------------------------------

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AUBOKinematicsCalibration
TEMPLATE = app
CONFIG += c++11

INCLUDEPATH += $$PWD/../aral_export/include/
#********linking library **********************************
LIBS += $$PWD/../aral_export/lib64/libaubo_robotics.a
LIBS += -lpthread


SOURCES += main.cpp\
        mainwindow.cpp \
    kinematics_calibration.cpp \

HEADERS  += mainwindow.h \
    kinematics_calibration.h \


FORMS    += mainwindow.ui \

