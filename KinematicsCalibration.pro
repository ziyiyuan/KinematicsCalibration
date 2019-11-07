#-------------------------------------------------
#
# Project created by QtCreator 2019-08-16T15:57:43
#
#-------------------------------------------------

QT       += core gui

include(./utility/utility.pri)

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = KinematicsCalibration
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    robotkinematics.cpp \
    kinematicscalibration.cpp

HEADERS  += mainwindow.h \
    robotkinematics.h \
    kinematicscalibration.h

FORMS    += mainwindow.ui \
#    \dialog.ui

