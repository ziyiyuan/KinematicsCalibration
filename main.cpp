#include "mainwindow.h"
#include <QApplication>
#include <QDesktopWidget>

#include <iostream>
#include <fstream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    w.setFixedSize(w.width(),w.height());
    w.move((QApplication::desktop()->width() - w.width())/2 ,(QApplication::desktop()->height() - w.height())/2);
    w.show();

    return a.exec();
}
