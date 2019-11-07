#include "mainwindow.h"
#include <QApplication>

#include <iostream>
#include <fstream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //    MainWindow w;
    //    w.show();

    double aa[6][6] = {{1,2,3,4,5,6},{7,8,9,10,11,12},{13,14,15,16,17,18},{19,20,21,22,23,34},{25,26,27,28,29,30},{31,32,33,34,35,36}};
    RMatrix A(6,6);
    RMatrix U(6), S(6,6), V(6), Q(6,6);
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
            A.value[i][j] = aa[i][j];
    }
    RMatrix::qrFullRMatrix(A,Q);

    //    RMatrix::svdSim(A, U, S, V);



    return a.exec();
}
