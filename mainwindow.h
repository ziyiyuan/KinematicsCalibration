#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "dialog.h"
#include <QFileDialog>
#include <QMessageBox>


#include "robotkinematics.h"
#include "kinematicscalibration.h"
#include "CalibrationType.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_cBRobotType_currentIndexChanged(int index);

    void on_cBCalibrationBeta_clicked(bool checked);

    void on_tBChoosedir_clicked();

    void on_pBCalibrate_clicked();

    void on_cBDatadir_currentTextChanged(const QString &arg1);

    void on_rBshowpara_clicked(bool checked);

    void on_rBshowDpara_clicked(bool checked);

    void on_pBrestart_clicked();

    void on_pBoutputresult_clicked();

    void on_cBCaliType_currentIndexChanged(int index);

    void showToolpara();


private:
    Ui::MainWindow *ui;
    KinematicsCalibration *kc;
    Dialog *dialog;

};

#endif // MAINWINDOW_H
