#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include "kinematics_calibration.h"

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

    void on_pBalpha2_clicked(bool checked);
    void on_pBalpha3_clicked(bool checked);
    void on_pBalpha4_clicked(bool checked);
    void on_pBalpha5_clicked(bool checked);
    void on_pBalpha6_clicked(bool checked);

    void on_pBa2_clicked(bool checked);
    void on_pBa3_clicked(bool checked);
    void on_pBa4_clicked(bool checked);
    void on_pBa5_clicked(bool checked);
    void on_pBa6_clicked(bool checked);

    void on_pBd2_clicked(bool checked);
    void on_pBd5_clicked(bool checked);

    void on_pBtheta2_clicked(bool checked);
    void on_pBtheta3_clicked(bool checked);
    void on_pBtheta4_clicked(bool checked);
    void on_pBtheta5_clicked(bool checked);

    void on_pBbeta3_clicked(bool checked);
    void on_pBbeta4_clicked(bool checked);


private:
    Ui::MainWindow *ui;
    KinematicsCalibration *kc;
};

#endif // MAINWINDOW_H
