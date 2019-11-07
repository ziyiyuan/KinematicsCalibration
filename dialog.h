#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "QDebug"
#include "kinematicscalibration.h"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

private slots:
    void on_pB_dh_alpha2_clicked(bool checked);
    void on_pB_dh_alpha3_clicked(bool checked);

    void on_pB_dh_ok_clicked();

    void on_pB_dh_alpha1_clicked(bool checked);

private:
    const QString pushButtonColorChecked = "background-color:orange";
    const QString pushButtonColorUnchecked = "background-color:lightGray";
    void initialUI();
    Ui::Dialog *ui;
    KinematicsCalibration *kc;
};

#endif // DIALOG_H
