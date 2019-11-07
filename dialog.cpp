#include "dialog.h"
#include "ui_dialog.h"


Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    initialUI();
    kc = new KinematicsCalibration();
}

Dialog::~Dialog()
{
    delete kc;
    delete ui;
}

void Dialog:: initialUI()
{
    ui->pB_dh_alpha1->setStyleSheet(pushButtonColorUnchecked);
    ui->pB_dh_alpha2->setStyleSheet(pushButtonColorUnchecked);
    ui->pB_dh_alpha3->setStyleSheet(pushButtonColorUnchecked);
    ui->pB_dh_alpha4->setStyleSheet(pushButtonColorUnchecked);
    ui->pB_dh_alpha5->setStyleSheet(pushButtonColorUnchecked);
}


void Dialog::on_pB_dh_alpha2_clicked(bool checked)
{
    if(checked)
        ui->pB_dh_alpha2->setStyleSheet(pushButtonColorChecked);
    else
        ui->pB_dh_alpha2->setStyleSheet(pushButtonColorUnchecked);
    kc->setCheckFlag(ALPHA2, checked);
}

void Dialog::on_pB_dh_alpha3_clicked(bool checked)
{
    qDebug()<<"checked:"<<checked;
}

void Dialog::on_pB_dh_ok_clicked()
{
    //kc->runCalibraProcess();
    this->hide();
}

void Dialog::on_pB_dh_alpha1_clicked(bool checked)
{

}
