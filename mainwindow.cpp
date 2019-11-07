#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //    dialog = new Dialog(this);
    //    dialog->setModal(false);

    kc = new KinematicsCalibration();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete kc;
}


void MainWindow::on_cBRobotType_currentIndexChanged(int index)
{
    kc->setRobotDHPara((ROBOT_TYPE)index);
    kc->initiallDHPara();
}

void MainWindow::on_cBCalibrationBeta_clicked(bool checked)
{
    if(!checked)
    {
        ui->pBbeta3->setChecked(checked);
        ui->pBbeta4->setChecked(checked);
        kc->setCheckFlag(BETA3,checked);
        kc->setCheckFlag(BETA4,checked);
    }
    ui->pBbeta3->setEnabled(checked);
    ui->pBbeta4->setEnabled(checked);
    kc->setCalibrationBeta(checked);
}

void MainWindow::on_tBChoosedir_clicked()
{
    ui->cBDatadir->clear();
    QString path = QDir::toNativeSeparators(QFileDialog::getExistingDirectory(this,tr("View file"),QDir::currentPath()));
    ui->labelDatapath->setText(path);

    QDir dir(path);
    //

    dir.setFilter(QDir::Files | QDir::NoDotAndDotDot | QDir::Dirs);
    QFileInfoList list = dir.entryInfoList();

    for(int i = 0; i < list.length(); i++)
    {
        QString file = list.at(i).fileName();
        ui->cBDatadir->addItem(file);
    }
}


void MainWindow::on_pBalpha2_clicked(bool checked)
{
    kc->setCheckFlag(ALPHA2,checked);
}

void MainWindow::on_pBalpha3_clicked(bool checked)
{
    kc->setCheckFlag(ALPHA3,checked);
}

void MainWindow::on_pBalpha4_clicked(bool checked)
{
    kc->setCheckFlag(ALPHA4,checked);
}

void MainWindow::on_pBalpha5_clicked(bool checked)
{
    kc->setCheckFlag(ALPHA5,checked);
}

void MainWindow::on_pBalpha6_clicked(bool checked)
{
    kc->setCheckFlag(ALPHA6,checked);
}


void MainWindow::on_pBa2_clicked(bool checked)
{
    kc->setCheckFlag(A2,checked);
}

void MainWindow::on_pBa3_clicked(bool checked)
{
    kc->setCheckFlag(A3,checked);
}

void MainWindow::on_pBa4_clicked(bool checked)
{
    kc->setCheckFlag(A4,checked);
}

void MainWindow::on_pBa5_clicked(bool checked)
{
    kc->setCheckFlag(A5,checked);
}

void MainWindow::on_pBa6_clicked(bool checked)
{
    kc->setCheckFlag(A6,checked);
}


void MainWindow::on_pBtheta2_clicked(bool checked)
{
    kc->setCheckFlag(THETA2,checked);
}

void MainWindow::on_pBtheta3_clicked(bool checked)
{
    kc->setCheckFlag(THETA3,checked);
}
void MainWindow::on_pBtheta4_clicked(bool checked)
{
    kc->setCheckFlag(THETA4,checked);
}
void MainWindow::on_pBtheta5_clicked(bool checked)
{
    kc->setCheckFlag(THETA5,checked);
}


void MainWindow::on_pBd2_clicked(bool checked)
{
    kc->setCheckFlag(D2,checked);
}

void MainWindow::on_pBd5_clicked(bool checked)
{
    kc->setCheckFlag(D5,checked);
}

void MainWindow::on_pBbeta3_clicked(bool checked)
{
    kc->setCheckFlag(BETA3,checked);
}

void MainWindow::on_pBbeta4_clicked(bool checked)
{
    kc->setCheckFlag(BETA4,checked);
}

//load para
void MainWindow::on_cBDatadir_currentTextChanged(const QString &arg1)
{
    std::string pathOne = ui->labelDatapath->text().toStdString()+ '/' + ui->cBDatadir->currentText().toStdString();
    kc->loadMeasuredData(pathOne);
    kc->loadInputJointAngle(pathOne);
    kc->loadInputToolData(pathOne);

    ui->lETx->setText(QString::number(kc->getToolPara("Tx")*MM2METER,'g',5));//in mm
    ui->lETy->setText(QString::number(kc->getToolPara("Ty")*MM2METER,'g',5));
    ui->lETz->setText(QString::number(kc->getToolPara("Tz")*MM2METER,'g',5));

    ui->lEMx->setText(QString::number(kc->getToolPara("Mx")*MM2METER,'g',5));
    ui->lEMy->setText(QString::number(kc->getToolPara("My")*MM2METER,'g',5));
    ui->lEMz->setText(QString::number(kc->getToolPara("Mz")*MM2METER,'g',5));
}

void MainWindow::on_pBCalibrate_clicked()
{
    ui->pBCalibrate->setEnabled(false);
    ui->pBrestart->setEnabled(true);

    //para frame false
    ui->gBCalibrationPara->setEnabled(false);
    ui->inputdata->setEnabled(false);
//    ui->RobotType->setEnabled(false);
    ui->cBRobotType->setEnabled(false);
    ui->cBCalibrationBeta->setEnabled(false);

    //
    bool flag = kc->calibration();
    if(flag)
    {
        kc->showdpara();

        ui->pBCalibrate->setText("done");

        ui->rBshowpara->setChecked(true);
        on_rBshowpara_clicked(true);

        ui->pBoutputresult->setEnabled(true);
    }
    else
    {
        ui->pBCalibrate->setText("error");
    }
}

void MainWindow::on_rBshowpara_clicked(bool checked)
{
    double allpara[30];
    kc->getAllPara(allpara);

    ui->rBshowDpara->setChecked(!checked);
    int i = 0;
    if(ui->pBalpha1->isChecked())
        ui->pBalpha1->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBa1->isChecked())
        ui->pBa1->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta1->isChecked())
        ui->pBtheta1->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBd1->isChecked())
        ui->pBd1->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta1->isChecked())
        ui->pBbeta1->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBalpha2->isChecked())
        ui->pBalpha2->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBa2->isChecked())
        ui->pBa2->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta2->isChecked())
        ui->pBtheta2->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBd2->isChecked())
        ui->pBd2->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta2->isChecked())
        ui->pBbeta2->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBalpha3->isChecked())
        ui->pBalpha3->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBa3->isChecked())
        ui->pBa3->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta3->isChecked())
        ui->pBtheta3->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBd3->isChecked())
        ui->pBd3->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta3->isChecked())
        ui->pBbeta3->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBalpha4->isChecked())
        ui->pBalpha4->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBa4->isChecked())
        ui->pBa4->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta4->isChecked())
        ui->pBtheta4->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBd4->isChecked())
        ui->pBd4->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta4->isChecked())
        ui->pBbeta4->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBalpha5->isChecked())
        ui->pBalpha5->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBa5->isChecked())
        ui->pBa5->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta5->isChecked())
        ui->pBtheta5->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBd5->isChecked())
        ui->pBd5->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta5->isChecked())
        ui->pBbeta5->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBalpha6->isChecked())
        ui->pBalpha6->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBa6->isChecked())
        ui->pBa6->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta6->isChecked())
        ui->pBtheta6->setText(QString::number((allpara[i]*r2D),'g',5)); i++;

    if(ui->pBd6->isChecked())
        ui->pBd6->setText(QString::number((allpara[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta6->isChecked())
        ui->pBbeta6->setText(QString::number((allpara[i]*r2D),'g',5)); i++;
}

void MainWindow::on_rBshowDpara_clicked(bool checked)
{
    double all_d_para[30];
    kc->getAllDPara(all_d_para);

    ui->rBshowpara->setChecked(!checked);

    int i = 0;
    if(ui->pBalpha1->isChecked())
        ui->pBalpha1->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBa1->isChecked())
        ui->pBa1->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta1->isChecked())
        ui->pBtheta1->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBd1->isChecked())
        ui->pBd1->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta1->isChecked())
        ui->pBbeta1->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBalpha2->isChecked())
        ui->pBalpha2->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBa2->isChecked())
        ui->pBa2->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta2->isChecked())
        ui->pBtheta2->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBd2->isChecked())
        ui->pBd2->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta2->isChecked())
        ui->pBbeta2->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBalpha3->isChecked())
        ui->pBalpha3->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBa3->isChecked())
        ui->pBa3->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta3->isChecked())
        ui->pBtheta3->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBd3->isChecked())
        ui->pBd3->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta3->isChecked())
        ui->pBbeta3->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBalpha4->isChecked())
        ui->pBalpha4->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBa4->isChecked())
        ui->pBa4->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta4->isChecked())
        ui->pBtheta4->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBd4->isChecked())
        ui->pBd4->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta4->isChecked())
        ui->pBbeta4->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBalpha5->isChecked())
        ui->pBalpha5->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBa5->isChecked())
        ui->pBa5->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta5->isChecked())
        ui->pBtheta5->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBd5->isChecked())
        ui->pBd5->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta5->isChecked())
        ui->pBbeta5->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBalpha6->isChecked())
        ui->pBalpha6->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBa6->isChecked())
        ui->pBa6->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBtheta6->isChecked())
        ui->pBtheta6->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

    if(ui->pBd6->isChecked())
        ui->pBd6->setText(QString::number((all_d_para[i]*MM2METER),'g',5)); i++;

    if(ui->pBbeta6->isChecked())
        ui->pBbeta6->setText(QString::number((all_d_para[i]*r2D),'g',5)); i++;

}


void MainWindow::on_pBrestart_clicked()
{
    ui->pBrestart->setEnabled(false);

    ui->pBCalibrate->setEnabled(true);
    ui->pBCalibrate->setText("start");

    ui->gBCalibrationPara->setEnabled(true);
    ui->inputdata->setEnabled(true);
//    ui->RobotType->setEnabled(true);
    ui->cBRobotType->setEnabled(true);

    ui->cBCalibrationBeta->setEnabled(true);

    ui->pBoutputresult->setEnabled(false);

    //
    ui->pBalpha1->setText("alpha1");
    ui->pBalpha2->setText("alpha2");
    ui->pBalpha3->setText("alpha3");
    ui->pBalpha4->setText("alpha4");
    ui->pBalpha5->setText("alpha5");
    ui->pBalpha6->setText("alpha6");

    ui->pBa1->setText("a1");
    ui->pBa2->setText("a2");
    ui->pBa3->setText("a3");
    ui->pBa4->setText("a4");
    ui->pBa5->setText("a5");
    ui->pBa6->setText("a6");

    ui->pBtheta1->setText("theta1");
    ui->pBtheta2->setText("theta2");
    ui->pBtheta3->setText("theta3");
    ui->pBtheta4->setText("theta4");
    ui->pBtheta5->setText("theta5");
    ui->pBtheta6->setText("theta6");

    ui->pBd1->setText("d1");
    ui->pBd2->setText("d2");
    ui->pBd3->setText("d3");
    ui->pBd4->setText("d4");
    ui->pBd5->setText("d5");
    ui->pBd6->setText("d6");

    ui->pBbeta1->setText("beta1");
    ui->pBbeta2->setText("beta2");
    ui->pBbeta3->setText("beta3");
    ui->pBbeta4->setText("beta4");
    ui->pBbeta5->setText("beta5");
    ui->pBbeta6->setText("beta6");
}

void MainWindow::on_pBoutputresult_clicked()
{
    std::string pathOne = ui->labelDatapath->text().toStdString()+ '/' + ui->cBDatadir->currentText().toStdString();
    kc->outputClibrationDPara(pathOne);
    //out put data;
}
