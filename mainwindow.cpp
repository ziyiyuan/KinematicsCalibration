#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //        dialog = new Dialog(this);
    //        dialog->setModal(false);

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
}

void MainWindow::on_cBCaliType_currentIndexChanged(int index)
{
    QString str = ui->cBCaliType->currentText();
    kc->setCaliType(str.toStdString());
}


void MainWindow::on_cBCalibrationBeta_clicked(bool checked)
{
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

//load para
void MainWindow::on_cBDatadir_currentTextChanged(const QString &arg1)
{
    std::string pathOne = ui->labelDatapath->text().toStdString()+ '/' + ui->cBDatadir->currentText().toStdString() + '/';
}

void MainWindow::on_pBCalibrate_clicked()
{
    ui->gBInitiallize->setEnabled(false);
    ui->inputdata->setEnabled(false);
    //    ui->gBCalibrationPara->setEnabled(false);
    ui->pBCalibrate->setEnabled(false);

    std::string pathOne = ui->labelDatapath->text().toStdString()+ '/' + ui->cBDatadir->currentText().toStdString() + '/';
    kc->setPara();

    bool flag_ld = kc->LoadData(pathOne);
    if(!flag_ld)
    {
        QString str = QString("Load data failed!");
        QMessageBox::warning(this,"Title",str);
//        QStatusBar.showMessage("press the restart button");
        ui->frResult->setEnabled(true);
        ui->pBrestart->setEnabled(true);
    }
    else
    {

    bool flag = kc->calibration();
    if(flag)
    {
        ui->pBCalibrate->setText("done");
        ui->rBshowDpara->setEnabled(true);
        ui->rBshowpara->setEnabled(true);

        ui->frResult->setEnabled(true);
        ui->pBoutputresult->setEnabled(true);
        ui->pBrestart->setEnabled(true);

        ui->rBshowDpara->setChecked(true);

        on_rBshowDpara_clicked(true);
        // show tool measurement and criter data
        showToolpara();
    }
    else
    {
        ui->pBCalibrate->setText("error");
    }
    }
}

void MainWindow::on_rBshowpara_clicked(bool checked)
{
    double allpara[MAX_IDEN_PARA_NUM];
    kc->getAllPara(allpara);

    ui->rBshowDpara->setChecked(!checked);
    int i;
    if(kc->kc_cali_method_ == CALI_LINE)
        i = 6;
    if(kc->kc_cali_method_ == CALI_LEICA)
        i = 9;

    ui->pBalpha1->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBa1->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBd1->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBtheta1->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBbeta1->setText(QString::number((allpara[i]*r2D),'g',6)); i++;

    ui->pBalpha2->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBa2->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBd2->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBtheta2->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBbeta2->setText(QString::number((allpara[i]*r2D),'g',6)); i++;

    ui->pBalpha3->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBa3->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBd3->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBtheta3->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBbeta3->setText(QString::number((allpara[i]*r2D),'g',6)); i++;

    ui->pBalpha4->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBa4->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBd4->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBtheta4->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBbeta4->setText(QString::number((allpara[i]*r2D),'g',6)); i++;

    ui->pBalpha5->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBa5->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBd5->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBtheta5->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBbeta5->setText(QString::number((allpara[i]*r2D),'g',6)); i++;

    ui->pBalpha6->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBa6->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBd6->setText(QString::number((allpara[i]*METER2MM),'g',6)); i++;
    ui->pBtheta6->setText(QString::number((allpara[i]*r2D),'g',6)); i++;
    ui->pBbeta6->setText(QString::number((allpara[i]*r2D),'g',6)); i++;



}

void MainWindow::on_rBshowDpara_clicked(bool checked)
{
    double all_d_para[MAX_IDEN_PARA_NUM];
    kc->getAllDPara(all_d_para);
    ui->rBshowpara->setChecked(!checked);

    int i;

    if(kc->kc_cali_method_ == CALI_LINE)
        i = 6;
    if(kc->kc_cali_method_ == CALI_LEICA)
        i = 9;

    ui->pBalpha1->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBa1->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBd1->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBtheta1->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBbeta1->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;

    ui->pBalpha2->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBa2->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBd2->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBtheta2->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBbeta2->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;

    ui->pBalpha3->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBa3->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBd3->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBtheta3->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBbeta3->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;

    ui->pBalpha4->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBa4->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBd4->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBtheta4->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBbeta4->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;

    ui->pBalpha5->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBa5->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBd5->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBtheta5->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBbeta5->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;

    ui->pBalpha6->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBa6->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBd6->setText(QString::number((all_d_para[i]*METER2MM),'f',5)); i++;
    ui->pBtheta6->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;
    ui->pBbeta6->setText(QString::number((all_d_para[i]*r2D),'f',5)); i++;

}

void MainWindow::showToolpara()
{
    // show criter data
    double  output_criter[3];
    kc->getCriter(output_criter);
    ui->lEMax->setText(QString::number((output_criter[0]*METER2MM),'g',4));
    ui->lEMean->setText(QString::number((output_criter[1]*METER2MM),'g',4));
    ui->lERMS->setText(QString::number((output_criter[2]*METER2MM),'g',4));
    // show tool and measurement para

    ui->lETx->setText(QString::number(kc->getToolPara("Tx")*METER2MM,'g',4));//in mm
    ui->lETy->setText(QString::number(kc->getToolPara("Ty")*METER2MM,'g',4));
    ui->lETz->setText(QString::number(kc->getToolPara("Tz")*METER2MM,'g',4));

    ui->lEMx->setText(QString::number(kc->getToolPara("Mx")*METER2MM,'g',4));
    ui->lEMy->setText(QString::number(kc->getToolPara("My")*METER2MM,'g',4));
    ui->lEMz->setText(QString::number(kc->getToolPara("Mz")*METER2MM,'g',4));

    if(kc->kc_cali_method_ == CALI_LEICA)
    {
        ui->lEMRx->setText(QString::number(kc->getToolPara("MRx")*r2D,'g',4));
        ui->lEMRy->setText(QString::number(kc->getToolPara("MRy")*r2D,'g',4));
        ui->lEMRz->setText(QString::number(kc->getToolPara("MRz")*r2D,'g',4));
    }
    else
    {
        ui->lEMRx->setText(QString::number(0));
        ui->lEMRy->setText(QString::number(0));
        ui->lEMRz->setText(QString::number(0));
    }
}

void MainWindow::on_pBrestart_clicked()
{
    ui->pBrestart->setEnabled(false);

    ui->pBCalibrate->setEnabled(true);
    ui->pBCalibrate->setText("start");

    ui->gBInitiallize->setEnabled(true);
    ui->inputdata->setEnabled(true);
    //    ui->gBCalibrationPara->setEnabled(true);
    ui->pBCalibrate->setEnabled(true);


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


    ui->lEMRx->setText(QString::number(0));
    ui->lEMRy->setText(QString::number(0));
    ui->lEMRz->setText(QString::number(0));

    ui->lETx->setText(QString::number(0));
    ui->lETy->setText(QString::number(0));
    ui->lETz->setText(QString::number(0));

    ui->lEMx->setText(QString::number(0));
    ui->lEMy->setText(QString::number(0));
    ui->lEMz->setText(QString::number(0));

    ui->lEMax->setText(QString::number(0));
    ui->lEMean->setText(QString::number(0));
    ui->lERMS->setText(QString::number(0));
}

void MainWindow::on_pBoutputresult_clicked()
{
    //    std::string pathOne = ui->labelDatapath->text().toStdString()+ '/' + ui->cBDatadir->currentText().toStdString() + '/';
    std::string pathOne = ui->labelDatapath->text().toStdString()+ '/' ;
    std::string data_file = ui->cBDatadir->currentText().toStdString();
    kc->outputClibrationDPara(pathOne,data_file);
    //out put data;
}




