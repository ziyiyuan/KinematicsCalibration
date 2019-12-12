#ifndef KINEMATICSCALIBRATION_H
#define KINEMATICSCALIBRATION_H

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream> // string stream
#include <math.h>
#include <string>
#include <stdio.h>

#include "robotkinematics.h"
#include "CalibrationType.h"
#include "CaliLine.h"
#include "CaliLeica.h"

using namespace std;

typedef enum
{
    ALPHA1 = 0,A1,D1,THETA1,BETA1,
    ALPHA2,A2,D2,THETA2,BETA2,
    ALPHA3,A3,D3,THETA3,BETA3,
    ALPHA4,A4,D4,THETA4,BETA4,
    ALPHA5,A5,D5,THETA5,BETA5,
    ALPHA6,A6,D6,THETA6,BETA6,

} CALIBRATE_PARA;

class KinematicsCalibration
{
public:
    KinematicsCalibration();
    ~KinematicsCalibration();

    void setCaliType(const std::string& type);
    void getCaliType();

    void setCalibrationBeta(bool value);
    void getCalibrationBeta();

    void setRobotDHPara(ROBOT_TYPE type);
    void initiallDHPara();

    void setPara();
    bool LoadData(const std::string& datafile);
    bool calibration();

    void calAllPara();
    void updateMemberPara();


    void getAllPara(double allpara[]);
    void getAllDPara(double all_d_para[]);

    double getToolPara(std::string index);
    void getCriter(double output_criter[]);

    void outputClibrationDPara(std::string  path, std::string data_file);

private:
    RobotKinematics *rk;
    CaliType *cali_type;
    CaliLine *cali_line;
    CaliLeica *cali_leica;

    RVector d_allpara;
    double all_para_[MAX_IDEN_PARA_NUM];

    bool kc_cali_beta_;

public:
    CALIBRATE_METHOD kc_cali_method_;


};

#endif // KINEMATICSCALIBRATION_H
