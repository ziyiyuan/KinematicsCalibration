#ifndef KINEMATICSCALIBRATION_H
#define KINEMATICSCALIBRATION_H

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "robotkinematics.h"


using namespace std;
#define MAX_IDEN_PARA_NUM 30  //maximum identification parameter number, all dh para;
#define MEA_DATA_NUM 60  //measure waypoint number,

#define DOF 6
#define POSITION_DOF 3

#define D2r M_PI/180
#define r2D 180/M_PI

#define MM2METER 1000.0

typedef enum
{
    ALPHA1 = 0,A1,THETA1,D1,BETA1,
    ALPHA2,A2,THETA2,D2,BETA2,
    ALPHA3,A3,THETA3,D3,BETA3,
    ALPHA4,A4,THETA4,D4,BETA4,
    ALPHA5,A5,THETA5,D5,BETA5,
    ALPHA6,A6,THETA6,D6,BETA6,
}CALIBRATE_PARA;

struct POSITION
{
    double x;
    double y;
    double z;
};



class KinematicsCalibration
{
public:
    KinematicsCalibration();
    ~KinematicsCalibration();

    void initiallDHPara();

    void setRobotDHPara(ROBOT_TYPE type);

    void loadMeasuredData(std::string datafile);
    void loadInputJointAngle(std::string  datafile);
    void loadInputToolData(std::string  datafile);

    void setCheckFlag(int index, bool value);
    void setCalibrationBeta(bool value);

    double getToolPara(std::string index);
    void getAllPara(double allpara[]);
    void getAllDPara(double all_d_para[]);

    //
    int calibrationDHParaNum();
    int calibrationParaNum(int calibration_dh_para_num);

    bool calibration();

    void GetEleIdentifyMatrix(double allPara[], int index, RVector& Phai_i, double& E_i);

    void GetIdentifyMatrix(RMatrix& Phai_m, RVector& Line_v, double& Eerror);

    void getIncrePara(RVector& d_para, double& Eerror_last);

    void updataAllPara(const RVector& d_para);

    void outputClibrationDPara(std::string  path);

    void showdpara();


//    static inline char * cp_str(enum CALIBRATE_PARA dpara);



private:
    bool check_flag_[MAX_IDEN_PARA_NUM];
    bool calibration_beta_;
    int measure_data_length_;
    std::vector<double> measure_line_data_;
    std::vector<vector<double> > input_joint_angle_;
    POSITION tool_para_;
    POSITION measurement_para_;

    RobotKinematics *rk;
    int calibration_dh_para_num_;//dh
    int calibration_para_num_;//dh + tool + measure;

    double all_para_[36];
    double all_d_para_[36];
    double all_dh_para_[30];//dh + tool + measure;//initiallized when path changed

    CALIBRATE_PARA dpara;


};

#endif // KINEMATICSCALIBRATION_H
