#ifndef CALIBRATIONTYPE
#define CALIBRATIONTYPE

#include "robotkinematics.h"

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream> // string stream
#include <stdlib.h>
#include <ctime>

using namespace std;


#define MAX_IDEN_PARA_NUM 40  //maximum identification parameter number, all dh para;
#define DOF 6
#define POSITION_DOF 3
#define D2r M_PI/180
#define r2D 180/M_PI
#define MM2METER 1.0/1000.0
#define METER2MM 1000.0


enum CALIBRATE_METHOD
{
    CALI_LINE = 0,
    CALI_LEICA,
};

struct POSITION
{
    double x;
    double y;
    double z;
};

struct RPY
{
    double r;// rz
    double p;
    double y;
};

struct CRITER
{
    double meanvalue;
    double maxvalue;
    double rmsvalue;
};

struct PARA
{
    POSITION tool_para;
    POSITION measurement_para;
    RPY measurement_rpy_para;
    double dh_para[24];
    double beta[6];
};

class CaliType
{

public:
    CaliType();
    ~CaliType();

    virtual int loadMeasuredData(const std::string& datafile) = 0;
    virtual int loadInputJointAngle(const std::string&  datafile) = 0;
    virtual int loadInputToolData(const std::string&  datafile) = 0;

    virtual void getFrameParaJacobian(double& Se, RMatrix& Je, RVector& Fe, RVector& mt_para, RVector& measure_data_i, RVector& joint_i) = 0;

    virtual void GetEleIdentifyMatrix(RMatrix& Phai_i, RVector& Fe_i, RVector& measure_data_i, RVector& joint_i) = 0;//all para include dh tool and measure

    virtual bool calError() = 0;

    bool estParaOfFrame();


    void GetIdentifyMatrix(RMatrix& Phai, RVector& Fe);

    bool getIncrePara(RVector& d_para);
    RVector calAllDpara(const RVector& d_para);

public:
    bool check_flag_[MAX_IDEN_PARA_NUM];
    bool calibration_beta_;
    int GN_;//4 0r 5
    int para_mt_num_;//6
    int para_total_num_;// 36 or30
    int All_para_num_;//36

    int calibration_para_num_;// check flag ++

    PARA cali_para_;
    PARA nom_cali_para_;

    CRITER criter_after_;

    CALIBRATE_METHOD cali_method_;

protected:
    int data_dim_;//measure dim;
    int measure_data_length_;
    std::vector<vector<double> > measure_data_;
    std::vector<vector<double> > input_joint_angle_;
    RobotKinematics *rk;
};



template <class Type>
Type stringtoNum(const string& str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}



#endif // CALIBRATIONTYPE

