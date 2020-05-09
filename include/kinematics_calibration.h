#ifndef KINEMATICSCALIBRATION_H
#define KINEMATICSCALIBRATION_H

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream> // string stream
#include <math.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctime>

#include "rl_interface/robot_interface.hpp"

#define MAX_IDEN_PARA_NUM 40
#define D2r M_PI/180
#define r2D 180/M_PI
#define POSITION_DOF 3

using namespace std;
using namespace ARAL;

typedef enum
{
    ALPHA1 = 0,A1,D1,THETA1,BETA1,
    ALPHA2,A2,D2,THETA2,BETA2,
    ALPHA3,A3,D3,THETA3,BETA3,
    ALPHA4,A4,D4,THETA4,BETA4,
    ALPHA5,A5,D5,THETA5,BETA5,
    ALPHA6,A6,D6,THETA6,BETA6,

} CALIBRATE_PARA;

enum ROBOT_TYPE
{
    AUBO_I3 = 0,
    AUBO_I5,
    AUBO_I7,
    AUBO_I10,

};

struct CRITER
{
    double mean_value;
    double max_value;
    double rms_value;
};

enum CALIBRATE_METHOD
{
    CALI_LINE = 0,      //Dynlog
    CALI_POS,           // Leica, Faro.
};

class KinematicsCalibration
{
public:
    KinematicsCalibration();
    ~KinematicsCalibration();

    bool calibration();

    bool LoadData(const std::string& datafile);

    void calAllPara();

    void setRobotType(const ROBOT_TYPE type);

    void setCaliType(const std::string& type);//

    unsigned int getCaliMethod();

    void setCalibrationBeta(const bool value);
    bool getCalibrationBeta();

    void setPara();

    void getAllPara(double allpara[]);

    void getAllDPara(double all_d_para[]);

    int getMtNum();

    double getToolPara(const std::string index);
    void getCriter(double output_criter[]);

    void outputClibrationDPara(const std::string  path, const std::string data_file);

    void setCheckFlag(const int index, const bool value);

    int loadMeasuredData(const std::string& datafile);

    int loadInputJointAngle(const std::string&  datafile);

    int loadInputToolData(const std::string&  datafile);

private:

    int dof_;

    ROBOT_TYPE robot_type_;

    CALIBRATE_METHOD cali_method_;

    bool check_flag_[MAX_IDEN_PARA_NUM];
    bool kc_cali_beta_; //
    bool DH_check_flag_[MAX_IDEN_PARA_NUM];// para in interface;

    int data_dim_;
    int para_mt_num_;//
    int All_para_num_;//

    int measure_data_length_;//
    std::vector<vector<double> > measure_data_;
    std::vector<vector<double> > input_joint_angle_;

    double d_allpara_[MAX_IDEN_PARA_NUM];
    double all_para_[MAX_IDEN_PARA_NUM];

    RLIntface *aral_interface_;
    KinematicsCalibrationResult cali_result_;

};

#endif // KINEMATICSCALIBRATION_H
