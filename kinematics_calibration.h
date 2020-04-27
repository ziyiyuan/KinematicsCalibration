#ifndef KINEMATICSCALIBRATION_H
#define KINEMATICSCALIBRATION_H

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream> // string stream
#include <math.h>
#include <string>
#include <stdio.h>

#include "/home/lg/Projects/aral_export/include/rl_interface/robot_interface.hpp"


#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream> // string stream
#include <stdlib.h>
#include <ctime>
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


struct PARA
{
    POSITION tool_para;
    POSITION measurement_para;
    RPY measurement_rpy_para;
    double dh_para[24];
    double beta[6];
};

struct CRITER
{
    double mean_value;
    double max_value;
    double rms_value;
};
using namespace std;

using namespace std;
using namespace ARAL;


enum CALIBRATE_METHOD
{
    CALI_LINE = 0,      //Dynlog
    CALI_POS,           // Leica, Faro.
};

#define MAX_IDEN_PARA_NUM 40
#define D2r M_PI/180
#define r2D 180/M_PI
#define POSITION_DOF 3

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
    unsigned int getCaliMethod();

    void setCalibrationBeta(bool value);
    bool getCalibrationBeta();

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

    void setCheckFlag(int index, bool value);

    void getCalibrationNum();

private:
    double d_allpara[MAX_IDEN_PARA_NUM];
    double all_para_[MAX_IDEN_PARA_NUM];

    bool kc_cali_beta_;
    bool kc_choose_para_[MAX_IDEN_PARA_NUM];// para in interface;

    RLIntface *aral_interface_;

    std::vector<vector<double> > measure_data_;
    std::vector<vector<double> > input_joint_angle_;
    bool check_flag_[MAX_IDEN_PARA_NUM];
    bool calibration_beta_;

public:
    KinematicsCalibrationResult cali_result_;


    int loadMeasuredData(const std::string& datafile);
    int loadInputJointAngle(const std::string&  datafile);
    int loadInputToolData(const std::string&  datafile);

    bool calError();

public:
    int GN_;//4 0r 5
    int para_mt_num_;//6
    int para_total_num_;// 36 or30
    int All_para_num_;//36

    int dof_;

    int calibration_para_num_;// check flag ++
    int choose_cali_num_;

    PARA cali_para_;
    PARA nom_cali_para_;

    CRITER criter_after_;

    CALIBRATE_METHOD cali_method_;

protected:
    int data_dim_;      //measure dim;
    int measure_data_length_;
};

#endif // KINEMATICSCALIBRATION_H
