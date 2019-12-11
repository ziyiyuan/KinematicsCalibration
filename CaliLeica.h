#ifndef CALI_LEICA_H
#define CALI_LEICA_H

#include "CalibrationType.h"

class CaliLeica:public CaliType
{
public:
    CaliLeica();
    ~CaliLeica();

    bool loadMeasuredData(const std::string& datafile);
    bool loadInputJointAngle(const std::string&  datafile);
    void loadInputToolData(const std::string&  datafile);
    bool estParaOfFrame();


    void getFrameParaJacobian(double& Se, RMatrix& Je, RVector& Fe, RVector& mt_para, RVector& measure_data_i, RVector& joint_i);
    void GetEleIdentifyMatrix(RVector& Phai_i, RVector& Fe_i, RVector &measure_data_i, RVector& joint_i);//all para include dh tool and measure

    void GetIdentifyMatrix(RMatrix& Phai, RVector& Fe, double& Eerror);

    bool getIncrePara(RVector& d_para, double& Eerror_last);

    bool calError();
};

#endif // CALILEICA

