#ifndef CALI_LEICA_H
#define CALI_LEICA_H

#include "CalibrationType.h"

class CaliLeica:public CaliType
{
public:
    CaliLeica();
    ~CaliLeica();

    virtual bool loadMeasuredData(const std::string& datafile);
    virtual bool loadInputJointAngle(const std::string&  datafile);
    virtual void loadInputToolData(const std::string&  datafile);


    virtual void getFrameParaJacobian(double& Se, RMatrix& Je, RVector& Fe, RVector& mt_para, RVector& measure_data_i, RVector& joint_i);
    virtual void GetEleIdentifyMatrix(RMatrix& Phai_i, RVector& Fe_i, RVector &measure_data_i, RVector& joint_i);//all para include dh tool and measure

    virtual bool calError();
};

#endif // CALILEICA

