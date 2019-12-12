#ifndef CALI_LINE_H
#define CALI_LINE_H

#include "CalibrationType.h"

class CaliLine:public CaliType
{
public:
    CaliLine();
    ~CaliLine();

    bool loadMeasuredData(const std::string& datafile);
    bool loadInputJointAngle(const std::string&  datafile);
    void loadInputToolData(const std::string&  datafile);
//    bool estParaOfFrame();


    void getFrameParaJacobian(double& Se, RMatrix& Je, RVector& Fe, RVector& mt_para, RVector& measure_data_i, RVector& joint_i);
    void GetEleIdentifyMatrix(RMatrix& Phai_i, RVector& Fe_i, RVector &measure_data_i, RVector& joint_i);//all para include dh tool and measure

    bool calError();


};




#endif // CALILINE

