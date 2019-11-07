#ifndef ROBOTKINEMATICS_H
#define ROBOTKINEMATICS_H

#include "./utility/include/rvector.h"
#include "./utility/include/rmatrix.h"

#define DOF 6

enum ROBOT_TYPE
{
    AUBO_I3 = 0,
    AUBO_I5,
    AUBO_I7,
    AUBO_I10,

};

struct DH_PARA
{
    double a2;
    double a3;
    double d1;
    double d2;
    double d5;
    double d6;
};

class RobotKinematics
{
public:
    RobotKinematics();

    void setRobotDHPara(ROBOT_TYPE type);
    void getRobotDHPara(double& a2, double& a3, double& d1, double& d2, double& d5, double& d6);


    RMatrix homogeneousTransfer(double alpha, double a,  double d, double theta);

    RMatrix fKWithBeta(double alpha, double a, double theta, double d, double beta);

    std::vector<RMatrix> GetAllTransMatrixtobase(double allPara[], bool caliBeta);


private:
    DH_PARA dh_para_;
    ROBOT_TYPE robot_type_;

};

#endif // ROBOTKINEMATICS_H
