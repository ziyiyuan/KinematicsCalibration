#include "robotkinematics.h"

RobotKinematics::RobotKinematics():robot_type_(AUBO_I3)
{
    setRobotDHPara(robot_type_);
}

void RobotKinematics::setRobotDHPara(ROBOT_TYPE type)
{
    robot_type_ = type;

    switch (type) {
    case AUBO_I3:
        dh_para_.a2 = 0.266;
        dh_para_.a3 = 0.2565;
        dh_para_.d1 = 0.157;
        dh_para_.d2 = 0.119;
        dh_para_.d5 = 0.1025;
        dh_para_.d6 = 0.094;
        break;

    case AUBO_I5:
        dh_para_.a2 = 0.408;
        dh_para_.a3 = 0.376;
        dh_para_.d1 = 0.0985;
        dh_para_.d2 = 0.1215;
        dh_para_.d5 = 0.1025;
        dh_para_.d6 = 0.094;
        break;

    case AUBO_I7:
        dh_para_.a2 = 0.552;
        dh_para_.a3 = 0.495;
        dh_para_.d1 = 0.1632;
        dh_para_.d2 = 0.178;
        dh_para_.d5 = 0.1025;
        dh_para_.d6 = 0.094;
        break;

    case AUBO_I10:
        dh_para_.a2 = 0.647;
        dh_para_.a3 = 0.6005;
        dh_para_.d1 = 0.1632;
        dh_para_.d2 = 0.2013;
        dh_para_.d5 = 0.1025;
        dh_para_.d6 = 0.094;
        break;
    }
}

void RobotKinematics::getRobotDHPara(double& a2, double& a3, double& d1, double& d2, double& d5, double& d6)
{
    a2 = dh_para_.a2;
    a3 = dh_para_.a3;
    d1 = dh_para_.d1;
    d2 = dh_para_.d2;
    d5 = dh_para_.d5;
    d6 = dh_para_.d6;
}

RMatrix RobotKinematics::homogeneousTransfer(double alpha, double a, double d, double theta)
{
    RMatrix T(4,4);

    //modified DH
    T(0,0) = cos(theta);
    T(0,1) = -sin(theta);
    T(0,2) = 0;
    T(0,3) = a;

    T(1,0) = sin(theta)*cos(alpha);
    T(1,1) = cos(theta)*cos(alpha);
    T(1,2) = -sin(alpha);
    T(1,3) = -sin(alpha)*d;

    T(2,0) = sin(theta)*sin(alpha);
    T(2,1) = cos(theta)*sin(alpha);
    T(2,2) = cos(alpha);
    T(2,3) = cos(alpha)*d;

    T(3,0) = 0;
    T(3,1) = 0;
    T(3,2) = 0;
    T(3,3) = 1;
    //            T <<    cos(theta),            -sin(theta),           0,           a,
    //            sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
    //            sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),  cos(alpha)*d,
    //            0,                     0,                     0,           1;
    return T;
}

// joint 0(base) 1 2 3 4 5 6 in base;
std::vector<RMatrix> RobotKinematics::GetAllTransMatrixtobase(double dh_para[], bool caliBeta, double beta[])//all para include dh tool and measure
{
    RMatrix Tbase(4), Tadd(4,4);
    std::vector<RMatrix> T_base(DOF+1);

    T_base[0] = Tbase;

//    double Tpos[3] = {dh_para[30], dh_para[31], dh_para[32]};
//    Ttool_f = RMatrix(R2, Tpos);
//    T[DOF+1] = Ttool_f;
    for(int i = 0; i < DOF; i++)
    {
        if(caliBeta)
            Tadd = fKWithBeta(dh_para[0+4*i],dh_para[1+4*i], dh_para[2+4*i], dh_para[3+4*i], beta[i]);
        else
            Tadd = homogeneousTransfer(dh_para[0+4*i],dh_para[1+4*i], dh_para[2+4*i], dh_para[3+4*i]);
        T_base[i+1] = T_base[i]*Tadd;
    }

    return T_base;
}

RMatrix RobotKinematics::fKWithBeta(double alpha, double a, double d, double theta, double beta)
{
    RMatrix T(4,4), T1(4,4), R2(3,3), R(3,3);
    RVector P(3);

    T1 = homogeneousTransfer(alpha, a, d, theta);
    R2 = RotY(beta);
    R = R2 * T1.subMatrix(0,0,2,2);
    P = R2 * T1.subVector(0,2,3,COL);

    for(int i = 0; i++; i < 3)
    {
        for(int j = 0; j++; j < 3)
            T(i,j) = R(i,j);
        T(i,4) = P(i);
    }
    T(3,0) = 0;
    T(3,1) = 0;
    T(3,2) = 0;
    T(3,3) = 1;

    return T;
}


RMatrix RobotKinematics::RotZ(double t)
{
    RMatrix rz = RMatrix::eye(3);
    double   ct = cos(t);
    double   st = sin(t);
    rz(0,0) = ct;
    rz(0,1) = -st;
    rz(1,0) = st;
    rz(1,1) = ct;

    return rz;
}

RMatrix RobotKinematics::RotY(double t)
{
    RMatrix ry = RMatrix::eye(3);

    double   ct = cos(t);
    double   st = sin(t);
    ry(0,0) = ct;
    ry(0,2) = st;
    ry(2,0) = -st;
    ry(2,2) = ct;
    return ry;
}

RMatrix RobotKinematics::RotX(double t)
{
    RMatrix rx = RMatrix::eye(3);
    double   ct = cos(t);
    double   st = sin(t);
    rx(1,1) = ct;
    rx(1,2) = -st;
    rx(2,1) = st;
    rx(2,2) = ct;

    return rx;
}

RMatrix RobotKinematics::fKFlangeInBase(double dh_para[], RVector& joint)
{
    RMatrix Tadd(4);
    RMatrix T = RMatrix::eye(4);
    for(int i = 0; i < DOF; i++)
    {
        Tadd = homogeneousTransfer(dh_para[0+4*i],dh_para[1+4*i], dh_para[2+4*i], dh_para[3+4*i] + joint(i));
        T = T*Tadd;
    }
    return T;
}

RMatrix RobotKinematics::RPToT(const RMatrix rot, double *eetrans)
{
    RMatrix T(4,4);
    int iRow = 4;
    int iCol = 4;
    for(size_t i = 0;i < iRow-1;i++)
    {
       for(size_t j = 0;j < iCol-1;j++)
           T(i,j) = rot(i,j);
        T(i,3) = eetrans[i];
    }
    T(3,0) = 0;
    T(3,1) = 0;
    T(3,2) = 0;
    T(3,3) = 1;

    return T;
}




