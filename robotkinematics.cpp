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
        dh_para_.d1 = 0.122;
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

RMatrix RobotKinematics::homogeneousTransfer(double alpha, double a, double theta, double d)
{
    RMatrix T(4,4);

    //modified DH
    T.value[0][0] = cos(theta);
    T.value[0][1] = -sin(theta);
    T.value[0][2] = 0;
    T.value[0][3] = a;

    T.value[1][0] = sin(theta)*cos(alpha);
    T.value[1][1] = cos(theta)*cos(alpha);
    T.value[1][2] = -sin(alpha);
    T.value[1][3] = -sin(alpha)*d;

    T.value[2][0] = sin(theta)*sin(alpha);
    T.value[2][1] = cos(theta)*sin(alpha);
    T.value[2][2] = cos(alpha);
    T.value[2][3] = cos(alpha)*d;

    T.value[3][0] = 0;
    T.value[3][1] = 0;
    T.value[3][2] = 0;
    T.value[3][3] = 1;
    //            T <<    cos(theta),            -sin(theta),           0,           a,
    //            sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
    //            sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),  cos(alpha)*d,
    //            0,                     0,                     0,           1;
    return T;
}

std::vector<RMatrix> RobotKinematics::GetAllTransMatrixtobase(double allPara[], bool caliBeta)//all para include dh tool and measure
{
    RMatrix Tbase(4), Ttool_f(4,4), R2(3);
    std::vector<RMatrix> T(DOF+2);
    std::vector<RMatrix> T_base(DOF+2);


    T[0] = Tbase;

    double Tpos[3] = {allPara[30], allPara[31], allPara[32]};
    Ttool_f = RMatrix(R2, Tpos);
    T[DOF+1] = Ttool_f;

    if(caliBeta)
    {

        for(int i = 0; i < DOF; i++)
        {
            T[i+1] = fKWithBeta(allPara[0+5*i],allPara[1+5*i], allPara[2+5*i], allPara[3+5*i], allPara[4+5*i]);
        }
    }
    else
    {
        for(int i = 0; i < DOF; i++)
        {
            T[i+1] = homogeneousTransfer(allPara[0+5*i],allPara[1+5*i], allPara[2+5*i], allPara[3+5*i]);        }

    }

    T_base[0] = T[0];
    for(int i = 0; i < DOF + 1; i++)
    {
        T_base[i+1] = T_base[i]*T[i+1];
    }
    return T_base;
}

RMatrix RobotKinematics::fKWithBeta(double alpha, double a, double theta, double d, double beta)
{
    RMatrix T(4,4), T1(4,4), T2(4,4), R2(3,3);

    T1 = homogeneousTransfer(alpha, a, theta, d);
    double eetrans[] = {0,0,0};
    R2 = RMatrix::RotY(beta);
    T2 = RMatrix(R2, eetrans);

    T = T2*T1;
    return T;
}

