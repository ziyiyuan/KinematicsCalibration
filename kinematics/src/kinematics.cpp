#include "../include/kinematics.h"

int SIGN(double x)
{
    return (x > 0) - (x < 0);
}

// input radian
void ForwardKinematics(RMatrix& T, const RVector q, int index)
{
    double  q1 = q.value[0], q2 = q.value[1], q3 = q.value[2], q4 = q.value[3], q5 = q.value[4], q6 = q.value[5];
    double  C1 = cos(q1), C2 = cos(q2), C4 = cos(q4), C5 = cos(q5), C6 = cos(q6);
    double  C23 = cos(q2 - q3), C234 = cos(q2 - q3 + q4), C2345 = cos(q2 - q3 + q4 - q5), C2345p = cos(q2 - q3 + q4 + q5);
    double  S1 = sin(q1), S2 = sin(q2), S4 = sin(q4), S5 = sin(q5), S6 = sin(q6);
    double  S23 = sin(q2 - q3), S234 = sin(q2 - q3 + q4);
    if (index == 1)
    {
        T.value[0][0] = -C1;
        T.value[0][1] = S1;
        T.value[0][2] = 0;
        T.value[0][3] = 0;

        T.value[1][0] = -S1;
        T.value[1][1] = -C1;
        T.value[1][2] = 0;
        T.value[1][3] = 0;

        T.value[2][0] = 0;
        T.value[2][1] = 0;
        T.value[2][2] = 1;
        T.value[2][3] = d1;

        T.value[3][0] = 0;
        T.value[3][1] = 0;
        T.value[3][2] = 0;
        T.value[3][3] = 1;
    }
    else if(index == 2)
    {
        T.value[0][0] = -C1 * S2;
        T.value[0][1] = -C1 * C2;
        T.value[0][2] = S1;
        T.value[0][3] = d2 * S1;

        T.value[1][0] = -S1 * S2;
        T.value[1][1] = -S1 * C2;
        T.value[1][2] = -C1;
        T.value[1][3] = -d2 * C1;

        T.value[2][0] = C2;
        T.value[2][1] = -S2;
        T.value[2][2] = 0;
        T.value[2][3] = d1;

        T.value[3][0] = 0;
        T.value[3][1] = 0;
        T.value[3][2] = 0;
        T.value[3][3] = 1;
    }
    else if(index == 3)
    {
        T.value[0][0] = -C1 * S23;
        T.value[0][1] = C1 * C23;
        T.value[0][2] = -S1;
        T.value[0][3] = d2 * S1 - a2 * C1 * S2;

        T.value[1][0] = -S1 * S23;
        T.value[1][1] = S1 * C23;
        T.value[1][2] = C1;
        T.value[1][3] = -d2 * C1 - a2 * S1 * S2;

        T.value[2][0] = C23;
        T.value[2][1] = S23;
        T.value[2][2] = 0;
        T.value[2][3] = d1 + a2 * C2;

        T.value[3][0] = 0;
        T.value[3][1] = 0;
        T.value[3][2] = 0;
        T.value[3][3] = 1;
    }
    else if(index == 4)
    {
        T.value[0][0] = C1 * C234;
        T.value[0][1] = -C1 * S234;
        T.value[0][2] = S1;
        T.value[0][3] = d2 * S1 - C1 * (a2 * S2 + a3 * S23);

        T.value[1][0] = S1 * S234;
        T.value[1][1] = -S1 * C234;
        T.value[1][2] = -C1;
        T.value[1][3] = -d2 * C1 - S1 * (a2 * S2 + a3 * S23);

        T.value[2][0] = S234;
        T.value[2][1] = C234;
        T.value[2][2] = 0;
        T.value[2][3] = d1 + a2 * C2 + a3 * C23;

        T.value[3][0] = 0;
        T.value[3][1] = 0;
        T.value[3][2] = 0;
        T.value[3][3] = 1;
    }
    else if(index == 5)//HERE
    {
        T.value[0][0] = C1 * C234;
        T.value[0][1] = -C1 * S234;
        T.value[0][2] = S1;
        T.value[0][3] = d2 * S1 - C1 * (a2 * S2 + a3 * S23);

        T.value[1][0] = S1 * S234;
        T.value[1][1] = -S1 * C234;
        T.value[1][2] = -C1;
        T.value[1][3] = -d2 * C1 - S1 * (a2 * S2 + a3 * S23);

        T.value[2][0] = S234;
        T.value[2][1] = C234;
        T.value[2][2] = 0;
        T.value[2][3] = d1 + a2 * C2 + a3 * C23;

        T.value[3][0] = 0;
        T.value[3][1] = 0;
        T.value[3][2] = 0;
        T.value[3][3] = 1;
    }



    T.value[0][0] = -C6 * S1 * S5 + C1 * (C234 * C5 * C6 - S234 * S6);
    T.value[0][1] = S1 * S5 * S6 - C1 * (C4 * C6 * S23 + C23 * C6 * S4 + C234 * C5 * S6);
    T.value[0][2] = C5 * S1 + C1 * C234 * S5;
    T.value[0][3] = (d2 + C5 * d6) * S1 - C1 * (a2 * S2 + (a3 + C4 * d5) * S23 + C23 * d5 * S4 - C234 * d6 * S5);

    T.value[1][0] = C234 * C5 * C6 * S1 + C1 * C6 * S5 - S1 * S234 * S6;
    T.value[1][1] = -C6 * S1 * S234 - (C234 * C5 * S1 + C1 * S5) * S6;
    T.value[1][2] = -C1 * C5 + C234 * S1 * S5;
    T.value[1][3] = -C1 * (d2 + C5 * d6) - S1 * (a2 * S2 + (a3 + C4 * d5) * S23 + C23 * d5 * S4 - C234 * d6 * S5);

    T.value[2][0] = C5 * C6 * S234 + C234 * S6;
    T.value[2][1] = C234 * C6 - C5 * S234 * S6;
    T.value[2][2] = S234 * S5;
    T.value[2][3] = d1 + a2 * C2 + a3 * C23 + d5 * C234 + d6 * C2345/2 - d6 * C2345p / 2;

    T.value[3][0] = 0;
    T.value[3][1] = 0;
    T.value[3][2] = 0;
    T.value[3][3] = 1;
}

// output radian//MatrixXd q_sols(6,8);
int inverse(RMatrix& q_sols, const RMatrix T, bool& singularity)
{
    int num_sols = 0;
    double nx = T.value[0][0]; double ox = T.value[0][1]; double ax = T.value[0][2]; double px = T.value[0][3];
    double ny = T.value[1][0]; double oy = T.value[1][1]; double ay = T.value[1][2]; double py = T.value[1][3];
    double nz = T.value[2][0]; double oz = T.value[2][1]; double az = T.value[2][2]; double pz = T.value[2][3];

    //////////////////////// shoulder rotate joint (q1) //////////////////////////////
    RVector q1(2);

    double A1 = d6 * ay - py;
    double B1 = d6 * ax - px;
    double R1 = A1 * A1 + B1 * B1;
    if((fabs(A1) < ZERO_THRESH) && (fabs(B1) < ZERO_THRESH))
        return num_sols;
    else if(R1 < d2*d2)
        return num_sols;
    else
    {
        q1.value[0] =  atan2(A1, B1) -  atan2(d2, sqrt(R1- d2 * d2));
        q1.value[1] =  atan2(A1, B1) -  atan2(d2, -sqrt(R1 - d2 * d2));
        for(int i = 0; i < 2; i++)
        {
            while(q1.value[i] > M_PI)
                q1.value[i] -= 2 * M_PI;
            while(q1.value[i] < -M_PI)
                q1.value[i] += 2 * M_PI;
        }
    }

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    RMatrix q5(2,2);

    for(int i = 0; i < 2; i++)
    {

        double C1 = cos(q1.value[i]), S1 = sin(q1.value[i]);
        double B5 = -ay * C1 + ax * S1;
        double R5 = (-ny * C1 + nx * S1) * (-ny * C1 + nx * S1) + (-oy * C1 + ox * S1) * (-oy * C1 + ox * S1);

        if(fabs(B5) < ZERO_THRESH && fabs(R5) < ZERO_THRESH)
        {
            singularity = true;
            continue;
        }
        else
        {
            q5.value[i][0] = atan2(sqrt(R5), B5);
            q5.value[i][1] = atan2(-sqrt(R5), B5);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
    double q6;
    RVector q3(2), q2(2), q4(2);

    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            // wrist 3 joint (q6) //
            double C1 = cos(q1.value[i]), S1 = sin(q1.value[i]);
            double S5 = sin(q5.value[i][j]);

            double A6 = (-oy * C1 + ox * S1);
            double B6 = (ny * C1 - nx * S1);

            if((fabs(A6) < ZERO_THRESH) && (fabs(B6) < ZERO_THRESH))
            {
                singularity = true;
                break;
            }
            //            //                  return num_sols;
            //            else if(fabs(S5) < ZERO_THRESH)
            //                q6= atan2(A6*S5,B6*S5);
            //                  return num_sols;
            else
                q6 = atan2(A6 * S5, B6 * S5);

            /////// joints (q3,q2,q4) //////
            double C6 = cos(q6);
            double S6 = sin(q6);

            double pp1 = C1 * (ax * d6 - px + d5 * ox * C6 + d5 * nx * S6) + S1 * (ay * d6 - py + d5 * oy * C6 + d5 * ny * S6);
            double pp2 = -d1 - az * d6 + pz - d5 * oz * C6 - d5 * nz * S6;
            double B3 = (pp1 * pp1 + pp2 * pp2 - a2 * a2 - a3 * a3) / (2 * a2 * a3);


            if((fabs(B3)-1) > ZERO_THRESH)
            {
                singularity = true;
                continue;
            }
            //                  return num_sols;
            else
            {
                double Sin3 = sqrt(fabs(1 - B3 * B3));
                q3.value[0] = atan2(Sin3, B3);
                q3.value[1] = atan2(-Sin3, B3);
            }

            for(int k = 0; k < 2; k++)
            {

                double C3 = cos(q3.value[k]), S3 = sin(q3.value[k]);
                double A2 = pp1 * (a2 + a3 * C3) + pp2 * (a3 * S3);
                double B2 = pp2 * (a2 + a3 * C3) - pp1 * (a3 * S3);
                if((fabs(A2) < ZERO_THRESH) && (fabs(B2) < ZERO_THRESH))
                {
                    singularity = true;
                    continue;
                }
                else
                    q2.value[k] = atan2(A2, B2);

                double C2 = cos(q2.value[k]), S2 = sin(q2.value[k]);

                double A4 = -C1 * (ox * C6 + nx * S6) - S1 * (oy * C6 + ny * S6);
                double B4 = oz * C6 + nz * S6;
                double A41 = pp1 - a2 * S2;
                double B41 = pp2 - a2 * C2;
                q4.value[k] = atan2(A4, B4) - atan2(A41, B41);
                while(q4.value[k] > M_PI)
                    q4.value[k] -= 2 * M_PI;
                while(q4.value[k] < -M_PI)
                    q4.value[k] += 2 * M_PI;

                q_sols.value[0][num_sols] = q1.value[i];    q_sols.value[1][num_sols] = q2.value[k];
                q_sols.value[2][num_sols] = q3.value[k];    q_sols.value[3][num_sols] = q4.value[k];
                q_sols.value[4][num_sols] = q5.value[i][j]; q_sols.value[5][num_sols] = q6;
                num_sols++;
            }
        }
    }
    return num_sols;
}

void transfer(RMatrix& T, double alpha, double a, double d, double theta)
{
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
}



RVector Vex(RMatrix A)
{
    RVector S(3);
    S.value[0] = 0.5 * (A.value[2][1] - A.value[1][2]);
    S.value[1] = 0.5 * (A.value[0][2] - A.value[2][0]);
    S.value[2] = 0.5 * (A.value[1][0] - A.value[0][1]);
    return S;
}

void GetJacobian(RMatrix& J, RMatrix& A, RVector q)
{
    RMatrix JJ2(3,6);
    RVector z1(3),z2(3),z3(3),z4(3),z5(3),z6(3),z7(3),o1(3),o2(3),o3(3),o4(3),o5(3),o6(3),o7(3),J1(3),J2(3),J3(3),J4(3),J5(3),J6(3),J7(3);
    RMatrix T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4), T67(4,4);
    RMatrix T1(4,4),T2(4,4),T3(4,4),T4(4,4),T5(4,4),T6(4,4),T7(4,4);


    transfer(T01, 0, 0, d1, q.value[0] + M_PI);
    transfer(T12, -M_PI / 2, 0, d2, q.value[1] - M_PI / 2);
    transfer(T23, M_PI, a2, 0, q.value[2]);
    transfer(T34, M_PI, a3, 0, q.value[3] - M_PI / 2);
    transfer(T45, -M_PI / 2, 0, d5, q.value[4]);
    transfer(T56, M_PI / 2, 0, d6, q.value[5]);


    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
//            T67.value[i][j] = RobotControl::m_toolOrientation[3*i+j];
        }
//        T67.value[i][3] = RobotControl::m_toolPosition[i];
    }
    T67.value[3][3] = 1;

    //    R = RMatrix.RotZ(M_PI/4);
    //    T67 << 0, -1, 0, 0,
    //           1, 0, 0, 0,
    //           0, 0, 1, 0.07,
    //           0, 0, 0, 1;

    T1 = T01;
    T2 = T1 * T12;
    T3 = T2 * T23;
    T4 = T3 * T34;
    T5 = T4 * T45;
    T6 = T5 * T56;
    T7 = T6 * T67;

    z1 = RMatrix::subRVector(T1,0,2,2,"Column");
    z2 = RMatrix::subRVector(T2,0,2,2,"Column");
    z3 = RMatrix::subRVector(T3,0,2,2,"Column");
    z4 = RMatrix::subRVector(T4,0,2,2,"Column");
    z5 = RMatrix::subRVector(T5,0,2,2,"Column");
    z6 = RMatrix::subRVector(T6,0,2,2,"Column");

    o1 = RMatrix::subRVector(T1,0,2,3,"Column");
    o2 = RMatrix::subRVector(T2,0,2,3,"Column");
    o3 = RMatrix::subRVector(T3,0,2,3,"Column");
    o4 = RMatrix::subRVector(T4,0,2,3,"Column");
    o5 = RMatrix::subRVector(T5,0,2,3,"Column");
    o6 = RMatrix::subRVector(T6,0,2,3,"Column");
    o7 = RMatrix::subRVector(T6,0,2,3,"Column");



    J1 = RVector::cross3(z1, o7 - o1);
    J2 = RVector::cross3(z2, o7 - o2);
    J3 = RVector::cross3(z3, o7 - o3);
    J4 = RVector::cross3(z4, o7 - o4);
    J5 = RVector::cross3(z5, o7 - o5);
    J6 = RVector::cross3(z6, o7 - o6);
    //    J1 = z1.cross(o7 - o1);
    //    J2 = z2.cross(o7 - o2);
    //    J3 = z3.cross(o7 - o3);
    //    J4 = z4.cross(o7 - o4);
    //    J5 = z5.cross(o7 - o5);
    //    J6 = z6.cross(o7 - o6);

    //MatrixXd C(A.rows(), A.cols()+B.cols());
    //C << A, B;
    RMatrix::catRMatrix(J,0,2,0,J1);
    RMatrix::catRMatrix(J,0,2,1,J2);
    RMatrix::catRMatrix(J,0,2,2,J3);
    RMatrix::catRMatrix(J,0,2,3,J4);
    RMatrix::catRMatrix(J,0,2,4,J5);
    RMatrix::catRMatrix(J,0,2,5,J6);

    RMatrix::catRMatrix(JJ2,0,2,0,z1);
    RMatrix::catRMatrix(JJ2,0,2,1,z2);
    RMatrix::catRMatrix(JJ2,0,2,2,z3);
    RMatrix::catRMatrix(JJ2,0,2,3,z4);
    RMatrix::catRMatrix(JJ2,0,2,4,z5);
    RMatrix::catRMatrix(JJ2,0,2,5,z6);

    RMatrix::catRMatrix(J,3,5,0,5,JJ2);

    RMatrix RT(3,3), R(3,3),S(3,3), B(3,3);
    R = RMatrix::subRMatrix(T7,0,2,0,2);
//    RT = RMatrix::RTranspose(R);
    S =  RVector::Skew(o7);

    RMatrix C = S*R;
    RMatrix::catRMatrix(A,0,2,0,2,R);
    RMatrix::catRMatrix(A,0,2,3,5,C);
    RMatrix::catRMatrix(A,3,5,0,2,B);
    RMatrix::catRMatrix(A,3,5,3,5,R);


    //    for(int i = 0; i < 6; i++)
    //        for(int j = 0; j < 6; j++)
    //        {
    //            JJ.value[i][j] = J(i,j);
    //            AA.value[i][j] = A(i,j);
    //        }


    //    std::cout << J0 << std::endl<< J1 << std::endl;
}
}
