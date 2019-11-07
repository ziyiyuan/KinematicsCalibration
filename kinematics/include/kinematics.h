#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "../utility/include/rvector.h"
#include "../utility/include/rmatrix.h"


#define i5_PARAMS
#define MIN -175    //随机数产生的范围
#define MAX 175
#define MAXIUM_ITERATOR 10

namespace
{
const double ZERO_THRESH = 1e-5;
const double PI = M_PI;
#ifdef i5_PARAMS
const double d1 =  0.122;
const double d2 =  0.1215;
const double d5 =  0.1025;
const double d6 =  0.094;
const double a2 =  0.408;
const double a3 =  0.376;
#endif

class Kinematics
{
public:
    Kinematics();
    ~Kinematics();
    void ForwardKinematics(RMatrix& T, const RVector q, int index);




};




















#endif // KINEMATICS_H
