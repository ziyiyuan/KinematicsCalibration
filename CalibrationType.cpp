#include "CalibrationType.h"

CaliType::CaliType()
{

}

CaliType::~CaliType()
{

}

RVector CaliType::updataAlldPara(const RVector& d_para)
{
    RVector d_all_para(All_para_num_);
    int j = 0;
    for(int i = 0; i < All_para_num_; i++)
    {
        if(check_flag_[i])
        {
            d_all_para(i) = d_para(j);
            j++;
        }
        else
            d_all_para(i) = 0.0;
    }
    return d_all_para;
}


