#include "CalibrationType.h"

CaliType::CaliType()
{

}

CaliType::~CaliType()
{

}

RVector CaliType::calAllDpara(const RVector& d_para)
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

bool CaliType::getIncrePara(RVector& d_para)
{
    RMatrix Phai_m(measure_data_length_ * data_dim_, para_total_num_);
    RVector Line_v(measure_data_length_ * data_dim_), Rii(para_total_num_);
    RMatrix Q_phai, R_phai;

    GetIdentifyMatrix(Phai_m, Line_v);

    Phai_m.qrDec(Q_phai,R_phai);

    Rii = R_phai.getDiagVector();

    Phai_m.show("Phai_m");
    Rii.show("Rii");
    R_phai.show("R_phai");
    double qr_rii = 1e-8;
    int emptyCol_num = 0;

    bool check[MAX_IDEN_PARA_NUM];
    for(int i = 0; i < para_total_num_; i++)
        check[i] = true;

    // tool para
    for(int i = 0; i < para_mt_num_; i++)
    {
        if(fabs(Rii(i)) < qr_rii)
        {
            check[i] = false;
            emptyCol_num++;
            check_flag_[i] = false;
        }
    }
    // dhpara
    for (int i = 0; i < DOF; i++)
    {
        for(int j = 0; j < GN_; j++)
        {
            if(fabs(Rii(para_mt_num_ + GN_*i + j)) < qr_rii)
            {
                check[para_mt_num_ + GN_*i + j] = false;

                emptyCol_num++;
                if(calibration_beta_)
                    check_flag_[para_mt_num_ + GN_*i + j] = false;
                else
                    check_flag_[para_mt_num_ + (GN_ + 1)*i + j] = false;
            }
        }
    }

    calibration_para_num_ = para_total_num_ - emptyCol_num;

    RMatrix Phai(measure_data_length_ * data_dim_, calibration_para_num_);
    RMatrix hS_inv(calibration_para_num_,calibration_para_num_);
    RMatrix hS(calibration_para_num_,calibration_para_num_);
    int m = 0;
    for(int j = 0; j < para_total_num_; j++)
    {
        if(check[j])
        {
            for(int k = 0; k < measure_data_length_ * data_dim_; k++)
                Phai(k,m) = Phai_m(k,j);
            m++;
        }
    }

    //    Phai.show("Phai");

    hS = Phai.transpose()*Phai;
    RVector gS = Phai.transpose()*Line_v;
    //    hS.show("hS");

    bool flag = hS.inverse(hS_inv);

    if(flag)
    {
        d_para = (hS_inv * gS)*(-1);
        d_para.show("d_para");
    }
    else
        return false;

    return true;
}

void CaliType::GetIdentifyMatrix(RMatrix& Phai, RVector& Fe)
{

    RMatrix Phai_i(data_dim_, para_total_num_);
    RVector measure_data_i(data_dim_), joint_i(DOF), Fe_i(data_dim_);

    int s = 0;

    for(int i = 0; i < measure_data_length_; i++)
    {
        for(int j = 0; j < data_dim_; j++)
        {
            measure_data_i(j) = measure_data_[i][j];
        }
        for(int j = 0; j < DOF; j++)
        {
            joint_i(j) = input_joint_angle_[i][j];
        }

        GetEleIdentifyMatrix(Phai_i, Fe_i, measure_data_i, joint_i);

//                Phai_i.show("Phai_i");
//                Fe_i.show("Fe_i");
        for(int j = 0; j < data_dim_; j++)
        {
            for(int k = 0; k < para_total_num_; k++)
            {
                Phai(s,k) = Phai_i(j,k);
            }
            Fe(s) = Fe_i(j);
            s++;
        }
    }
}

bool CaliType::estParaOfFrame()
{
    double c1, c2, lamda, rho, sigma, a, b;
    int groupSz, iters, loops, data_index, s;

    c1 = 0.1;
    c2 = 0.7;
    lamda = 1.0;
    groupSz = para_mt_num_/data_dim_;
    iters = 200;
    data_index = -1;

    double Se_i, Se, Se_plus;
    RMatrix Je(groupSz * data_dim_,para_mt_num_), hS, hS_inv(para_mt_num_,para_mt_num_);
    RMatrix Je_i(data_dim_,para_mt_num_);

    RVector gS(para_mt_num_), gS_plus(para_mt_num_), dk;
    RVector Fe_i(data_dim_), Fe(groupSz * data_dim_);
    RVector measure_data_i(data_dim_), joint_i(DOF);

    RVector mt_para(para_mt_num_), mt_para_c(para_mt_num_);
    mt_para.setZero();

    while(iters--)
    {
        for(int i = 0; i < measure_data_length_; i++)
        {
            if(i == measure_data_length_-1)
                bool sss = 0;

            data_index = data_index +1;

            if(data_index >= measure_data_length_)
                data_index = data_index - measure_data_length_;

            if((data_index + 1)%groupSz == 1)
            {
                Je.clear();
                Fe.setZero();
                Se = 0;
                s = 0;
            }
            for(int j = 0; j < data_dim_; j++)
            {
                measure_data_i(j) = measure_data_[data_index][j];
            }
            for(int j = 0; j < DOF; j++)
            {
                joint_i(j) = input_joint_angle_[data_index][j];
            }

            getFrameParaJacobian(Se_i, Je_i, Fe_i, mt_para, measure_data_i, joint_i);

            Se = Se + Se_i;

            for(int q = 0; q < data_dim_; q++)
            {
                for(int p = 0; p < para_mt_num_; p++)
                {
                    Je(s,p) = Je_i(q,p);
                }
                Fe(s) = Fe_i(q);
                s++;
            }

            if((data_index + 1) % groupSz == 0)
            {
                //                               Je.show();
                //                               Fe.show();

                hS = Je.transpose()*Je*2;
                gS = Je.transpose()*Fe*2;

                //               hS.show("HS");
                //               gS.show("GS");
                bool flag = hS.inverse(hS_inv);
                if(flag)
                    dk = (hS_inv * gS)*(-1);
                else
                    return false;

                //                               dk.show("dk");

                rho = 0.1;
                sigma = 0.7;

                srand(time(NULL));
                lamda = (rand() % 10000) * 0.0001 + 0.0001;

                //                               lamda = 0.3;
                a = 0.0;
                b = INFINITY;
                loops = 0;
                while(1)
                {
                    loops = loops + 1;
                    Je.clear();
                    Fe.setZero();
                    Se_plus = 0;
                    s = 0;

                    for(int ii = data_index + 1 - groupSz; ii < data_index+1; ii++)
                    {
                        for(int j = 0; j < data_dim_; j++)
                        {
                            measure_data_i(j) = measure_data_[ii][j];
                        }
                        for(int j = 0; j < DOF; j++)
                        {
                            joint_i(j) = input_joint_angle_[ii][j];
                        }

                        mt_para_c = mt_para + lamda * dk;

                        //                        mt_para_c.show("mt_para_c");

                        getFrameParaJacobian(Se_i, Je_i, Fe_i, mt_para_c, measure_data_i, joint_i);

                        Se_plus = Se_plus + Se_i;

                        for(int q = 0; q < data_dim_; q++)
                        {
                            for(int p = 0; p < para_mt_num_; p++)
                                Je(s,p) = Je_i(q,p);
                            Fe(s) = Fe_i(q);
                            s++;
                        }
                    }
                    gS_plus = Je.transpose()*Fe*2;

                    if(!(Se_plus <= Se + gS.dot(dk) * rho*lamda))
                    {
                        b = lamda;
                        lamda = (lamda + a)/2;
                        continue;
                    }
                    if(!(gS_plus.dot(dk) >= gS.dot(dk)*sigma))
                    {
                        a = lamda;
                        lamda = std::min(2*lamda,(b + lamda)/2);
                        continue;
                    }
                    break;
                }
                mt_para = mt_para + lamda * dk;
            }
        }
    }

    //    mt_para.show("mt_para");

    s = 0;
    cali_para_.measurement_para.x = mt_para(s); s++;
    cali_para_.measurement_para.y = mt_para(s); s++;
    cali_para_.measurement_para.z = mt_para(s); s++;

    if(cali_method_ == CALI_LEICA)
    {
        cali_para_.measurement_rpy_para.r = mt_para(s); s++;
        cali_para_.measurement_rpy_para.p = mt_para(s); s++;
        cali_para_.measurement_rpy_para.y = mt_para(s); s++;
    }

    cali_para_.tool_para.x = mt_para(s); s++;
    cali_para_.tool_para.y = mt_para(s); s++;
    cali_para_.tool_para.z = mt_para(s); s++;
    return true;
}
