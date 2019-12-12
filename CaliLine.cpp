#include "CaliLine.h"

CaliLine::CaliLine()
{
    // initiallize class member
    data_dim_ = 1;
    para_mt_num_ = 6;
}

CaliLine::~CaliLine()
{

}

bool CaliLine::loadMeasuredData(const std::string&  datafile)
{
    std::ifstream afile;
    std::string filename = "DynaCal1.msr";
    //read measure line data
    std::string totalPathName = datafile + filename;

    afile.open(totalPathName.data());
    if(!afile)
    {
        cout<<"Open File Fail!"<<endl;
        return false;
    }
    std::string s;
    double count = 0;
    int i = 0;
    while(getline(afile,s))
    {
        try
        {
            count = stringtoNum<double>(s);
        }
        catch(...)
        {
            cout<<"file format error";
            return false;
        }

        if(i == 0)
        {
            measure_data_length_ = (int)count;
            measure_data_.resize(measure_data_length_);
        }
        else if(i == measure_data_length_ + 1)
            break;
        else
        {
            measure_data_[i-1].resize(data_dim_);
            measure_data_[i-1][0] = count/MM2METER;
        }
        i++;
    }
    afile.close();

    return true;
}

//read input joint angle
bool CaliLine::loadInputJointAngle(const std::string&  datafile)
{
    ifstream afile;
    std::string filename = "point60.txt";
    std::string totalPathName = datafile + filename;

    input_joint_angle_.resize(measure_data_length_);
    for(int i = 0; i < measure_data_length_; i++)
        input_joint_angle_[i].resize(DOF);

    afile.open(totalPathName.data());
    if(!afile)
    {
        cout<<"Open File Fail!"<<endl;
        return false;
    }
    std::string sline;
    std::string ss;

    double count = 0;
    int j = 0;
    while(getline(afile,sline))
    {
        int i = 0;
        std::stringstream output(sline);
        while(output>>ss)
        {
            if(i == 0)
            {
                i++;
                continue;
            }
            else if(i == DOF+1)
                break;
            else
            {
                try
                {
                    count = stringtoNum<double>(ss);
                }
                catch(...)
                {
                    cout<<"file format error";
                    return false;
                }
                input_joint_angle_[j][i-1] = count * D2r;
                i++;
            }
        }
        j++;
    }
    afile.close();

    return true;
}

void CaliLine::loadInputToolData(const std::string&  datafile)
{
    ifstream afile;
    std::string filename = "our_ii.dyn";
    std::string totalPathName = datafile + filename;

    std::string sline;
    std::string ss;

    double count = 0;
    double temp[2][3] = {0};

    afile.open(totalPathName.data());
    if(!afile)
    {
        cout<<"Open File Fail!"<<endl;
        return ;
    }

    for(int i = 0; i < 2; i++)
    {
        getline(afile,sline);
        std::stringstream output(sline);
        for(int j = 0; j < POSITION_DOF+1; j++)
        {
            output>>ss;
            count = stringtoNum<double>(ss);

            if(j == 0)
                continue;
            temp[i][j-1] = count;
        }
    }

    afile.close();

    cali_para_.tool_para.x = temp[1][0]/MM2METER;
    cali_para_.tool_para.y = temp[1][1]/MM2METER;
    cali_para_.tool_para.z = temp[1][2]/MM2METER;

    cali_para_.measurement_para.x = temp[0][0]/MM2METER;
    cali_para_.measurement_para.y = temp[0][1]/MM2METER;
    cali_para_.measurement_para.z = temp[0][2]/MM2METER;
}

//bool CaliLine::estParaOfFrame()
//{
//    // input : DHPARA
//    // OUTPUT : X Y Z r p y
////    getFrameParaJacobian();
//    double c1, c2, lamda, rho, sigma, a, b;
//    int groupSz, iters, loops, data_index, s;

//    c1 = 0.1;
//    c2 = 0.7;
//    lamda = 1.0;
//    groupSz = 11;
//    iters = 200;
//    data_index = -1;

//    RVector Pbm_c(3), Pft_c(3), Pbm(3), Pft(3);
//    Pbm.setZero();
//    Pft.setZero();

//    double Se_i, Se, Se_plus;
//    RMatrix Je(groupSz,para_mt_num_), hS, hS_inv(para_mt_num_,para_mt_num_), Je_i(data_dim_,para_mt_num_);

//    RVector gS(para_mt_num_), gS_plus(para_mt_num_), dk;
//    RVector Fe_i(data_dim_), Fe(groupSz * data_dim_);
//    RVector measure_data_i(data_dim_), joint_i(DOF);
//    RVector mt_para(9);
//    while(iters--)
//    {
//        for(int i = 0; i < measure_data_length_; i++)
//        {
//            if(i == measure_data_length_-1)
//                bool sss = 0;

//            data_index = data_index +1;

//            if(data_index >= measure_data_length_)
//                data_index = data_index - measure_data_length_;

//           if((data_index + 1)%groupSz == 1)
//           {
//               Je.clear();
//               Fe.setZero();
//               Se = 0;
//               s = 0;
//           }

//           measure_data_i(0) = measure_data_[data_index][0];
//           for(int k = 0; k < 6; k++)
//           {
//               joint_i(k) = input_joint_angle_[data_index][k];
//           }

//           for(int kk = 0; kk < 3; kk++)
//           {
//               mt_para(kk) = Pbm(kk);
//               mt_para(kk+3) = 0;
//               mt_para(kk+6) = Pft(kk);
//           }
//           getFrameParaJacobian(Se_i, Je_i, Fe_i, mt_para, measure_data_i, joint_i);

//           Se = Se + Se_i;
//           for(int p = 0; p < para_mt_num_; p++)
//               Je(s,p) = Je_i(0,p);
//           Fe(s) = Fe_i(0);
//           s++;

//           if((data_index + 1) % groupSz == 0)
//           {
////               Je.show();
////               Fe.show();

//               hS = Je.transpose()*Je*2;
//               gS = Je.transpose()*Fe*2;

////               hS.show("HS");
////               gS.show("GS");
//               bool flag = hS.inverse(hS_inv);
//               if(flag)
//                   dk = (hS_inv * gS)*(-1);
//               else
//                   return false;

////               dk.show("dk");

//               rho = 0.1;
//               sigma = 0.7;

//               srand(time(NULL));
//               lamda = (rand() % 9) * 0.1 + 0.1;

////               lamda = 0.3;
//               a = 0.0;
//               b = INFINITY;
//               loops = 0;
//               while(1)
//               {
//                   loops = loops + 1;
//                   Je.clear();
//                   Fe.setZero();
//                   Se_plus = 0;
//                   s = 0;

//                   for(int ii = data_index + 1 - groupSz; ii < data_index+1; ii++)
//                   {
//                       measure_data_i(0) = measure_data_[ii][0];
//                       for(int k = 0; k < 6; k++)
//                       {
//                           joint_i(k) = input_joint_angle_[ii][k];
//                       }

//                       for(int r = 0; r < 3; r++)
//                       {
//                           Pbm_c(r) = Pbm(r) + lamda*dk(r);
//                           Pft_c(r) = Pft(r) + lamda*dk(r+3);
//                       }

////                       Pbm_c.show("Pbm_c");
////                       Pft_c.show("Pft_c");
//                       for(int kk = 0; kk < 3; kk++)
//                       {
//                           mt_para(kk) = Pbm_c(kk);
//                           mt_para(kk+3) = 0;
//                           mt_para(kk+6) = Pft_c(kk);
//                       }

//                       getFrameParaJacobian(Se_i, Je_i, Fe_i, mt_para,  measure_data_i, joint_i);

//                       Se_plus = Se_plus + Se_i;

//                       for(int p = 0; p < para_mt_num_; p++)
//                           Je(s,p) = Je_i(0,p);
//                       Fe(s) = Fe_i(0);
//                       s++;
//                   }

//                   gS_plus = Je.transpose()*Fe*2;

////                   double aa = gS.dot(dk);
////                   double bb = gS_plus.dot(dk);
////                   double cc = Se + gS.dot(dk) * rho*lamda;

//                   if(!(Se_plus <= Se + gS.dot(dk) * rho*lamda))
//                   {
//                       b = lamda;
//                       lamda = (lamda + a)/2;
//                       continue;
//                   }
//                   if(!(gS_plus.dot(dk) >= gS.dot(dk)*sigma))
//                   {
//                       a = lamda;
//                       lamda = std::min(2*lamda,(b + lamda)/2);
//                       continue;
//                   }
//                   break;
//               }
//               dk = dk*lamda;
////               dk.show("dk2");

//               for(int r = 0; r < 3; r++)
//               {
//                   Pbm(r) = Pbm(r) + dk(r);
//                   Pft(r) = Pft(r) + dk(r+3);
//               }
////               Pbm.show("Pbm");
////               Pft.show("Pft");

//           }
//        }
//    }

//    cali_para_.measurement_para.x = Pbm(0);
//    cali_para_.measurement_para.y = Pbm(1);
//    cali_para_.measurement_para.z = Pbm(2);

//    cali_para_.tool_para.x = Pft(0);
//    cali_para_.tool_para.y = Pft(1);
//    cali_para_.tool_para.z = Pft(2);
//    return true;
//}

void CaliLine::getFrameParaJacobian(double& Se, RMatrix& Je, RVector& Fe, RVector& mt_para, RVector& measure_data_i, RVector& joint_i)
{
    // input DHPARA xyz rpy pt
    RMatrix Teye = RMatrix::eye(3);
    RMatrix Tbf(4);
    RVector Pbt(3), Pmt(3), Pbm(3), Pft(3);
    std::vector<RVector> J(para_mt_num_);
    for(int i = 0; i < 3; i++)
    {
        Pbm(i) = mt_para(i);
        Pft(i) = mt_para(i+3);
    }

    Tbf = rk->fKFlangeInBase(cali_para_.dh_para, joint_i);
    Pbt = Tbf.subMatrix(0,2,0,2)*Pft + Tbf.subVector(0,2,3,COL);

    Pmt = Pbt - Pbm;
//    Tbf.show();
//    Pbt.show();
//    Pmt.show();

    int s = 0;

    J[s++] = - Teye.subVector(0,2,0,COL);
    J[s++] = - Teye.subVector(0,2,1,COL);
    J[s++] = - Teye.subVector(0,2,2,COL);

    J[s++] = Tbf.subVector(0,2,0,COL);
    J[s++] = Tbf.subVector(0,2,1,COL);
    J[s++] = Tbf.subVector(0,2,2,COL);

    for(int i = 0; i < para_mt_num_; i++)
    {
        Je(0,i) = 0;
        for(int j = 0; j < 3; j++)
            Je(0,i) = Pmt(j) * J[i](j) * 2 + Je(0,i);
    }
    Fe(0) = Pmt.dot(Pmt) - measure_data_i.dot(measure_data_i);
    Se = Fe.dot(Fe);
//    Je.show();
//    Fe.show();
}

void CaliLine::GetEleIdentifyMatrix(RMatrix& Phai_i, RVector &Fe_i, RVector& measure_data_i, RVector& joint_i)//all para include dh tool and measure
{

    RMatrix T_eye = RMatrix::eye(3);
    RMatrix Tbase = RMatrix::eye(4);
    RMatrix Ttool_f, Tadd(4);
    RVector Pt_m(3), p_m_base(3), p_tool_f(3);
    std::vector<RMatrix> T_world(DOF+2), T_base(DOF+1);
    std::vector<RVector> Origen_world(DOF+2), x_world(DOF+2), y_world(DOF+2), z_world(DOF+2);
    std::vector<RVector> J;
    J.resize(para_total_num_);

    RMatrix T_base_world = Tbase;


//    cali_para_.measurement_para.x = -0.0411;
//    cali_para_.measurement_para.y = -1.1728;
//    cali_para_.measurement_para.z = 0.0116;

//    cali_para_.tool_para.x = 0.0010;
//    cali_para_.tool_para.y = -0.1034;
//    cali_para_.tool_para.z = 0.0949;


    p_m_base(0) = cali_para_.measurement_para.x;
    p_m_base(1) = cali_para_.measurement_para.y;
    p_m_base(2) = cali_para_.measurement_para.z;

    p_tool_f(0) = cali_para_.tool_para.x;
    p_tool_f(1) = cali_para_.tool_para.y;
    p_tool_f(2) = cali_para_.tool_para.z;


    T_base[0] = Tbase;
    for(int i = 0; i < DOF; i++)
    {
        Tadd = rk->homogeneousTransfer(cali_para_.dh_para[0+4*i],cali_para_.dh_para[1+4*i], cali_para_.dh_para[2+4*i], cali_para_.dh_para[3+4*i] + joint_i(i));
        T_base[i+1] = T_base[i]*Tadd;
    }

    Ttool_f = rk->RPToT(T_eye, p_tool_f);
//    Ttool_f.show("Ttool_f");

    for(int i = 0; i < DOF + 1; i++)
    {
        T_world[i] = T_base_world*T_base[i];
    }

    T_world[DOF + 1] = T_world[DOF] * Ttool_f;

    for(int i = 0; i < DOF + 2; i++)
    {
//        T_world[i].show("ss");
        Origen_world[i] = T_world[i].subVector(0,2,3,COL);
        x_world[i] = T_world[i].subVector(0,2,0,COL);
        y_world[i] = T_world[i].subVector(0,2,1,COL);
        z_world[i] = T_world[i].subVector(0,2,2,COL);
    }

    Pt_m = Origen_world[DOF + 1] - p_m_base;

    int k = 6, s = 0;

    J[s++] = -T_eye.subVector(0,2,0,COL);
    J[s++] = -T_eye.subVector(0,2,1,COL);
    J[s++] = -T_eye.subVector(0,2,2,COL);

    J[s++] = x_world[DOF];
    J[s++] = y_world[DOF];
    J[s++] = z_world[DOF];

    for(int j = 0; j < DOF; j++)
    {
        if(check_flag_[k++])
            J[s++] = RVector::cross3(x_world[j],(Origen_world[DOF + 1] - Origen_world[j])); //alpha
        if(check_flag_[k++])
            J[s++] = x_world[j];//a
        if(check_flag_[k++])
            J[s++] = z_world[j+1];//d
        if(check_flag_[k++])
            J[s++] = RVector::cross3(z_world[j+1],(Origen_world[DOF + 1] - Origen_world[j+1]));//theta
        if(check_flag_[k++])
            J[s++] = RVector::cross3(y_world[j],(Origen_world[DOF + 1] - Origen_world[j])); //beta
    }


    for(int i = 0; i < para_total_num_; i++)
    {
        Phai_i(0,i) = 0;
        for(int j = 0; j < POSITION_DOF; j++)
            Phai_i(0,i) =  2 * Pt_m(j) * J[i](j) + Phai_i(0,i);
    }

    Fe_i(0) = Pt_m.dot(Pt_m) - measure_data_i.dot(measure_data_i);
}

bool CaliLine::calError()
{
    RMatrix Tbf, roty(3), Ty(4);
    RVector Pbt(3), Pmt(3), Pbm(3), Pft(3);
    RVector Fe(measure_data_length_ * data_dim_);
    RMatrix Tadd(4);
    RVector measure_data_i(data_dim_), joint_i(DOF);

    Pbm(0) = cali_para_.measurement_para.x;
    Pbm(1) = cali_para_.measurement_para.y;
    Pbm(2) = cali_para_.measurement_para.z;

    Pft(0) = cali_para_.tool_para.x;
    Pft(1) = cali_para_.tool_para.y;
    Pft(2) = cali_para_.tool_para.z;

    RVector py(3);

    for(int i = 0; i < measure_data_length_; i++)
    {
        measure_data_i(0) = measure_data_[i][0];

        Tbf = RMatrix::eye(4);

        for(int j = 0; j < DOF; j++)
        {
            joint_i(j) = input_joint_angle_[i][j];

            roty =  rk->RotY(cali_para_.beta[j]);
            Ty = rk->RPToT(roty, py);

            Tadd = Ty * rk->homogeneousTransfer(cali_para_.dh_para[0+4*j], cali_para_.dh_para[1+4*j], cali_para_.dh_para[2+4*j], cali_para_.dh_para[3+4*j] + joint_i(j));

            Tbf = Tbf*Tadd;
        }

        Pbt = Tbf.subMatrix(0,2,0,2)*Pft + Tbf.subVector(0,2,3,COL);
        Pmt = Pbt - Pbm;

        Fe(i) = fabs(Pmt.norm() - measure_data_i(0));
    }

    double c_max = 0, c2, c_sum = 0;
    for(int i = 0; i < measure_data_length_; i++)
    {
        c2 = Fe(i);
        if(c_max < c2)
        {
            c_max = c2;
        }

        c_sum = c_sum + Fe(i);
    }
    if(c_max == 0)
        return false;

    criter_after_.maxvalue = c_max;
    criter_after_.meanvalue = c_sum / measure_data_length_;
    criter_after_.rmsvalue = sqrt((Fe.dot(Fe)) / measure_data_length_);

    return true;
}
