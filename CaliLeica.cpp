#include "CaliLeica.h"

CaliLeica::CaliLeica()
{
    // initiallize class member
    data_dim_ = 3;
    para_mt_num_ = 9;
}

CaliLeica::~CaliLeica()
{

}

bool CaliLeica::loadMeasuredData(const std::string&  datafile)
{
    std::ifstream afile;
    std::string filename = "mesurements.txt";
    //read measure line data
    std::string totalPathName = datafile + filename;

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
    int i = 0;

    while(getline(afile,sline))
    {
        if(i == 0)
        {
            try
            {
                count = stringtoNum<double>(sline);
            }
            catch(...)
            {
                cout<<"file format error";
                return false;
            }

            measure_data_length_ = (int)count;
            measure_data_.resize(measure_data_length_);
        }
        else if(i == measure_data_length_ + 1)
            break;
        else
        {
            j = 0;
            measure_data_[i-1].resize(data_dim_);
            std::stringstream output(sline);
            while(output>>ss)
            {
                if(j == data_dim_)
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
                    measure_data_[i-1][j] = count/MM2METER;
                    j++;
                }
            }
        }
        i++;
    }
    afile.close();

    return true;
}

//read input joint angle
bool CaliLeica::loadInputJointAngle(const std::string&  datafile)
{
    ifstream afile;
    std::string filename = "CAL.txt";
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
    int i = 0;
    while(getline(afile,sline))
    {
        i = 0;
        std::stringstream output(sline);
        while(output>>ss)
        {
            if(i == DOF)
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
                input_joint_angle_[j][i] = count * D2r;
                i++;
            }
        }
        j++;
    }
    afile.close();

    return true;
}

void CaliLeica::loadInputToolData(const std::string&  datafile)
{
    ;
}

void CaliLeica::getFrameParaJacobian(double& Se, RMatrix& Je, RVector& Fe, RVector& mt_para, RVector& measure_data_i, RVector& joint_i)
{
    // input DHPARA xyz rpy pt
    RMatrix Teye = RMatrix::eye(3);
    RMatrix Tbf(4), rz(3), rzry(3), R01(3);
    RVector Pbt(3), P0t(3);
    std::vector<RVector> J(para_mt_num_);
    RVector Pbm(3), rpy(3), Pft(3);
    for(int i = 0; i < 3; i++)
    {
        Pbm(i) = mt_para(i);
        rpy(i) = mt_para(i+3);
        Pft(i) = mt_para(i+6);
    }


    Tbf = rk->fKFlangeInBase(cali_para_.dh_para, joint_i);// in base

    //    mt_para.show("mt_para");

    //    joint_i.show("joint_i");

    Pbt = Tbf.subMatrix(0,2,0,2)*Pft + Tbf.subVector(0,2,3,COL);//in base

    //    rpy.show("rpy");
    rz = rk->RotZ(rpy(0));
    rzry = rz * rk->RotY(rpy(1));
    R01 = rzry * rk->RotX(rpy(2));
    P0t = R01 * Pbt + Pbm;

    //    Tbf.show("Tbf");

    //    Pbt.show("Pbt");
    //    R01.show("R01");

    int s = 0;
    //    rz.show("rz");
    //    rzry.show("rzry");

    J[s++] = Teye.subVector(0,2,0,COL);
    J[s++] = Teye.subVector(0,2,1,COL);
    J[s++] = Teye.subVector(0,2,2,COL);

    J[s++] = RVector::cross3(Teye.subVector(0,2,2,COL),R01*Pbt); //rz
    J[s++] = RVector::cross3(rz.subVector(0,2,1,COL),R01*Pbt); //ry
    J[s++] = RVector::cross3(rzry.subVector(0,2,0,COL),R01*Pbt); //rx

    J[s++] = R01 * Tbf.subVector(0,2,0,COL);
    J[s++] = R01 * Tbf.subVector(0,2,1,COL);
    J[s++] = R01 * Tbf.subVector(0,2,2,COL);

    for(int i = 0; i < data_dim_; i++)
    {
        for(int j = 0; j < para_mt_num_; j++)
        {
            Je(i,j) = J[j](i);
        }
        Fe(i) = P0t(i) - measure_data_i(i);
    }

    Se = Fe.dot(Fe);
    //        Je.show();
    //        Fe.show();
}

void CaliLeica::GetEleIdentifyMatrix(RMatrix& Phai_i, RVector &Fe_i, RVector& measure_data_i, RVector& joint_i)//all para include dh tool and measure
{

    RMatrix Teye = RMatrix::eye(3);
    RMatrix Tbase = RMatrix::eye(4);
    RMatrix T_w_0(4), T_f_t(4), rz(3), rzry(3);

    RMatrix Tadd(4), Rzyx(3);
    RVector P_f_t(3), P_w_0(3), rpy_m(3);
    std::vector<RMatrix> T_world(DOF+2), T_base(DOF+1);
    std::vector<RVector> Origen_world(DOF+2), x_world(DOF+2), y_world(DOF+2), z_world(DOF+2);
    std::vector<RVector> J;

    J.resize(para_total_num_);

    cali_para_.measurement_para.x = 3.783869;
    cali_para_.measurement_para.y = 1.88846;
    cali_para_.measurement_para.z = 0.06325;

    cali_para_.measurement_rpy_para.r = -1.1047;
    cali_para_.measurement_rpy_para.p = 0.0103;
    cali_para_.measurement_rpy_para.y = 0.0048;

    cali_para_.tool_para.x = -0.0031;
    cali_para_.tool_para.y = 0.0123;
    cali_para_.tool_para.z = 0.0265;



    P_w_0(0) = cali_para_.measurement_para.x;
    P_w_0(1) = cali_para_.measurement_para.y;
    P_w_0(2) = cali_para_.measurement_para.z;

    rpy_m(0) = cali_para_.measurement_rpy_para.r;
    rpy_m(1) = cali_para_.measurement_rpy_para.p;
    rpy_m(2) = cali_para_.measurement_rpy_para.y;

    P_f_t(0) = cali_para_.tool_para.x;
    P_f_t(1) = cali_para_.tool_para.y;
    P_f_t(2) = cali_para_.tool_para.z;

    rz = rk->RotZ(rpy_m(0));
    rzry = rz * rk->RotY(rpy_m(1));
    Rzyx = rzry * rk->RotX(rpy_m(2));

    T_w_0 = rk->RPToT(Rzyx, P_w_0);
    T_f_t = rk->RPToT(Teye, P_f_t);

    T_base[0] = Tbase;
    for(int i = 0; i < DOF; i++)
    {
        Tadd = rk->homogeneousTransfer(cali_para_.dh_para[0+4*i],cali_para_.dh_para[1+4*i], cali_para_.dh_para[2+4*i], cali_para_.dh_para[3+4*i] + joint_i(i));
        T_base[i+1] = T_base[i]*Tadd;
    }

    for(int i = 0; i < DOF + 1; i++)
    {
        T_world[i] = T_w_0*T_base[i];
    }

    T_world[DOF + 1] = T_world[DOF] * T_f_t;

    for(int i = 0; i < DOF + 2; i++)
    {
        Origen_world[i] = T_world[i].subVector(0,2,3,COL);
        x_world[i] = T_world[i].subVector(0,2,0,COL);
        y_world[i] = T_world[i].subVector(0,2,1,COL);
        z_world[i] = T_world[i].subVector(0,2,2,COL);
    }

    int k = para_mt_num_, s = 0;

    J[s++] = Teye.subVector(0,2,0,COL);
    J[s++] = Teye.subVector(0,2,1,COL);
    J[s++] = Teye.subVector(0,2,2,COL);

    J[s++] = RVector::cross3(Teye.subVector(0,2,2,COL),Origen_world[DOF + 1] - Origen_world[0]); //rz
    J[s++] = RVector::cross3(rz.subVector(0,2,1,COL),Origen_world[DOF + 1] - Origen_world[0]); //ry
    J[s++] = RVector::cross3(rzry.subVector(0,2,0,COL),Origen_world[DOF + 1] - Origen_world[0]); //rx

    J[s++] = x_world[DOF];
    J[s++] = y_world[DOF];
    J[s++] = z_world[DOF];

    for(int j = 0; j < DOF; j++)
    {
        if(check_flag_[k++])
            J[s++] = RVector::cross3(x_world[j],(Origen_world[DOF + 1] - Origen_world[j])); //alpha
        RVector aa = Origen_world[DOF + 1] - Origen_world[j];
        if(check_flag_[k++])
            J[s++] = x_world[j];//a
        if(check_flag_[k++])
            J[s++] = z_world[j+1];//d
        if(check_flag_[k++])
            J[s++] = RVector::cross3(z_world[j+1],(Origen_world[DOF + 1] - Origen_world[j+1]));//theta
        if(check_flag_[k++])
            J[s++] = RVector::cross3(y_world[j],(Origen_world[DOF + 1] - Origen_world[j])); //beta
    }

    for(int i = 0; i < data_dim_; i++)
    {
        for(int j = 0; j < para_total_num_; j++)
        {
            Phai_i(i,j) = J[j](i);
        }
        Fe_i(i) = Origen_world[DOF+1](i) - measure_data_i(i);
    }

//        Phai_i.show("Phai_i");
//        Fe_i.show("Fe_i");

}

bool CaliLeica::calError()
{
    RMatrix T0f(4), roty(3), Ty(4);
    RMatrix rz(3), rzry(3), R01(3);
    RVector P0t(3), Pbm(3), Pft(3), rpy(3);
    RVector Fe(measure_data_length_ * data_dim_);
    RMatrix Tadd(4);
    RVector measure_data_i(data_dim_), joint_i(DOF);

    Pbm(0) = cali_para_.measurement_para.x;
    Pbm(1) = cali_para_.measurement_para.y;
    Pbm(2) = cali_para_.measurement_para.z;

    rpy(0) = cali_para_.measurement_rpy_para.r;
    rpy(1) = cali_para_.measurement_rpy_para.p;
    rpy(2) = cali_para_.measurement_rpy_para.y;


    Pft(0) = cali_para_.tool_para.x;
    Pft(1) = cali_para_.tool_para.y;
    Pft(2) = cali_para_.tool_para.z;

    RVector py(3);

//    T0f = RMatrix::eye(4);

    rz = rk->RotZ(rpy(0));
    rzry = rz * rk->RotY(rpy(1));
    R01 = rzry * rk->RotX(rpy(2));

    R01.show("ro1");

    T0f.show("T0f");


    int s = 0;

    for(int i = 0; i < measure_data_length_; i++)
    {
        T0f = rk->RPToT(R01, Pbm);

        for(int j = 0; j < DOF; j++)
        {
            joint_i(j) = input_joint_angle_[i][j];

            roty =  rk->RotY(cali_para_.beta[j]);
            Ty = rk->RPToT(roty, py);

            Tadd = Ty * rk->homogeneousTransfer(cali_para_.dh_para[0+4*j], cali_para_.dh_para[1+4*j], cali_para_.dh_para[2+4*j], cali_para_.dh_para[3+4*j] + joint_i(j));

            T0f = T0f*Tadd;
        }

        P0t = T0f.subMatrix(0,2,0,2)*Pft + T0f.subVector(0,2,3,COL);

        P0t.show("P0t");


        for(int j = 0; j < data_dim_; j++)
        {
            measure_data_i(j) = measure_data_[i][j];
            Fe(s) = fabs(P0t(j) - measure_data_i(j));
            s++;
        }
    }

    Fe.show("Fe");

    double c_max = 0, c2, c_sum = 0;
    for(int i = 0; i < measure_data_length_ * data_dim_; i++)
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
    criter_after_.meanvalue = c_sum /(measure_data_length_ * data_dim_);
    criter_after_.rmsvalue = sqrt((Fe.dot(Fe)) / (measure_data_length_ * data_dim_));
double aa = c_sum;
double bb = Fe.dot(Fe);
    return true;
}

