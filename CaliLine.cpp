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

int CaliLine::loadMeasuredData(const std::string&  datafile)
{
    std::ifstream afile;
    std::string filename = "DynaCal1.msr";
    //read measure line data
    std::string totalPathName = datafile + filename;

    afile.open(totalPathName.data());

    if(!afile)
    {
//        cout<<"Open File Fail!"<<endl;
        return -1; // open file fail!
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
//            cout<<"file format error";
            return -2;
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

    return 1;
}

//read input joint angle
int CaliLine::loadInputJointAngle(const std::string&  datafile)
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
        return -1;
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
                    return -2;
                }
                input_joint_angle_[j][i-1] = count * D2r;
                i++;
            }
        }
        j++;
    }
    afile.close();

    return 1;
}

int CaliLine::loadInputToolData(const std::string&  datafile)
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
        return -1;
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
