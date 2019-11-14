#include "kinematicscalibration.h"

template <class Type>
Type stringtoNum(const string& str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

static inline char * cp_str(CALIBRATE_PARA dpara)
{
    static char *strings[] = {"ALPHA1","A1","THETA1","D1","BETA1",
                              "ALPHA2","A2","THETA2","D2","BETA2",
                              "ALPHA3","A3","THETA3","D3","BETA3",
                              "ALPHA4","A4","THETA4","D4","BETA4",
                              "ALPHA5","A5","THETA5","D5","BETA5",
                              "ALPHA6","A6","THETA6","D6","BETA6",};
    return strings[dpara];
}

KinematicsCalibration::KinematicsCalibration()
{
    rk = new RobotKinematics();

    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
        check_flag_[i] = false;
    calibration_beta_ = false;
}

KinematicsCalibration::~KinematicsCalibration()
{
    delete rk;
}


void KinematicsCalibration::initiallDHPara()
{
    double a2, a3, d1, d2, d5, d6;
    rk->getRobotDHPara(a2, a3, d1, d2, d5, d6);
    // alpha a theta d beta
    double temp1[MAX_IDEN_PARA_NUM] = {0      ,0  ,M_PI   ,d1,0,
                                       -M_PI/2,0  ,-M_PI/2,d2,0,
                                       M_PI   ,a2 ,0      ,0 ,0,
                                       M_PI   ,a3 ,-M_PI/2,0 ,0,
                                       -M_PI/2,0  ,0      ,d5,0,
                                       M_PI/2 ,0  ,0      ,d6,0};

    memcpy(all_dh_para_,temp1,sizeof(temp1));
}

void KinematicsCalibration::setRobotDHPara(ROBOT_TYPE type)
{
    rk->setRobotDHPara(type);
}

void KinematicsCalibration::setCalibrationBeta(bool value)
{
    calibration_beta_ = value;
}

void KinematicsCalibration::loadMeasuredData(std::string  datafile)
{
    ifstream afile;
    std::string filename = "/DynaCal1.msr";
    //read measure line data
    std::string totalPathName = datafile + filename;

    afile.open(totalPathName.data());
    if(!afile)
        cout<<"Open File Fail!"<<endl;
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
            return ;
        }

        if(i == 0)
        {
            measure_data_length_ = (int)count;
            measure_line_data_.resize(measure_data_length_);
        }
        else if(i == measure_data_length_ + 1)
            break;
        else
            measure_line_data_[i-1] = count/MM2METER;
        i++;

    }
    afile.close();
}

//output result
void KinematicsCalibration::outputClibrationDPara(std::string  path)
{
    ofstream afile;
    std::string filename = "/dpara.txt";
    std::string totalPathName = path + filename;

    afile.open(totalPathName.data());
    if(!afile)
        cout<<"Open File Fail!"<<endl;

    for(int i = 0; i < 30; i++)
    {
        dpara = (CALIBRATE_PARA) i;

        std::string paraname = cp_str(dpara);
        afile<<paraname<<":"<<all_d_para_[i]<<std::endl;
    }

    afile<<"Tx"<<":"<<all_d_para_[30]<<std::endl;
    afile<<"Ty"<<":"<<all_d_para_[31]<<std::endl;
    afile<<"Tz"<<":"<<all_d_para_[32]<<std::endl;
    afile<<"Mx"<<":"<<all_d_para_[33]<<std::endl;
    afile<<"My"<<":"<<all_d_para_[34]<<std::endl;
    afile<<"Mz"<<":"<<all_d_para_[35]<<std::endl;

    afile.close();
}

//read input joint angle
void KinematicsCalibration::loadInputJointAngle(std::string  datafile)
{
    ifstream afile;
    std::string filename = "/point60.txt";
    std::string totalPathName = datafile + filename;

    input_joint_angle_.resize(measure_data_length_);
    for(int i = 0; i < measure_data_length_; i++)
        input_joint_angle_[i].resize(DOF);


    afile.open(totalPathName.data());
    if(!afile)
        cout<<"Open File Fail!"<<endl;
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
                    return ;
                }
                input_joint_angle_[j][i-1] = count * D2r;
                i++;
            }
        }
        j++;
    }
    afile.close();

}

void KinematicsCalibration::loadInputToolData(std::string  datafile)
{
    ifstream afile;
    std::string filename = "/our_ii.dyn";
    std::string totalPathName = datafile + filename;

    afile.open(totalPathName.data());
    if(!afile)
        cout<<"Open File Fail!"<<endl;
    std::string sline;
    std::string ss;

    double count = 0;
    double temp[2][3] = {0};

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

    tool_para_.x = temp[1][0]/MM2METER;
    tool_para_.y = temp[1][1]/MM2METER;
    tool_para_.z = temp[1][2]/MM2METER;

    measurement_para_.x = temp[0][0]/MM2METER;
    measurement_para_.y = temp[0][1]/MM2METER;
    measurement_para_.z = temp[0][2]/MM2METER;

    afile.close();
}

double KinematicsCalibration::getToolPara(std::string index)
{
    double ss;

    if(index == "Tx")
        ss = tool_para_.x;
    else if(index == "Ty")
        ss = tool_para_.y;
    else if(index == "Tz")
        ss = tool_para_.z;
    else if(index == "Mx")
        ss = measurement_para_.x;
    else if(index == "My")
        ss = measurement_para_.y;
    else if(index == "Mz")
        ss = measurement_para_.z;
    else
        ; //error;

}

void KinematicsCalibration::getAllPara(double allpara[])
{
    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
    {
        allpara[i] = all_para_[i];
    }
}

void KinematicsCalibration::getAllDPara(double all_d_para[])
{
    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
    {
        all_d_para[i] = all_para_[i] - all_dh_para_[i];
    }
}

void KinematicsCalibration::setCheckFlag(int index, bool value)
{
    check_flag_[index] = value;
}

void KinematicsCalibration::GetEleIdentifyMatrix(double allPara[], int index, RVector& Phai_i, double& E_i)//all para include dh tool and measure
{
    RMatrix T_eye(3);
    RVector Pt_m(3), Pm(3);
    std::vector<RMatrix> T_base(DOF+2);
    std::vector<RVector> Origen_base(DOF+2), x_base(DOF+2), y_base(DOF+2), z_base(DOF+2);
    std::vector<RVector> J;

    J.resize(calibration_para_num_);

    double line_i = measure_line_data_[index];

    Pm.value[0] = allPara[33];
    Pm.value[1] = allPara[34];
    Pm.value[2] = allPara[35];

    T_base = rk->GetAllTransMatrixtobase(allPara, calibration_beta_);

    for(int i = 0; i < DOF + 2; i++)
    {
        Origen_base[i] = RMatrix::subRVector(T_base[i],0,2,3,"Column");
        x_base[i] = RMatrix::subRVector(T_base[i],0,2,0,"Column");
        y_base[i] = RMatrix::subRVector(T_base[i],0,2,1,"Column");
        z_base[i] = RMatrix::subRVector(T_base[i],0,2,2,"Column");
    }

    int k = 0, s = 0;
    for(int j = 0; j < DOF; j++)
    {
        if(check_flag_[k++])
            J[s++] = RVector::cross3(x_base[j],(Origen_base[DOF + 1] - Origen_base[j])); //alpha
        if(check_flag_[k++])
            J[s++] = x_base[j];//a
        if(check_flag_[k++])
            J[s++] = RVector::cross3(z_base[j+1],(Origen_base[DOF + 1] - Origen_base[j+1]));//theta
        if(check_flag_[k++])
            J[s++] = z_base[j+1];//d
        if(check_flag_[k++])
            J[s++] = RVector::cross3(y_base[j],(Origen_base[DOF + 1] - Origen_base[j])); //beta
    }

    J[s++] = x_base[DOF];
    J[s++] = y_base[DOF];
    J[s++] = z_base[DOF];

    J[s++] = - RMatrix::subRVector(T_eye,0,2,0,"Column");
    J[s++] = - RMatrix::subRVector(T_eye,0,2,1,"Column");
    J[s++] = - RMatrix::subRVector(T_eye,0,2,2,"Column");


    Pt_m = Origen_base[DOF + 1] - Pm;


//    double N_l = Pt_m.norm(Pt_m);
//    double d_i = line_i - Pt_m;

    for(int i = 0; i < 36; i++)
    {
        Phai_i.value[i] = 0;
        for(int j = 0; j < 3; j++)
            Phai_i.value[i] =  2 * Pt_m.value[j] * J[i].value[j] + Phai_i.value[i];
    }

    E_i = line_i * line_i - (Pt_m.value[0]*Pt_m.value[0] + Pt_m.value[1]*Pt_m.value[1] + Pt_m.value[2]*Pt_m.value[2]);
}

int KinematicsCalibration::calibrationDHParaNum()
{
    int count = 0;
    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
        count += (int)check_flag_[i];
    int calibration_dh_para_num = count;

    return calibration_dh_para_num;
}

int KinematicsCalibration::calibrationParaNum(int calibration_dh_para_num)
{
    int calibration_para_num = calibration_dh_para_num + 6;//dhpara + tool + measure;
    return calibration_para_num;
}

void KinematicsCalibration::GetIdentifyMatrix(RMatrix& Phai_m, RVector& Line_v, double& Eerror)
{
    //    RMatrix Phai_m(measure_data_length_,calibration_para_num_);
    //    RVector Line_v(measure_data_length_);
    RVector Phai_i(measure_data_length_);
    double E_i = 0;

    double allpara_i[36] = {0};

    memcpy(allpara_i,all_para_,sizeof(all_para_));

    for(int i = 0; i < measure_data_length_; i++)
    {

        for(int j = 0; j < DOF; j++)
            allpara_i[5*j + 2] = all_para_[5*j + 2] + input_joint_angle_[i][j];//add joint angle :THETA

        GetEleIdentifyMatrix(allpara_i, i, Phai_i, E_i);

        for(int k = 0; k < calibration_para_num_; k++)
        {
            Phai_m.value[i][k] = Phai_i.value[k];
        }
        Line_v.value[i] = E_i;

    }

    Eerror = RVector::norm(Line_v);
}

void KinematicsCalibration::getIncrePara(RVector& d_para, double& Eerror_last)
{
    RMatrix Phai_m(measure_data_length_,calibration_para_num_);
    RVector Line_v(measure_data_length_);

    RMatrix Phai_T(calibration_para_num_, measure_data_length_);
    RMatrix Eye(Phai_m.iCol);

    GetIdentifyMatrix(Phai_m, Line_v, Eerror_last);
    Phai_T = RMatrix::RTranspose(Phai_m);

    //    svdSim(Phai,RMatrix &U,RMatrix &S,RMatrix &V);
    int mu = 0;
    RMatrix PTP_1(calibration_para_num_, calibration_para_num_);
    RMatrix temp1(calibration_para_num_, measure_data_length_);

    bool inv = RMatrix::RMatrixInv(Phai_T*Phai_m, PTP_1, measure_data_length_);
    if(inv)
    {
        temp1 = PTP_1*Phai_T;
        d_para = temp1*Line_v;
    }
    else
    {
        ;//error;
    }
}

void KinematicsCalibration::updataAllPara(const RVector& d_para)
{
    //update dhpara
    int j = 0;
    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
    {
        if(check_flag_[i])
        {
            all_para_[i] += d_para.value[j];
            j++;
        }
    }
    //updata tool and measure data
    all_para_[30] += d_para.value[calibration_para_num_-6];
    all_para_[31] += d_para.value[calibration_para_num_-5];
    all_para_[32] += d_para.value[calibration_para_num_-4];
    all_para_[33] += d_para.value[calibration_para_num_-3];
    all_para_[34] += d_para.value[calibration_para_num_-2];
    all_para_[35] += d_para.value[calibration_para_num_-1];
}


bool KinematicsCalibration::calibration()
{
    //initiallize class member variables
    calibration_dh_para_num_ = calibrationDHParaNum();
    calibration_para_num_ = calibrationParaNum(calibration_dh_para_num_);

    //initiall input ALLPARA
    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
    {
        all_para_[i] = all_dh_para_[i];
    }
    all_para_[30] = tool_para_.x;
    all_para_[31] = tool_para_.y;
    all_para_[32] = tool_para_.z;

    all_para_[33] = measurement_para_.x;
    all_para_[34] = measurement_para_.y;
    all_para_[35] = measurement_para_.z;

    //calculate last error
    RVector d_para(calibration_para_num_);

    double error_old = 1;
    double error = 1;
    int loop = 0;
    while((error > 0.001))
    {
        loop++;
        getIncrePara(d_para, error);
        if((fabs(error_old - error) < 1e-5) || (loop > 100))
            break;
        error_old = error;
        // update para
        updataAllPara(d_para);
    }
    std::cout<<"error_last"<<error_old<<endl;
    if(loop > 100)
        return false;
    else
        return true;
}

void KinematicsCalibration::showdpara()
{
    for(int i = 0; i < 30; i++)
    {
        if((i%5 == 1)||(i%5 == 3))
        {
            all_d_para_[i] = (all_para_[i] - all_dh_para_[i]) * MM2METER; //in mm
        }
        else
        {
            all_d_para_[i] = (all_para_[i] - all_dh_para_[i])*r2D;// in degree
        }
    }
    all_d_para_[30] = (all_para_[30] - tool_para_.x) * MM2METER;
    all_d_para_[31] = (all_para_[31] - tool_para_.y) * MM2METER;
    all_d_para_[32] = (all_para_[32] - tool_para_.z) * MM2METER;

    all_d_para_[33] = (all_para_[33] - measurement_para_.x) * MM2METER;
    all_d_para_[34] = (all_para_[34] - measurement_para_.y) * MM2METER;
    all_d_para_[35] = (all_para_[35] - measurement_para_.z) * MM2METER;
}
