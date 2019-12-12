#include "kinematicscalibration.h"

static inline std::string cp_str(CALIBRATE_PARA dpara)
{
    static std::string str[] = {"ALPHA1","A1","THETA1","D1","BETA1",
                                "ALPHA2","A2","THETA2","D2","BETA2",
                                "ALPHA3","A3","THETA3","D3","BETA3",
                                "ALPHA4","A4","THETA4","D4","BETA4",
                                "ALPHA5","A5","THETA5","D5","BETA5",
                                "ALPHA6","A6","THETA6","D6","BETA6",};
    return str[dpara];
}

KinematicsCalibration::KinematicsCalibration()
{
    // initiall all para: calitype;dhpara beta,chekflag;
    cali_line = new CaliLine();
    cali_leica = new CaliLeica();

    kc_cali_method_ = CALI_LINE;
    cali_type = cali_line;

    rk = new RobotKinematics();
    cali_type->calibration_beta_ = false;
}

KinematicsCalibration::~KinematicsCalibration()
{
    delete rk;
    delete cali_line;
    delete cali_leica;
}

void KinematicsCalibration::initiallDHPara()
{
    double a2, a3, d1, d2, d5, d6;
    rk->getRobotDHPara(a2, a3, d1, d2, d5, d6);
    // alpha a theta d beta
    double temp1[4*DOF] = {0      , 0 , d1, M_PI   ,
                           -M_PI/2, 0 , d2, -M_PI/2,
                           M_PI   , a2, 0 , 0      ,
                           M_PI   , a3, 0 , -M_PI/2,
                           -M_PI/2, 0 , d5, 0      ,
                           M_PI/2 , 0 , d6, 0      };

    memcpy(cali_type->cali_para_.dh_para,temp1,sizeof(temp1));
    for (int i = 0; i < DOF; i++)
    {
        cali_type->cali_para_.beta[i] = 0;
    }
}

void KinematicsCalibration::setRobotDHPara(ROBOT_TYPE type)
{
    rk->setRobotDHPara(type);
}

//?
void KinematicsCalibration::setCaliType(const std::string& type)
{
    if(type == "Line")
    {
        kc_cali_method_ = CALI_LINE;
    }
    else
    {
        kc_cali_method_ = CALI_LEICA;
    }
}

void KinematicsCalibration::getCaliType()
{
    cali_type->cali_method_ = kc_cali_method_;
}

void KinematicsCalibration::setCalibrationBeta(bool value)
{
    kc_cali_beta_ = value;
}

void KinematicsCalibration::getCalibrationBeta()
{
    cali_type->calibration_beta_ = kc_cali_beta_;
}

bool KinematicsCalibration::LoadData(const std::string& datafile)
{
    cali_type->loadMeasuredData(datafile);
    cali_type->loadInputJointAngle(datafile);
    //    cali_type->loadInputToolData(datafile);

    return true;
}

void KinematicsCalibration::setPara()
{
    // set calitype
    getCaliType();

    if(cali_type->cali_method_ == CALI_LINE)
    {
        cali_type = cali_line;
        cali_type->cali_method_ = CALI_LINE;
    }
    if(cali_type->cali_method_ == CALI_LEICA)
    {
        cali_type = cali_leica;
        cali_type->cali_method_ = CALI_LEICA;
    }

    // set calibeta;
    getCalibrationBeta();
    // set dhpara
    initiallDHPara();
    // ALL para num include beta
    cali_type->All_para_num_ = cali_type->para_mt_num_ + 5*DOF;

    // initiall check_flag_;
    for(int i = 0; i < cali_type->All_para_num_; i++)
        cali_type->check_flag_[i] = true;

    // set dh para num;
    if(cali_type->calibration_beta_)
    {
        cali_type->GN_ = 5;
    }
    else
    {
        for(int i = 0; i < DOF; i++)
            cali_type->check_flag_[cali_type->para_mt_num_ + 4 + 5*i] = false;
        cali_type->GN_ = 4;
    }

    // set all para num
    cali_type->para_total_num_ = cali_type->para_mt_num_ + DOF * cali_type->GN_;
}

bool KinematicsCalibration::calibration()
{
    // est frame para and update tool para
    RVector d_para;
    double error;
    cali_type->estParaOfFrame();// set tool para;
    cali_type->getIncrePara(d_para);
    d_allpara = cali_type->calAllDpara(d_para);
    calAllPara();
    d_allpara.show("d_allpara");

    updateMemberPara();

    bool flag = cali_type->calError();

    std::cout<<"error_last"<<error<<endl;

    return true;
}

void KinematicsCalibration::calAllPara()
{
    int i = 0;
    all_para_[i] = cali_type->cali_para_.measurement_para.x + d_allpara(i); i++;
    all_para_[i] = cali_type->cali_para_.measurement_para.y + d_allpara(i); i++;
    all_para_[i] = cali_type->cali_para_.measurement_para.z + d_allpara(i); i++;

    if(cali_type->cali_method_ == CALI_LEICA)
    {
        all_para_[i] = cali_type->cali_para_.measurement_rpy_para.r + d_allpara(i); i++;
        all_para_[i] = cali_type->cali_para_.measurement_rpy_para.p + d_allpara(i); i++;
        all_para_[i] = cali_type->cali_para_.measurement_rpy_para.y + d_allpara(i); i++;
    }

    all_para_[i] = cali_type->cali_para_.tool_para.x + d_allpara(i); i++;
    all_para_[i] = cali_type->cali_para_.tool_para.y + d_allpara(i); i++;
    all_para_[i] = cali_type->cali_para_.tool_para.z + d_allpara(i); i++;

    for(int j = 0; j < DOF; j++)
    {
        for(int k = 0; k < 5; k++)
        {
            if(k == 4)
            {
                all_para_[i] = cali_type->cali_para_.beta[j] + d_allpara(i);i++;
            }
            else
            {
                all_para_[i] = cali_type->cali_para_.dh_para[4*j+k] + d_allpara(i);i++;
            }
        }
    }
}

void KinematicsCalibration::getAllPara(double allpara[])
{
    for(int i = 0; i < cali_type->All_para_num_; i++)
    {
        allpara[i] = all_para_[i];
    }
}

void KinematicsCalibration::getAllDPara(double all_d_para[])
{
    for(int i = 0; i < cali_type->All_para_num_; i++)
        all_d_para[i] = d_allpara(i);
}

//output result
void KinematicsCalibration::outputClibrationDPara(std::string  path, std::string  data_file)
{
    ofstream afile;
    std::string filename = data_file + '_' + "dpara.txt";
    //    std::string totalPathName = path + filename;

    std::string totalPathName = path + data_file + '/' + filename;

    CALIBRATE_PARA dpara_name;
    std::string paraname;

    afile.open(totalPathName.data());
    if(!afile)
        cout<<"Open File Fail!"<<endl;
    std::string link_name;
    int showlenth = 15;
    int mt_num = 0;

    if(cali_type->cali_method_ == CALI_LINE)
        mt_num = 6;
    else
        mt_num = 9;

    afile<<setiosflags(ios::fixed)<<endl;
    afile<<setprecision(5)<<std::endl;


    afile<<left<<"D_PARA"<<"MM/DEGREE"<<std::endl;

    afile<<setw(5)<<right<<"link:"<<
           setw(showlenth)<<right<<"alpha"<<
           setw(showlenth)<<right<<"a"<<
           setw(showlenth)<<right<<"d"<<
           setw(showlenth)<<right<<"theta"<<
           setw(showlenth)<<right<<"beta"<<std::endl;

    for(int j = 0; j < DOF; j++)
    {
        link_name = "link" + std::to_string(j) + ':';
        afile<<setw(5)<<right<<link_name<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 0]*r2D<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 1]*METER2MM<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 2]*METER2MM<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 3]*r2D<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 4]*r2D<<std::endl;
    }
    afile<<std::endl;

    afile<<left<<"dhPara after calibration"<<"MM/DEGREE"<<std::endl;

    afile<<setw(5)<<right<<"link:"<<
           setw(showlenth)<<right<<"alpha"<<
           setw(showlenth)<<right<<"a"<<
           setw(showlenth)<<right<<"d"<<
           setw(showlenth)<<right<<"theta"<<
           setw(showlenth)<<right<<"beta"<<std::endl;

    for(int j = 0; j < DOF; j++)
    {
        link_name = "link" + std::to_string(j) + ':';
        afile<<setw(5)<<right<<link_name<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 0]*r2D<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 1]*METER2MM<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 2]*METER2MM<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 3]*r2D<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 4]*r2D<<std::endl;
    }

    afile<<left<<"MEASUREMENT AND TOOL PARA"<<"MM"<<std::endl;
    afile<<std::endl;


    afile<<setw(5)<<right<<"Mx:"<<setw(showlenth)<<right<<cali_type->cali_para_.measurement_para.x*METER2MM<<std::endl;
    afile<<setw(5)<<right<<"My:"<<setw(showlenth)<<right<<cali_type->cali_para_.measurement_para.y*METER2MM<<std::endl;
    afile<<setw(5)<<right<<"Mz:"<<setw(showlenth)<<right<<cali_type->cali_para_.measurement_para.z*METER2MM<<std::endl;

    afile<<setw(5)<<right<<"Rz:"<<setw(showlenth)<<right<<cali_type->cali_para_.measurement_rpy_para.r*r2D<<std::endl;
    afile<<setw(5)<<right<<"Ry:"<<setw(showlenth)<<right<<cali_type->cali_para_.measurement_rpy_para.p*r2D<<std::endl;
    afile<<setw(5)<<right<<"Rx:"<<setw(showlenth)<<right<<cali_type->cali_para_.measurement_rpy_para.y*r2D<<std::endl;

    afile<<setw(5)<<right<<"Tx:"<<setw(showlenth)<<right<<cali_type->cali_para_.tool_para.x*METER2MM<<std::endl;
    afile<<setw(5)<<right<<"Ty:"<<setw(showlenth)<<right<<cali_type->cali_para_.tool_para.y*METER2MM<<std::endl;
    afile<<setw(5)<<right<<"Tz:"<<setw(showlenth)<<right<<cali_type->cali_para_.tool_para.z*METER2MM<<std::endl;

    afile.close();
}

double KinematicsCalibration::getToolPara(std::string index)
{
    double ss;

    if(index == "Tx")
        ss = cali_type->cali_para_.tool_para.x;
    else if(index == "Ty")
        ss = cali_type->cali_para_.tool_para.y;
    else if(index == "Tz")
        ss = cali_type->cali_para_.tool_para.z;
    else if(index == "Mx")
        ss = cali_type->cali_para_.measurement_para.x;
    else if(index == "My")
        ss = cali_type->cali_para_.measurement_para.y;
    else if(index == "Mz")
        ss = cali_type->cali_para_.measurement_para.z;
    else if(index == "MRx")
        ss = cali_type->cali_para_.measurement_rpy_para.y;
    else if(index == "MRy")
        ss = cali_type->cali_para_.measurement_rpy_para.p;
    else if(index == "MRz")
        ss = cali_type->cali_para_.measurement_rpy_para.r;
    else
    {
        ; //error;
    }

    return ss;
}
//read input joint angle
void KinematicsCalibration::getCriter(double output_criter[])
{
    output_criter[0] = cali_type->criter_after_.maxvalue;
    output_criter[1] = cali_type->criter_after_.meanvalue;
    output_criter[2] = cali_type->criter_after_.rmsvalue;
}

void KinematicsCalibration::updateMemberPara()
{
    int i = 0;
    cali_type->cali_para_.measurement_para.x += d_allpara(i); i++;
    cali_type->cali_para_.measurement_para.y += d_allpara(i); i++;
    cali_type->cali_para_.measurement_para.z += d_allpara(i); i++;

    if(cali_type->cali_method_ == CALI_LEICA)
    {
        cali_type->cali_para_.measurement_rpy_para.r += d_allpara(i); i++;
        cali_type->cali_para_.measurement_rpy_para.p += d_allpara(i); i++;
        cali_type->cali_para_.measurement_rpy_para.y += d_allpara(i); i++;
    }
    else
    {
        cali_type->cali_para_.measurement_rpy_para.r = 0;
        cali_type->cali_para_.measurement_rpy_para.p = 0;
        cali_type->cali_para_.measurement_rpy_para.y = 0;
    }

    cali_type->cali_para_.tool_para.x += d_allpara(i); i++;
    cali_type->cali_para_.tool_para.y += d_allpara(i); i++;
    cali_type->cali_para_.tool_para.z += d_allpara(i); i++;

    for(int j = 0; j < DOF; j++)
    {
        for(int k = 0; k < 5; k++)
        {
            if(k == 4)
            {
                cali_type->cali_para_.beta[j] += d_allpara(i);i++;
            }
            else
            {
                cali_type->cali_para_.dh_para[4*j+k] += d_allpara(i);i++;
            }
        }
    }
}











