#include "kinematics_calibration.h"
#include <iomanip>

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
    cali_method_ = CALI_LINE;
    data_dim_ = 1;
    para_mt_num_ = 6;


    kc_cali_beta_ = false;
    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
    {
        kc_choose_para_[i] = 0;
    }

    aral_interface_ = new RLIntface("aubo_i5", /*Creat_Share_Memory*/0 | LOG_DEBUG);
    dof_ = 6;

}

KinematicsCalibration::~KinematicsCalibration()
{

}

//?
void KinematicsCalibration::setCaliType(const std::string& type)
{
    if(type == "Line")
    {
        cali_method_ = CALI_LINE;
        data_dim_ = 1;
        para_mt_num_ = 6;
    }
    else
    {
        cali_method_ = CALI_POS;
        data_dim_ = 3;
        para_mt_num_ = 9;
    }
}

unsigned int KinematicsCalibration::getCaliMethod()
{
    return cali_method_;
}

void KinematicsCalibration::setCalibrationBeta(bool value)
{
    kc_cali_beta_ = value;
}

bool KinematicsCalibration::getCalibrationBeta()
{
    return kc_cali_beta_;
}

bool KinematicsCalibration::LoadData(const std::string& datafile)
{
    int flag1, flag2;
    flag1 = loadMeasuredData(datafile);
    flag2 = loadInputJointAngle(datafile);
    //    loadInputToolData(datafile);

    if((flag1 + flag2) < 2)
        return false;
    return true;
}

void KinematicsCalibration::setPara()
{
    // set calitype
    getCaliMethod();

    //    if(cali_method_ == CALI_LINE)
    //    {
    //        cali_type = cali_line;
    //        cali_method_ = CALI_LINE;
    //    }
    //    if(cali_method_ == CALI_LEICA)
    //    {
    //        cali_type = cali_leica;
    //        cali_method_ = CALI_LEICA;
    //    }


    // set calibeta;
    getCalibrationBeta();
    // ALL para num include beta
    All_para_num_ = para_mt_num_ + 5 * dof_;

    // initiall check_flag_;
    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
    {
        if(i < para_mt_num_)
            check_flag_[i] = true;
        else
            check_flag_[i] = kc_choose_para_[i - para_mt_num_];
    }

    // set dh para num;
    if(calibration_beta_)
    {
        GN_ = 5;
    }
    else
    {
        //        for(int i = 0; i < DOF; i++)
        //            check_flag_[para_mt_num_ + 4 + 5*i] = false;
        GN_ = 4;
    }

    // set all para num
    para_total_num_ = para_mt_num_ + dof_ * GN_;

    getCalibrationNum();
}

bool KinematicsCalibration::calibration()
{
    // est frame para and update tool para

    int res = aral_interface_->calibRobotKinematicsPara(measure_data_, input_joint_angle_,
                               check_flag_, calibration_beta_, cali_method_, cali_result_);

    memcpy(d_allpara, cali_result_.all_d_para_, sizeof(double)*MAX_IDEN_PARA_NUM);
    calAllPara();
//    d_allpara.show("d_allpara");
    updateMemberPara();

    return true;
}

void KinematicsCalibration::calAllPara()
{
    int i = 0;
    all_para_[i] = cali_para_.measurement_para.x + d_allpara[i]; i++;
    all_para_[i] = cali_para_.measurement_para.y + d_allpara[i]; i++;
    all_para_[i] = cali_para_.measurement_para.z + d_allpara[i]; i++;

    if(cali_method_ == CALI_POS)
    {
        all_para_[i] = cali_para_.measurement_rpy_para.r + d_allpara[i]; i++;
        all_para_[i] = cali_para_.measurement_rpy_para.p + d_allpara[i]; i++;
        all_para_[i] = cali_para_.measurement_rpy_para.y + d_allpara[i]; i++;
    }

    all_para_[i] = cali_para_.tool_para.x + d_allpara[i]; i++;
    all_para_[i] = cali_para_.tool_para.y + d_allpara[i]; i++;
    all_para_[i] = cali_para_.tool_para.z + d_allpara[i]; i++;

    for(int j = 0; j < dof_; j++)
    {
        for(int k = 0; k < 5; k++)
        {
            if(k == 4)
            {
                all_para_[i] = cali_para_.beta[j] + d_allpara[i];i++;
            }
            else
            {
                all_para_[i] = cali_para_.dh_para[4*j+k] + d_allpara[i];i++;
            }
        }
    }
}

void KinematicsCalibration::getAllPara(double allpara[])
{
    for(int i = 0; i < All_para_num_; i++)
    {
        allpara[i] = all_para_[i];
    }
}

void KinematicsCalibration::getAllDPara(double all_d_para[])
{
    for(int i = 0; i < All_para_num_; i++)
        all_d_para[i] = d_allpara[i];
}

//output result
void KinematicsCalibration::outputClibrationDPara(std::string  path, std::string  data_file)
{
    ofstream afile;
    std::string filename = data_file + '_' + "dpara.txt";
    //    std::string totalPathName = path + filename;

    std::string totalPathName = path + data_file + '/' + filename;

    afile.open(totalPathName.data());
    if(!afile)
        cout<<"Open File Fail!"<<endl;
    std::string link_name;
    int showlenth = 15;
    int mt_num = 0;

    if(cali_method_ == CALI_LINE)
        mt_num = 6;
    else
        mt_num = 9;

    afile<<setiosflags(ios::fixed)<<endl;
    afile<<setprecision(5)<<std::endl;


    afile<<left<<"D_PARA "<<"MM/DEGREE"<<std::endl;

    afile<<setw(5)<<right<<"link:"<<
           setw(showlenth)<<right<<"alpha"<<
           setw(showlenth)<<right<<"a"<<
           setw(showlenth)<<right<<"d"<<
           setw(showlenth)<<right<<"theta"<<
           setw(showlenth)<<right<<"beta"<<std::endl;

    for(int j = 0; j < dof_; j++)
    {
        link_name = "link" + std::to_string(j) + ':';
        afile<<setw(5)<<right<<link_name<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 0]*r2D<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 1]*1000<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 2]*1000<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 3]*r2D<<
               setw(showlenth)<<right<<d_allpara[mt_num + 5*j + 4]*r2D<<std::endl;
    }
    afile<<std::endl;

    afile<<left<<"dhPara after calibration "<<"MM/DEGREE"<<std::endl;

    afile<<setw(5)<<right<<"link:"<<
           setw(showlenth)<<right<<"alpha"<<
           setw(showlenth)<<right<<"a"<<
           setw(showlenth)<<right<<"d"<<
           setw(showlenth)<<right<<"theta"<<
           setw(showlenth)<<right<<"beta"<<std::endl;

    for(int j = 0; j < dof_; j++)
    {
        link_name = "link" + std::to_string(j) + ':';
        afile<<setw(5)<<right<<link_name<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 0]*r2D<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 1]*1000<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 2]*1000<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 3]*r2D<<
               setw(showlenth)<<right<<all_para_[mt_num + 5*j + 4]*r2D<<std::endl;
    }

    afile<<std::endl;
    afile<<left<<"MEASUREMENT AND TOOL PARA "<<"MM"<<std::endl;
    afile<<setw(5)<<right<<"Mx:"<<setw(showlenth)<<right<<cali_para_.measurement_para.x*1000<<std::endl;
    afile<<setw(5)<<right<<"My:"<<setw(showlenth)<<right<<cali_para_.measurement_para.y*1000<<std::endl;
    afile<<setw(5)<<right<<"Mz:"<<setw(showlenth)<<right<<cali_para_.measurement_para.z*1000<<std::endl;

    afile<<setw(5)<<right<<"Rz:"<<setw(showlenth)<<right<<cali_para_.measurement_rpy_para.r*r2D<<std::endl;
    afile<<setw(5)<<right<<"Ry:"<<setw(showlenth)<<right<<cali_para_.measurement_rpy_para.p*r2D<<std::endl;
    afile<<setw(5)<<right<<"Rx:"<<setw(showlenth)<<right<<cali_para_.measurement_rpy_para.y*r2D<<std::endl;

    afile<<setw(5)<<right<<"Tx:"<<setw(showlenth)<<right<<cali_para_.tool_para.x*1000<<std::endl;
    afile<<setw(5)<<right<<"Ty:"<<setw(showlenth)<<right<<cali_para_.tool_para.y*1000<<std::endl;
    afile<<setw(5)<<right<<"Tz:"<<setw(showlenth)<<right<<cali_para_.tool_para.z*1000<<std::endl;

    afile.close();
}

double KinematicsCalibration::getToolPara(std::string index)
{
    double ss;

    if(index == "Tx")
        ss = cali_para_.tool_para.x;
    else if(index == "Ty")
        ss = cali_para_.tool_para.y;
    else if(index == "Tz")
        ss = cali_para_.tool_para.z;
    else if(index == "Mx")
        ss = cali_para_.measurement_para.x;
    else if(index == "My")
        ss = cali_para_.measurement_para.y;
    else if(index == "Mz")
        ss = cali_para_.measurement_para.z;
    else if(index == "MRx")
        ss = cali_para_.measurement_rpy_para.y;
    else if(index == "MRy")
        ss = cali_para_.measurement_rpy_para.p;
    else if(index == "MRz")
        ss = cali_para_.measurement_rpy_para.r;
    else
    {
        ; //error;
    }

    return ss;
}
//read input joint angle
void KinematicsCalibration::getCriter(double output_criter[])
{
    output_criter[0] = criter_after_.max_value;
    output_criter[1] = criter_after_.mean_value;
    output_criter[2] = criter_after_.rms_value;
}

void KinematicsCalibration::updateMemberPara()
{
    int i = 0;
    cali_para_.measurement_para.x += d_allpara[i]; i++;
    cali_para_.measurement_para.y += d_allpara[i]; i++;
    cali_para_.measurement_para.z += d_allpara[i]; i++;

    if(cali_method_ == CALI_POS)
    {
        cali_para_.measurement_rpy_para.r += d_allpara[i]; i++;
        cali_para_.measurement_rpy_para.p += d_allpara[i]; i++;
        cali_para_.measurement_rpy_para.y += d_allpara[i]; i++;
    }
    else
    {
        cali_para_.measurement_rpy_para.r = 0;
        cali_para_.measurement_rpy_para.p = 0;
        cali_para_.measurement_rpy_para.y = 0;
    }

    cali_para_.tool_para.x += d_allpara[i]; i++;
    cali_para_.tool_para.y += d_allpara[i]; i++;
    cali_para_.tool_para.z += d_allpara[i]; i++;

    for(int j = 0; j < dof_; j++)
    {
        for(int k = 0; k < 5; k++)
        {
            if(k == 4)
            {
                cali_para_.beta[j] += d_allpara[i];
                i++;
            }
            else
            {
                cali_para_.dh_para[4*j+k] += d_allpara[i];
                i++;
            }
        }
    }
}

void KinematicsCalibration::setCheckFlag(int index, bool value)
{
    kc_choose_para_[index] = value;
}

void KinematicsCalibration::getCalibrationNum()
{
    int count = 0;
    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
        if(check_flag_[i])
            count ++;
    choose_cali_num_ = count;
}

template <class Type>
Type stringtoNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

int KinematicsCalibration::loadMeasuredData(const std::string&  datafile)
{
    std::ifstream afile;
    std::string filename = "DynaCal1.msr"; //"mesurements.txt";
    //read measure line data
    std::string totalPathName = datafile + filename;
    afile.open(totalPathName.data());

    if(!afile)
    {
        cerr<<"Open File Fail!"<<endl;
        return -1; // open file fail!
    }

    std::string s;
    double count = 0;
    int i = 0;
    while(getline(afile, s))
    {
        if(i == 0)
        {
            try
            {
                count = stringtoNum<double>(s);
            }
            catch(...)
            {
                cout<<"file format error";
                return -2;
            }

            measure_data_length_ = (int)count;
            measure_data_.resize(measure_data_length_);
        }
        else if(i == measure_data_length_ + 1)
            break;
        else
        {
            measure_data_[i-1].resize(data_dim_);
            if(cali_method_ == CALI_LINE)
            {
                count = stringtoNum<double>(s);
                measure_data_[i-1][0] = count / 1000.0;
            }
            else
            {
                int j = 0;
                std::stringstream output(s);
                std::string ss;
                while(output >> ss)
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
                        measure_data_[i-1][j] = count/1000.;
                        j++;
                    }
                }
            }
        }
        i++;
    }
    afile.close();

    return 1;
}

//read input joint angle
int KinematicsCalibration::loadInputJointAngle(const std::string&  datafile)
{
    ifstream afile;
    std::string filename = "point60.txt"; // "CAL.txt";
    std::string totalPathName = datafile + filename;

    input_joint_angle_.resize(measure_data_length_);
    for(int i = 0; i < measure_data_length_; i++)
        input_joint_angle_[i].resize(dof_);

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
    int i = 0;
    while(getline(afile,sline))
    {
        int i = 0;
        std::stringstream output(sline);
        while(output>>ss)
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


            if(i == 0)
            {
                if(cali_method_ == CALI_LINE)
                    continue;
            }

            input_joint_angle_[j][i] = count * D2r;
            if(i == dof_-1 && cali_method_ != CALI_LINE)
                break;

            if(i == dof_ && cali_method_ == CALI_LINE)
                break;
            i++;
        }
        j++;
    }
    afile.close();

    return 1;
}

int KinematicsCalibration::loadInputToolData(const std::string&  datafile)
{
    if(cali_method_ != CALI_LINE)
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

        cali_para_.tool_para.x = temp[1][0]/1000.;
        cali_para_.tool_para.y = temp[1][1]/1000.;
        cali_para_.tool_para.z = temp[1][2]/1000.;

        cali_para_.measurement_para.x = temp[0][0]/1000.;
        cali_para_.measurement_para.y = temp[0][1]/1000.;
        cali_para_.measurement_para.z = temp[0][2]/1000.;
    }
}


