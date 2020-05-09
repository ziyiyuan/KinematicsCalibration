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
        check_flag_[i] = 0;
        DH_check_flag_[i] = 0;
    }

    aral_interface_ = new RLIntface("../../aral_export/aubo_description/urdf/aubo_i5", /*Creat_Share_Memory*/0 | LOG_DEBUG);
    dof_ = 6;
}

KinematicsCalibration::~KinematicsCalibration()
{

}


void KinematicsCalibration::setRobotType(const ROBOT_TYPE type)
{
    robot_type_ = type;
    switch (type) {
    case AUBO_I3:
        aral_interface_->setRobotModel("aubo_i3");
        break;
    case AUBO_I5:
        aral_interface_->setRobotModel("aubo_i5");
        break;
    case AUBO_I7:
        aral_interface_->setRobotModel("aubo_i7");
        break;
    case AUBO_I10:
        aral_interface_->setRobotModel("aubo_i10");
        break;

    default:
        break;
    }
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

void KinematicsCalibration::setCalibrationBeta(const bool value)
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

    All_para_num_ = para_mt_num_ + 5 * dof_;

    // initiall check_flag_;
    for(int i = 0; i < MAX_IDEN_PARA_NUM; i++)
    {
        if(i < para_mt_num_)
            check_flag_[i] = true;
        else
            check_flag_[i] = DH_check_flag_[i - para_mt_num_];
    }

}

bool KinematicsCalibration::calibration()
{
    // est frame para and update tool para

    int res = aral_interface_->calibRobotKinematicsPara(measure_data_, input_joint_angle_, check_flag_, cali_method_, cali_result_);

    memcpy(d_allpara_, cali_result_.all_d_para_, sizeof(double)*MAX_IDEN_PARA_NUM);

    calAllPara();

    return true;
}

void KinematicsCalibration::calAllPara()
{
    int i = 0;
    all_para_[i] = cali_result_.para_calid.measurement_para.x;i++;
    all_para_[i] = cali_result_.para_calid.measurement_para.y;i++;
    all_para_[i] = cali_result_.para_calid.measurement_para.z;i++;

    if(cali_method_ == CALI_POS)
    {
        all_para_[i] = cali_result_.para_calid.measurement_rpy_para.r;i++;
        all_para_[i] = cali_result_.para_calid.measurement_rpy_para.p;i++;
        all_para_[i] = cali_result_.para_calid.measurement_rpy_para.y;i++;
    }

    all_para_[i] = cali_result_.para_calid.tool_para.x;i++;
    all_para_[i] = cali_result_.para_calid.tool_para.y;i++;
    all_para_[i] = cali_result_.para_calid.tool_para.z;i++;

    for(int j = 0; j < dof_; j++)
    {
        for(int k = 0; k < 5; k++)
        {
            if(k == 4)
            {
                all_para_[i] = cali_result_.para_calid.beta[j];i++;
            }
            else
            {
                all_para_[i] = cali_result_.para_calid.dh_para[4*j+k];i++;
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
        all_d_para[i] = cali_result_.all_d_para_[i];
}

int KinematicsCalibration::getMtNum()
{
    return para_mt_num_;
}

void KinematicsCalibration::getCriter(double output_criter[])
{
    output_criter[0] = cali_result_.max_value;
    output_criter[1] = cali_result_.mean_value;
    output_criter[2] = cali_result_.rms_value;
}


//output result
void KinematicsCalibration::outputClibrationDPara(const string path, const string data_file)
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
               setw(showlenth)<<right<<d_allpara_[mt_num + 5*j + 0]*r2D<<
               setw(showlenth)<<right<<d_allpara_[mt_num + 5*j + 1]*1000<<
               setw(showlenth)<<right<<d_allpara_[mt_num + 5*j + 2]*1000<<
               setw(showlenth)<<right<<d_allpara_[mt_num + 5*j + 3]*r2D<<
               setw(showlenth)<<right<<d_allpara_[mt_num + 5*j + 4]*r2D<<std::endl;
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
    afile<<setw(5)<<right<<"Mx:"<<setw(showlenth)<<right<<cali_result_.para_calid.measurement_para.x*1000<<std::endl;
    afile<<setw(5)<<right<<"My:"<<setw(showlenth)<<right<<cali_result_.para_calid.measurement_para.y*1000<<std::endl;
    afile<<setw(5)<<right<<"Mz:"<<setw(showlenth)<<right<<cali_result_.para_calid.measurement_para.z*1000<<std::endl;

    afile<<setw(5)<<right<<"Rz:"<<setw(showlenth)<<right<<cali_result_.para_calid.measurement_rpy_para.r*r2D<<std::endl;
    afile<<setw(5)<<right<<"Ry:"<<setw(showlenth)<<right<<cali_result_.para_calid.measurement_rpy_para.p*r2D<<std::endl;
    afile<<setw(5)<<right<<"Rx:"<<setw(showlenth)<<right<<cali_result_.para_calid.measurement_rpy_para.y*r2D<<std::endl;

    afile<<setw(5)<<right<<"Tx:"<<setw(showlenth)<<right<<cali_result_.para_calid.tool_para.x*1000<<std::endl;
    afile<<setw(5)<<right<<"Ty:"<<setw(showlenth)<<right<<cali_result_.para_calid.tool_para.y*1000<<std::endl;
    afile<<setw(5)<<right<<"Tz:"<<setw(showlenth)<<right<<cali_result_.para_calid.tool_para.z*1000<<std::endl;

    afile.close();
}

double KinematicsCalibration::getToolPara(const string index)
{
    double ss;

    if(index == "Tx")
        ss = cali_result_.para_calid. tool_para.x;
    else if(index == "Ty")
        ss = cali_result_.para_calid.tool_para.y;
    else if(index == "Tz")
        ss = cali_result_.para_calid.tool_para.z;
    else if(index == "Mx")
        ss = cali_result_.para_calid.measurement_para.x;
    else if(index == "My")
        ss = cali_result_.para_calid.measurement_para.y;
    else if(index == "Mz")
        ss = cali_result_.para_calid.measurement_para.z;
    else if(index == "MRx")
        ss = cali_result_.para_calid.measurement_rpy_para.y;
    else if(index == "MRy")
        ss = cali_result_.para_calid.measurement_rpy_para.p;
    else if(index == "MRz")
        ss = cali_result_.para_calid.measurement_rpy_para.r;
    else
    {
        ; //error;
    }

    return ss;
}


void KinematicsCalibration::setCheckFlag(const int index, const bool value)
{
    DH_check_flag_[index] = value;
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
    std::string filename;
    if(cali_method_ == CALI_POS)
        filename = "mesurements.txt";
    else
        filename = "DynaCal1.msr";

//    std::string filename = "DynaCal1.msr"; //"mesurements.txt";
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
//    // output
//    for(int aa = 0; aa < measure_data_length_; aa++)
//    {
//        for(int j = 0; j < data_dim_; j++)
//        {
//            std::cout << measure_data_[aa][j] << " ";
//        }
//        std::cout << std::endl;
//    }
    return 1;
}

//read input joint angle
int KinematicsCalibration::loadInputJointAngle(const std::string&  datafile)
{
    ifstream afile;
    std::string filename;
//    std::string filename = "point60.txt"; // "CAL.txt";
    if(cali_method_ == CALI_POS)
        filename = "CAL.txt";
    else
        filename = "point60.txt";
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
    while(getline(afile,sline))
    {
        int i = 0;
        std::stringstream output(sline);
        while(output>>ss)
        {
            if(i == 0)
            {
                i++;
                if(cali_method_ == CALI_LINE)
                    continue;
            }
            if(i == dof_+1 )
            {
                break;
            }

            try
            {
                count = stringtoNum<double>(ss);
            }

            catch(...)
            {
                cout<<"file format error";
                return -2;
            }

            input_joint_angle_[j][i-1] = count* D2r;
            i++;
        }
        j++;
    }
    afile.close();
//    //output

//    for(int i = 0; i < measure_data_length_; i++)
//    {
//        for(int j = 0; j < 6; j++)
//        {
//            std::cout << input_joint_angle_[i][j] << " ";
//        }
//        std::cout << std::endl;
//    }
    return 1;
}

int KinematicsCalibration::loadInputToolData(const std::string&  datafile)
{
    if(cali_method_ == CALI_LINE)
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

        //        cali_para_.tool_para.x = temp[1][0]/1000.;
        //        cali_para_.tool_para.y = temp[1][1]/1000.;
        //        cali_para_.tool_para.z = temp[1][2]/1000.;

        //        cali_para_.measurement_para.x = temp[0][0]/1000.;
        //        cali_para_.measurement_para.y = temp[0][1]/1000.;
        //        cali_para_.measurement_para.z = temp[0][2]/1000.;
        return 1;
    }
    return -1;
}


