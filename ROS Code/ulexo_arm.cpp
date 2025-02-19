/*
Copyright (c) 2022, Rehabilitation Research Institute of Singapore
Author: Wei Chuan, Lincong Luo
*/

#include "ulexo_arm/ulexo_arm.hpp"
#include "ulexo_arm/stopwatch.hpp"
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split
#include<cmath>

using namespace Eigen;
using namespace ARAE;

namespace ulexo
{

ULEArm::ULEArm(): nh_("~")/* robot_running_(false) */
{
    // Read parameters from launch file
    nh_.param<float>("loop_rate",           control_period_ms_,     2.00);
    nh_.param<bool>("is_female",            param_.Cat1.is_female,  false);
    nh_.param<double>("body_weight",        param_.Cat1.wt,         63.0);
    nh_.param<double>("upperarm_len",       param_.Cat1.ul,         0.32);
    nh_.param<double>("forearm_len",        param_.Cat1.fl,         0.28);
    nh_.param<double>("trunk_height",       param_.Cat1.th,         0.4161);
    nh_.param<double>("trunk_width",        param_.Cat1.tw,         -0.1826);
    // nh_.param<double>("upperarm_scale_COM", param_.Cat1.Ugc,        0.427);
    // nh_.param<double>("forearm_scale_COM",  param_.Cat1.Lgc,        0.417);
    // nh_.param<double>("forearm_len_c",      param_.Cat1.fl_c,       0.00);

    nh_.param<double>("Px_shoulder2robot", param_.Cat2.p_shoulder_robot[0], 0.3);
    nh_.param<double>("Py_shoulder2robot", param_.Cat2.p_shoulder_robot[1], 0.1);
    nh_.param<double>("Pz_shoulder2robot", param_.Cat2.p_shoulder_robot[2], 0.3);
    nh_.param<double>("Px_pelvis2robot", param_.Cat2.p_pelvis_robot[0], 0.3);
    nh_.param<double>("Py_pelvis2robot", param_.Cat2.p_pelvis_robot[1], 0.1);
    nh_.param<double>("Pz_pelvis2robot", param_.Cat2.p_pelvis_robot[2], 0.3);
    
    nh_.param<double>("load_weight",     param_.Cat3.load_wt,           0.0);
    nh_.param<double>("control_scaling", param_.Cat3.control_scaling,   1.0);

    nh_.param<int>("control_mode",  param_.Cat4.control_mode,   10); // 10-Stop
    nh_.param<int>("shoulder_mode", param_.Cat4.shoulder_mode,  0);

    nh_.param<float>("m1p_manual_params",  param_.Cat5.mP[0] , 0.0);
    nh_.param<float>("m1v_manual_params",  param_.Cat5.mV[0] , 0.0);
    nh_.param<float>("m1t_manual_params",  param_.Cat5.mT[0] , 0.0);
    nh_.param<float>("m1kp_manual_params", param_.Cat5.mKp[0], 0.0);
    nh_.param<float>("m1kd_manual_params", param_.Cat5.mKd[0], 0.0);
    nh_.param<float>("m2p_manual_params",  param_.Cat5.mP[1] , 0.0);
    nh_.param<float>("m2v_manual_params",  param_.Cat5.mV[1] , 0.0);
    nh_.param<float>("m2t_manual_params",  param_.Cat5.mT[1] , 0.0);
    nh_.param<float>("m2kp_manual_params", param_.Cat5.mKp[1], 0.0);
    nh_.param<float>("m2kd_manual_params", param_.Cat5.mKd[1], 0.0);
    nh_.param<float>("m3p_manual_params",  param_.Cat5.mP[2] , 0.0);
    nh_.param<float>("m3v_manual_params",  param_.Cat5.mV[2] , 0.0);
    nh_.param<float>("m3t_manual_params",  param_.Cat5.mT[2] , 0.0);
    nh_.param<float>("m3kp_manual_params", param_.Cat5.mKp[2], 0.0);
    nh_.param<float>("m3kd_manual_params", param_.Cat5.mKd[2], 0.0);

    nh_.param<int>("idle_mode", idle_mode_, 0);  

    // Initialization
    p_TC_   = std::make_unique<tmotor::TC>();
    p_ARAE_ = std::make_unique<ARAE::ARAEarm>();

    // ROS initialization
    topic_arm2stm_ = "/ulexo_stm_node/arm2stm";
    topic_arm2gui_ = "/ulexo_stm_node/arm2gui";
    topic_stm2arm_ = "/ulexo_stm_node/stm2arm";


    // Topics
    sub_stm2arm_   = nh_.subscribe(topic_stm2arm_, 1, &ULEArm::subscribeStmData, this);
    pub_arm2stm_   = nh_.advertise<ulexo_stm_msg::MotorCmd>(topic_arm2stm_, 1);
    pub_arm2gui_   = nh_.advertise<ulexo_stm_msg::ControllerState>(topic_arm2gui_, 1);

    // Services
    server_set_arm_para_ = nh_.advertiseService("/ulexo_arm/set_para", &ULEArm::setArmParamService, this);
    server_runmode_arm_para_ = nh_.advertiseService("/ulexo_stm/set_run_mode_param", &ULEArm::setArmParamService_WhenRun, this);
    // server_get_arm_para_ = nh_.advertiseService("/ulexo_arm/get_para", &ULEArm::getArmParamService, this);
}

ULEArm::~ULEArm()
{
    // Do nothing
    control_thread_running_ = false;
    control_thread_.join();
}

/**
 * @brief control loop running different control methods
 * @param
*/
void ULEArm::ControlLoop(float period_ms)
{
    StopWatch sw;

    // std::unique_ptr<tmotor::TC> torqueNode = std::make_unique<tmotor::TC>();

    UlexoState r_state;
    double robot_joint[4][3];
    // double human_properties[8]={0};
    HumanPara h_para;
    RobotPara r_para;
    Human2robot h2r_para;
    Huamn_angle ang_h;
    
    double load = 0;
    double U_cal = 0;
    Vector3d Ps_c(0,0,0), Ps_e(0,0,0), Ps_w(0,0,0), Ps(0,0,0);
    Vector3d Pp_c(0,0,0), Pp_e(0,0,0), Pp_w(0,0,0);
    Vector3d end_support_force(0,0,0);
    Vector3d torque(0,0,0);

    while ( control_thread_running_ /*ros::ok()*/)
    {
        sw.tic();

        if(running_control_)
        {

            /** start control and update robot state **/
            GetRobotState(r_state,robot_joint);
            // robot_joint[0][0] = -0.013;
            // robot_joint[0][1] = 0;
            // robot_joint[0][2] = 0;
            // robot_joint[1][0] = 0.6477;
            // robot_joint[1][1] = 0;
            // robot_joint[1][2] = 0;
            // robot_joint[2][0] = 2.3184;
            // robot_joint[2][1] = 0;
            // robot_joint[2][2] = 0;
            // robot_joint[3][0] = 0.8011;
            // robot_joint[3][1] = 1.5820;
            // robot_joint[3][2] = 0;                                        
            /** human and robot property update **/
            // UpdateHumanProperty(r_state,human_properties);
            UpdateParameters(param_,h_para,h2r_para,r_para);
            p_ARAE_->UpdateParams(h_para,h2r_para,r_para);

            torque.setZero();

            // std::cout<<"param_.Cat4.control_mode"<<param_.Cat4.control_mode<<std::endl;
            // std::cout<< "=============================" << param_.Cat4.shoulder_mode <<","<< 
            //          h_para.wt <<","<< h_para.th <<","<< h_para.tw <<","<<
            //          h_para.ul <<","<< h_para.fl<<"," << param_.Cat1.is_female <<"," << "RL_Load"<<
            //          h2r_para.xsr <<","<< h2r_para.ysr<<","<<
            //          h2r_para.zsr <<","<< h2r_para.xpr <<","<< h2r_para.ypr <<","<< h2r_para.zpr<<","<<
            //          h_para.ws <<","<< h_para.Ugc <<","<< h_para.Lgc <<","<< h_para.fl_c<<","<<
            //          std::endl;

            // calculate support force
            switch(param_.Cat4.control_mode)
            {
                case ROBOT_GRAV:
                    // std::cout<<"In pure robot grav compensation..."<<std::endl;
                    load = 0;
                    end_support_force.setZero();
                    torque = p_ARAE_->RobotTorque(*robot_joint,1,end_support_force,load);
                    // torque = p_TC_->torque_cal(*robot_joint,1,end_support_force,load);
                break;
                case FIXED_LOAD:
                    // std::cout<<"In fixed load mode..."<<std::endl;

                    load = param_.Cat3.load_wt;
                    end_support_force.setZero();
                    torque = p_ARAE_->RobotTorque(*robot_joint,1,end_support_force,load);
                    // torque = p_TC_->torque_cal(*robot_joint,1,end_support_force,load);
                break;
            
            
                case RIGID_LINK:
                    /////////////////// * FIxed torso model * ///////////////////
                    if(param_.Cat4.shoulder_mode == 0){
                        std::cout<<"rigid link fixed shoulder"<<std::endl;
                        // end_support_force = p_TC_->human_cp(*robot_joint,human_properties);
                        // end_support_force = end_support_force * param_.Cat3.control_scaling;
                        // torque = p_TC_->torque_cal(*robot_joint,1,end_support_force,load);
                        load = param_.Cat3.load_wt;
                        p_ARAE_->Robot_FK(*robot_joint,h2r_para.xsr,h2r_para.ysr,h2r_para.zsr,Ps_c, Ps_e, Ps_w);
                        p_ARAE_->Arm_IK(Ps_e,Ps_w,U_cal,ang_h);
                        p_ARAE_->CalFh_rigid(ang_h,h_para,U_cal,Ps_c,end_support_force);
                        end_support_force = end_support_force * param_.Cat3.control_scaling;
                        torque = p_ARAE_->RobotTorque(*robot_joint,1,end_support_force,load);
                    }
                    ///////////////// * Sagittal plane model * /////////////////////////
                    else{
                        std::cout<<"rigid link Sagittal plane"<<std::endl;
                        load = param_.Cat3.load_wt;
                        // end_support_force = p_TC_->human_cp(*robot_joint,human_properties);
                        // end_support_force = end_support_force * param_.Cat3.control_scaling;
                        // torque = p_TC_->torque_cal(*robot_joint,1,end_support_force,load);
                        p_ARAE_->Robot_FK(*robot_joint,h2r_para.xpr,h2r_para.ypr,h2r_para.zpr,Pp_c, Pp_e, Pp_w);
                        p_ARAE_->SagittalPlane(Pp_e,Pp_w,Ps,Ps_e,Ps_w);
                        p_ARAE_->Arm_IK(Ps_e,Ps_w,U_cal,ang_h);
                        Ps_c = (Ps_e + Ps_w)/2;
                        p_ARAE_->CalFh_rigid(ang_h,h_para,U_cal,Ps_c,end_support_force);
                        end_support_force = end_support_force * param_.Cat3.control_scaling;
                        torque = p_ARAE_->RobotTorque(*robot_joint,1,end_support_force,load);
                    }
                break;

                case ARM_DYNAMIC:
                    /////////////////// * FIxed torso model * ///////////////////
                    if(param_.Cat4.shoulder_mode == 0){
                        std::cout<<"arm dynamic fixed shoulder"<<std::endl;
                        load = param_.Cat3.load_wt;
                        // end_support_force = p_TC_->human_cp_jac(*robot_joint,human_properties,controller_state_.human_state.joint);
                        // end_support_force = end_support_force * param_.Cat3.control_scaling;
                        // torque = p_TC_->torque_cal(*robot_joint,1,end_support_force,load);                  
                        p_ARAE_->Robot_FK(*robot_joint,h2r_para.xsr,h2r_para.ysr,h2r_para.zsr,Ps_c, Ps_e, Ps_w);
                        p_ARAE_->Arm_IK(Ps_e,Ps_w,U_cal,ang_h);
                        p_ARAE_->CalFh_armdynamics(ang_h,h_para,U_cal,end_support_force, load);
                        end_support_force = end_support_force * param_.Cat3.control_scaling;
                        torque = p_ARAE_->RobotTorque(*robot_joint,1,end_support_force,load);
                    }
                     ///////////////// * Sagittal plane model * /////////////////////////
                    else{
                        std::cout<<"arm dynamic Sagittal plane"<<std::endl;
                        load = param_.Cat3.load_wt;
                        // end_support_force = p_TC_->human_cp_jac(*robot_joint,human_properties,controller_state_.human_state.joint);
                        // end_support_force = end_support_force * param_.Cat3.control_scaling;
                        // torque = p_TC_->torque_cal(*robot_joint,1,end_support_force,load);                  
                        p_ARAE_->Robot_FK(*robot_joint,h2r_para.xpr,h2r_para.ypr,h2r_para.zpr,Pp_c, Pp_e, Pp_w);
                        p_ARAE_->SagittalPlane(Pp_e,Pp_w,Ps,Ps_e,Ps_w);
                        p_ARAE_->Arm_IK(Ps_e,Ps_w,U_cal,ang_h);
                        p_ARAE_->CalFh_armdynamics(ang_h,h_para,U_cal,end_support_force, load);
                        end_support_force = end_support_force * param_.Cat3.control_scaling;
                        torque = p_ARAE_->RobotTorque(*robot_joint,1,end_support_force,load);
                    }
                break;

                case MANUAL_CTRL:
                    std::cout<<"In arm dynamic mode..."<<std::endl;
                    torque[0] = param_.Cat5.mT[0];
                    torque[1] = param_.Cat5.mT[1];
                    torque[2] = param_.Cat5.mT[2];
                break;

                case STOP:
                    torque.setZero();
                break;

                default:
                break;
            }

            // std::cout<<"End support force: "<< end_support_force(0) <<", "<< end_support_force(1)<<", "<<end_support_force(2)<< std::endl;
            // std::cout<<"Joint torque: "<< torque(0) <<", "<< torque(1)<<", "<<torque(2)<< std::endl;

            /** publish controller results   **/
            // std_msgs::Float32MultiArray msg;
            
            ulexo_stm_msg::MotorCmd msg;

            msg.m_p[0]  = 0;
            msg.m_p[1]  = 0;
            msg.m_p[2]  = 0;
            // Temporary to show the end effector support force
            // msg.m_v[0]  = end_support_force[0];
            // msg.m_v[1]  = end_support_force[1];
            // msg.m_v[2]  = end_support_force[2];
            msg.m_v[0]  = 0;
            msg.m_v[1]  = 0;
            msg.m_v[2]  = 0;
            msg.m_t[0]  = torque[0];
            msg.m_t[1]  = torque[1];
            msg.m_t[2]  = torque[2];
            msg.m_kp[0]  = 0;
            msg.m_kp[1]  = 0;
            msg.m_kp[2]  = 0;
            msg.m_kd[0]  = 0;
            msg.m_kd[1]  = 0;
            msg.m_kd[2]  = 0;
            // for (int i = 0; i < 3; i++)
            // {   
                // msg.data.push_back((_Float32)0);
                // msg.data.push_back((_Float32)0);
                // msg.data.push_back((_Float32)torque(i));
                // msg.data.push_back((_Float32)0);
                // msg.data.push_back((_Float32)0);
            // }
            pub_arm2stm_.publish(msg);
            
            ulexo_stm_msg::ControllerState msg_ctr;
            
            // Raw data from Stm Pkg
            // msg_ctr.mP[0]                           = r_state.mP[0];
            // msg_ctr.mP[1]                           = r_state.mP[1];
            // msg_ctr.mP[2]                           = r_state.mP[2];
            // msg_ctr.mV[0]                           = r_state.mV[0];
            // msg_ctr.mV[1]                           = r_state.mV[1];
            // msg_ctr.mV[2]                           = r_state.mV[2];
            // msg_ctr.mT[0]                           = r_state.mT[0];
            // msg_ctr.mT[1]                           = r_state.mT[1];
            // msg_ctr.mT[2]                           = r_state.mT[2];
            // msg_ctr.enc[0]                          = r_state.enc[0];
            // msg_ctr.enc[1]                          = r_state.enc[1];
            // msg_ctr.fc[0]                           = r_state.fc[0];
            // msg_ctr.fc[1]                           = r_state.fc[1];
            // msg_ctr.fc[2]                           = r_state.fc[2];
            std::copy(std::begin(r_state.mP),  std::end(r_state.mP),  std::begin(msg_ctr.mP));
            std::copy(std::begin(r_state.mV),  std::end(r_state.mV),  std::begin(msg_ctr.mV));
            std::copy(std::begin(r_state.mT),  std::end(r_state.mT),  std::begin(msg_ctr.mT));
            std::copy(std::begin(r_state.enc), std::end(r_state.enc), std::begin(msg_ctr.enc));
            std::copy(std::begin(r_state.fc),  std::end(r_state.fc),  std::begin(msg_ctr.fc));
            msg_ctr.t    = r_state.t   ;



            // Calculated data from Arm Pkg
            // msg_ctr.header          = "Ctrl_state";
            timeval curTime;
            gettimeofday(&curTime, NULL);
            int milli = curTime.tv_usec / 1000;
            char buffer [80];
            strftime(buffer, 80, "%Y%m%d_%H%M%S", localtime(&curTime.tv_sec));
            char currentTime[84] = "";
            sprintf(currentTime, "%s.%03d", buffer, milli);
            std::string currentTime_str(currentTime);
            // std::cout<<currentTime_str<<std::endl;

            msg_ctr.control_mode                    = param_.Cat4.control_mode;
            msg_ctr.robot_angle[0]                  = controller_state_.robot_state.mP[0];
            msg_ctr.robot_angle[1]                  = controller_state_.robot_state.mP[1];
            msg_ctr.robot_angle[2]                  = controller_state_.robot_state.mP[2];
            msg_ctr.robot_angle[3]                  = controller_state_.robot_state.enc[0];
            msg_ctr.robot_angle[4]                  = controller_state_.robot_state.enc[1];
            // msg_ctr.human_angle[0]                  = controller_state_.human_state.joint[0];
            // msg_ctr.human_angle[1]                  = controller_state_.human_state.joint[1];
            // msg_ctr.human_angle[2]                  = controller_state_.human_state.joint[2];
            // msg_ctr.human_angle[3]                  = controller_state_.human_state.joint[3];
            msg_ctr.human_angle[0]                  = ang_h.h1;
            msg_ctr.human_angle[1]                  = ang_h.h2;
            msg_ctr.human_angle[2]                  = ang_h.h3;
            msg_ctr.human_angle[3]                  = ang_h.h4;
            msg_ctr.end_effector_support_force.x    = end_support_force[0];
            msg_ctr.end_effector_support_force.y    = end_support_force[1];
            msg_ctr.end_effector_support_force.z    = end_support_force[2];
            msg_ctr.robot_torque[0]                 = torque[0];
            msg_ctr.robot_torque[1]                 = torque[1];
            msg_ctr.robot_torque[2]                 = torque[2];
            msg_ctr.t_ms                            = currentTime_str;
            msg_ctr.scaling                         = param_.Cat3.control_scaling;
            pub_arm2gui_.publish(msg_ctr);
            
            // Publish data from arm to stm32
            // if(param_.Cat4.control_mode != MANUAL_CTRL)
            // {
            //     pub_arm2stm_.publish(msg); // no control cmd is published in manual control mode
            // }

            // Publish controller state including intermediate results    

        }
        sw.sleep_until_ms(period_ms);
    }
}

/**
 * @brief
 * 
*/
void ULEArm::GetRobotState(UlexoState &state, double robot_joint[4][3])
{
    state_mutex_.lock();
    state = act_robot_state_;
    state_mutex_.unlock();

    double deg2rad = M_PI / 180.0;
    for(int i=0;i<3;i++)
    {
        robot_joint[i][0] = state.mP[i] * deg2rad;
        robot_joint[i][1] = state.mV[i] * deg2rad;
        robot_joint[i][2] = 0.0;
    }
    robot_joint[3][0] = state.enc[0] * deg2rad;
    robot_joint[3][1] = state.enc[1] * deg2rad;
    robot_joint[3][2] = 0.0;

    // std::cout<<"/**** Robot joint angle: "<< robot_joint[0][0] << ", "<< robot_joint[1][0]<< ", " << robot_joint[2][0] <<std::endl;
    // double  P[4][3] = {{m1P*deg2rad,m1V,m1A},{m2P*deg2rad,m2V,m2A},{m3P*deg2rad,m3V,m3A},{enc1*deg2rad,enc2*deg2rad,0}};
}
/**
 * @brief Update human property and estimate shoulder position
*/
void ULEArm::UpdateParameters(UlexoArmParam param_, HumanPara &h_para, Human2robot &h_r_para, RobotPara &r_para){
    h_para.gender = param_.Cat1.is_female == 1?false:true;
    h_para.wt = param_.Cat1.wt;
    h_para.ul = param_.Cat1.ul;
    h_para.fl = param_.Cat1.fl;
    h_para.th = param_.Cat1.th;
    h_para.tw = param_.Cat1.tw;

    r_para.l7 = param_.Cat1.fl;
    // the robot base position under shoulder frame //
    h_r_para.xsr = param_.Cat2.p_shoulder_robot[0];
    h_r_para.ysr = param_.Cat2.p_shoulder_robot[1];
    h_r_para.zsr = param_.Cat2.p_shoulder_robot[2];
    // the robot base position under pelvis frame //
    h_r_para.xpr = param_.Cat2.p_pelvis_robot[0];
    h_r_para.ypr = param_.Cat2.p_pelvis_robot[1];
    h_r_para.zpr = param_.Cat2.p_pelvis_robot[2];
}
// void ULEArm::UpdateHumanProperty(const UlexoState &state, double hp[])
// {
//     double s_x,s_y,s_z;
//     /* Estimate shoulder position (displacement from robot in shoulder frame)*/
    
//     // method-1: assume shoulder is fixed
//     s_x = param_.Cat2.p_shoulder_robot[0];
//     s_y = param_.Cat2.p_shoulder_robot[1];
//     s_z = param_.Cat2.p_shoulder_robot[2];

//     // method-2 estimate shoulde position (plane method)


//     p_TC_->human_prop_cal(param_.Cat1.wt,param_.Cat1.fl,param_.Cat1.is_female,s_x,s_y,s_z,hp);

// }

/**
 * @brief Run Arm node core functions
 * 
 */
void ULEArm::runArm()
{
    // Initialize control loop
    control_thread_running_ = true;
    control_thread_ = std::thread(std::bind(&ULEArm::ControlLoop, this, control_period_ms_));
    std::cout << "start control loop thread ....." << std::endl;
    
    // ROS_INFO_STREAM("Robot ready...");
    // updateArmState(ArmState::AVAILABLE);
    // timer_publish_arm_state_   = nh_.createWallTimer(ros::WallDuration(1.0 / loop_rate_), &ULEArm::publishArmState, this);
    while (ros::ok())
    {   
        ros::spinOnce();
    }
}

/**
 * @brief call back function for receiving data from stm pkg
 * @param msg
*/
void ULEArm::subscribeStmData(const ulexo_stm_msg::StmToArm& msg)
{
    running_control_ = true;

    state_mutex_.lock();
    act_robot_state_.mP[0] = msg.m1P;
    act_robot_state_.mV[0] = msg.m1V;
    act_robot_state_.mT[0] = msg.m1T;
    act_robot_state_.mP[1] = msg.m2P;
    act_robot_state_.mV[1] = msg.m2V;
    act_robot_state_.mT[1] = msg.m2T;
    act_robot_state_.mP[2] = msg.m3P; //+ 180.0;
    act_robot_state_.mV[2] = msg.m3V;
    act_robot_state_.mT[2] = msg.m3T;
    act_robot_state_.enc[0] = msg.enc1;
    act_robot_state_.enc[1] = msg.enc2;
    act_robot_state_.fc[0] = msg.fcX;
    act_robot_state_.fc[1] = msg.fcY;
    act_robot_state_.fc[2] = msg.fcZ;
    act_robot_state_.t = msg.t;
    state_mutex_.unlock();

    return;
}

/**
 * Get user input value from GUI pkg to ARM
*/
bool ULEArm::setArmParamService(ulexo_stm_msg::SetArmParam::Request &req, ulexo_stm_msg::SetArmParam::Response &res)
{

            // timeval curTime;
            // gettimeofday(&curTime, NULL);
            // int milli = curTime.tv_usec / 1000;
            // char buffer [80];
            // strftime(buffer, 80, "%Y%m%d_%H%M%S", localtime(&curTime.tv_sec));
            // char currentTime[84] = "";
            // sprintf(currentTime, "%s.%03d", buffer, milli);
            // std::string currentTime_str(currentTime);
            // std::cout<<"Type:"<<std::to_string(req.control_mode)<<", Time:"<<currentTime_str<<std::endl;

    // cmd_mutex_.lock();
    param_.SetCat.setting_type      = req.setting_type;
    param_.Cat1.is_female           = req.is_female;
    param_.Cat1.wt                  = req.wt;
    // param_.Cat1.ws                  = req.ws;
    param_.Cat1.ul                  = req.ul;
    param_.Cat1.fl                  = req.fl;
    param_.Cat1.th                  = req.th; 
    param_.Cat1.tw                  = req.tw;
    // param_.Cat1.Ugc                 = req.Ugc;
    // param_.Cat1.Lgc                 = req.Lgc; 
    // param_.Cat1.fl_c                = req.fl_c;

    param_.Cat2.p_shoulder_robot[0] = req.p_shoulder_robot.x; 
    param_.Cat2.p_shoulder_robot[1] = req.p_shoulder_robot.y; 
    param_.Cat2.p_shoulder_robot[2] = req.p_shoulder_robot.z;
    param_.Cat2.p_pelvis_robot[0]   = req.p_pelvis_robot.x; 
    param_.Cat2.p_pelvis_robot[1]   = req.p_pelvis_robot.y; 
    param_.Cat2.p_pelvis_robot[2]   = req.p_pelvis_robot.z;

    param_.Cat3.load_wt             = req.load_wt; 
    param_.Cat3.control_scaling     = req.control_scaling;
    param_.Cat4.shoulder_mode       = req.shoulder_mode;     
    param_.Cat4.control_mode        = req.control_mode;

    // param_.Cat5.mP                  = req.mP    ;
    // param_.Cat5.mV                  = req.mV    ;
    // param_.Cat5.mT                  = req.mT    ;
    // param_.Cat5.mKp                 = req.mKp   ;
    // param_.Cat5.mKd                 = req.mKd   ;

    std::copy(std::begin(req.mP),  std::end(req.mP),  std::begin(param_.Cat5.mP));
    std::copy(std::begin(req.mV),  std::end(req.mV),  std::begin(param_.Cat5.mV));
    std::copy(std::begin(req.mT),  std::end(req.mT),  std::begin(param_.Cat5.mT));
    std::copy(std::begin(req.mKp), std::end(req.mKp), std::begin(param_.Cat5.mKp));
    std::copy(std::begin(req.mKd), std::end(req.mKd), std::begin(param_.Cat5.mKd));

    // std::cout<<"param_.Cat"<<param_.Cat5.mT[1]<<","<<param_.Cat5.mT[2]<<std::endl;
    // param_.Cat5.mP[0]               = req.mP[0]  ;
    // param_.Cat5.mV[0]               = req.mV[0]  ;
    // param_.Cat5.mT[0]               = req.mT[0]  ;
    // param_.Cat5.mKp[0]              = req.mKp[0] ;
    // param_.Cat5.mKd[0]              = req.mKd[0] ;
    // param_.Cat5.mP[1]               = req.mP[1]  ;
    // param_.Cat5.mV[1]               = req.mV[1]  ;
    // param_.Cat5.mT[1]               = req.mT[1]  ;
    // param_.Cat5.mKp[1]              = req.mKp[1] ;
    // param_.Cat5.mKd[1]              = req.mKd[1] ;
    // param_.Cat5.mP[2]               = req.mP[2]  ;
    // param_.Cat5.mV[2]               = req.mV[2]  ;
    // param_.Cat5.mT[2]               = req.mT[2]  ;
    // param_.Cat5.mKp[2]              = req.mKp[2] ;
    // param_.Cat5.mKd[2]              = req.mKd[2] ;
    // cmd_mutex_.unlock();


    // When arm stops, STOP mode is overwritten by the predefined Default mode
    if (param_.Cat4.control_mode == STOP)
    {
        switch(idle_mode_)
        {
            case 0:
                param_.Cat4.control_mode = ROBOT_GRAV;  
            break;
            case 1:
                param_.Cat4.control_mode = FIXED_LOAD;
            break;
            case 2:
                param_.Cat4.control_mode = RIGID_LINK;
            break;
            case 3:
                param_.Cat4.control_mode = ARM_DYNAMIC;
            break;
            case 4:
                param_.Cat4.control_mode = MANUAL_CTRL;
            break;
            case 10:
                param_.Cat4.control_mode = STOP;
            break;
        }
        // if(idle_mode_ == 0)  param_.Cat4.control_mode = ROBOT_GRAV;
        // if(idle_mode_ == 1)  param_.Cat4.control_mode = FIXED_LOAD;
        // if(idle_mode_ == 2)  param_.Cat4.control_mode = RIGID_LINK;
        // if(idle_mode_ == 3)  param_.Cat4.control_mode = ARM_DYNAMIC;
        // if(idle_mode_ == 4)  param_.Cat4.control_mode = MANUAL_CTRL;
        // if(idle_mode_ == 10) param_.Cat4.control_mode = STOP;
    }

    res.success = true;
    return true;
}


bool ULEArm::setArmParamService_WhenRun(ulexo_stm_msg::SetRunModeParam::Request &req, ulexo_stm_msg::SetRunModeParam::Response &res)
{
    param_.Cat3.control_scaling     = req.control_scaling_whenrun;
    res.success = true;
    return true;
}
/**
 * 
*/
// bool ULEArm::getArmParamService(ulexo_stm_msg::GetArmParam::Request &req, ulexo_stm_msg::GetArmParam::Response &res)
// {
    // res.is_female = param_.Cat1.is_female;
    // res.wt = param_.Cat1.wt;
    // res.ul = param_.ul;
    // res.fl = param_.fl;
    // res.trunk_height = param_.th;
    // res.trunk_width = param_.tw;
    
    // res.p_shoulder_robot.x = param_.p_shoulder_robot[0];
    // res.p_shoulder_robot.y = param_.p_shoulder_robot[1];
    // res.p_shoulder_robot.z = param_.p_shoulder_robot[2];
    
    // res.load_weight = param_.load_wt;
    // res.scaling = param_.control_scaling;
    
    // res.shoulder_mode = param_.shoulder_mode; 
    // res.control_mode = param_.control_mode;
    
    // return true;
// }

} // namespace ulexo