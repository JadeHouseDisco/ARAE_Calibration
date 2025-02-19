/*
* Created on: May 2, 2023
* Author: Lincong Luo , Sibo Yang
*/

#ifndef ARAE_HPP
#define ARAE_HPP

#include <cmath>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;

namespace ARAE{

struct HumanPara
{
    bool gender = false;    // Get From GUI user input
    double wt = 0;          // Get From GUI user input
    double ws = 1;
    double Ugc = 0.427; // upper arm length scaling to COM
    double Lgc = 0.417; // forearm length scaling to COM
    double ul = 0;          // Get From GUI user input
    double fl = 0;          // Get From GUI user input
    double fl_c = 0.14;
    double th = 0.3704;     // Get From GUI user input
    double tw = -0.1808;    // Get From GUI user input
};

struct RobotPara
{
    double l1 = 0.068;
    double l21 = 0.430;
    double l22 = 0.446;
    double l31 = 0.100;
    double l32 = 0.430;
    double l4 = 0.071+0.04;
    double l5 = 0.0895;
    double d5 = 0.04; // the height from 5th link to the bottom of forearm
    double l6 = 0.14; // distance of contact point to elbow
    double l7; // forearm length
};

struct Human2robot
{
    
    // the robot base position under shoulder frame //
    double xsr = 0;         // Get From GUI user input 
    double ysr = 0;         // Get From GUI user input
    double zsr = 0;         // Get From GUI user input
    // the robot base position under pelvis frame //
    double xpr = 0;         // Get From GUI user input
    double ypr = 0;         // Get From GUI user input
    double zpr = 0;         // Get From GUI user input
};

struct Huamn_angle{
    double h1;
    double h2;
    double h3;
    double h4;
};


class ARAEarm{
    // public:
    // ARAEarm(HumanPara h_para, Human2robot h_r_para, RobotPara r_para);
    // ~ARAEarm() = default;

    private:
        HumanPara h_para_;
        RobotPara r_para_;
        Human2robot hr_para_;
        Vector3f p_shoulder_robot_; // the distance of shoulder refer to robot in shoulder frame
        Matrix3f r_shoulder_robot_;
        Huamn_angle ang_k;
        MatrixXf Jocb_r_ = MatrixXf(3,3);
        MatrixXf Jocab_a_ = MatrixXf(4,3);
        // Human and robot parameters
        void UpdateHumanArmProperties(HumanPara h_para_, double &mU, double &mL);
    public:
        void UpdateParams(HumanPara &h_para, Human2robot &h_r_para, RobotPara &r_para);
        Matrix4d T_robot(double theta, double alpha, double a, double d);
        static Matrix4d TM(double xhr, double yhr, double zhr);
        /** calculate the  **/
        void Robot_FK(double *A, double x, double y, double z, Vector3d &P_c, Vector3d &P_e, Vector3d &P_w);
        void Robot_IK(double x, double y, double z, double &q1, double &q21, double &q31);
        Matrix<double, 2, Dynamic> circcirc(double x1, double y1, double r1, double x2, double y2, double r2);
        void SagittalPlane(Vector3d Pp_e, Vector3d Pp_w, Vector3d &P_s, Vector3d &Ps_e, Vector3d &Ps_w);
        // void Arm_FK(const float ang_k[4], Vector3f &p_e, Vector3f &p_w);
        void Arm_IK(const Vector3d &Ps_e, const Vector3d &Ps_w, double &U_cal, Huamn_angle &ang_k);
        void Update_Robot_Jocabian();
        void Update_Arm_Jocabian();
        void CalFh_rigid(Huamn_angle ang_k, HumanPara h_para_, const double &U_cal, Vector3d &Ps_c, Vector3d &f1);
        void CalFh_armdynamics(Huamn_angle ang_k, HumanPara h_para_, const double &U_cal, Vector3d &f2);
        Vector3d RobotTorque(double *P, int cA, const Eigen::Vector3d &end_f, double load);
        // Vector3f RobotForce2Torque(const float ang_q[5][3], Vector3f f, float load);

};



// struct HumanPara
// {
//     bool gender;
//     double wt;
//     double ul;
//     double fl;
//     double th;
//     double tw;
// };

// struct RobotPara
// {
//     double l1 = 0.068;
//     double l21 = 0.430;
//     double l22 = 0.446;
//     double l31 = 0.100;
//     double l32 = 0.430;
//     double l4 = 0.071+0.04;
//     double l5 = 0.0895;
//     double l6 = 0.14; // distance of contact point to elbow
//     double l7; // forearm length
// };

// class ARAEarm{
//     public:
//     ARAEarm(HumanPara para, Vector3f p_sr);
//     ~ARAEarm() = default;

//     private:
//         HumanPara h_para_;
//         RobotPara r_para_;
//         Vector3f p_shoulder_robot_; // the distance of shoulder refer to robot in shoulder frame
//         Matrix3f r_shoulder_robot_;

//         MatrixXf Jocb_r_ = MatrixXf(3,3);
//         MatrixXf Jocab_a_ = MatrixXf(4,3);

//         // Human and robot parameters

//         void UpdateHumanArmProperties();

//     public:
//         void Robot_FK(const float ang_q[5], Vector3f &p_c, Vector3f &p_e, Vector3f &p_w);

//         void Arm_FK(const float ang_k[4], Vector3f &p_e, Vector3f &p_w);

//         void Arm_IK(const Vector3f &p_s, const Vector3f &p_e, const Vector3f &p_w, float ang_k[4]);
        
//         void Update_Robot_Jocabian();

//         void Update_Arm_Jocabian();

//         void GetShoulderPosition(const float ang_q[5], Vector3f &est_sp);

//         void CalEndeffectorForce(const float ang_k[4], Vector3f &f);

//         Vector3f RobotDynamic(const float ang_q[5][3], bool is_only_grav);

//         Vector3f RobotForce2Torque(const float ang_q[5][3], Vector3f f, float load);

// };

}

#endif
