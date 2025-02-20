/*
Copyright (c) 2023, Rehabilitation Research Institute of Singapore
Author: Sibo Yang
*/
#include "ARAE_CORE/ARAE.hpp"

using namespace ARAE;
using namespace Eigen;


void ARAEarm::UpdateParams(HumanPara &h_para, Human2robot &h_r_para, RobotPara &r_para){
    h_para_ = h_para;
    r_para_ = r_para;
    hr_para_ = h_r_para;
}

Matrix4d ARAEarm::T_robot(double theta, double alpha, double a, double d) {
    Matrix4d T;

    // Define the elements of the transformation matrix
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;

    return T;
}

Matrix4d ARAEarm::TM(double xhr, double yhr, double zhr) {
    Matrix4d T_hr;

    T_hr << 0, 1, 0, xhr,
           -1, 0, 0, yhr,
            0, 0, 1, zhr,
            0, 0, 0, 1;

    return T_hr;
}
void ARAEarm::UpdateHumanArmProperties(HumanPara h_para_, double &mU, double &mL, double scale){
    if(h_para_.gender){ //male
        if (h_para_.wt < 10.0) {
            std::cout << "Personlized Mode"  << std::endl;
            mU = h_para_.wt;
            mL = scale;
        }
        else {
            std::cout << "Normal Mode"  << std::endl;
            mU = h_para_.ws * h_para_.wt * 3.25/100.0;
            mL = h_para_.ws * h_para_.wt * 1.87/100.0;
        }
    }
    else { //female
        if (h_para_.wt < 10.0) {
            mU = h_para_.wt;
            mL = scale;
        }
        else {
            mU = h_para_.ws * h_para_.wt * 2.9/100.0;
            mL = h_para_.ws * h_para_.wt * 1.57/100.0;
        }
    }
}

void ARAEarm::Robot_FK(double *A, double x, double y, double z, Vector3d &P_c, Vector3d &P_e, Vector3d &P_w)
{
   const double c_pi = 3.141592653589793115997963468544185161590576171875; // double PI
   double L = h_para_.fl; // forearm length
   double q1, q_dot_1, q_ddot_1, q21, q_dot_21, q_ddot_21, q31, q_dot_31, q_ddot_31, q2_1,q22,q32,q4,q5;
   int i=0;
   q1 = *(A+i);
   i++;
   q_dot_1 = *(A+i);
   i++;
   q_ddot_1 = *(A+i);
   i++;
   q21 = *(A+i);
   i++;
   q_dot_21 = *(A+i);
   i++;
   q_ddot_21 = *(A+i);
   i++;
   q31 = *(A+i);
   i++;
   q_dot_31 = *(A+i); 
   i++;
   q_ddot_31 = *(A+i);
   i++;
   q4 = *(A+i);
   i++;
   q5 = *(A+i);
   i =0;
   std::cout << "q1: " << q1 << std::endl;
   std::cout << "q21: " << q21 << std::endl;
   std::cout << "q31: " << q31 << std::endl;
   std::cout << "q4: " << q4 << std::endl;
   std::cout << "q5: " << q5 << std::endl;
   q2_1 =  c_pi/2 - q21;
   q22 = q31 - q2_1 + c_pi;
   q32=2*c_pi - (q31 - q2_1);
   // DH parameters
   double alpha_1 = c_pi / 2;
   double alpha_21 = 0;
   double alpha_22 = 0;
   double alpha_3 = 0;
   double alpha_31 = 0;
   double alpha_32 = 0;
   double alpha_4 = -c_pi / 2;
   double alpha_5 = 0;
   double alpha_6 = 0;
   double alpha_7 = 0;
   double a1 = 0;
   double d1 = r_para_.l1;
   double a21 = r_para_.l21;
   double d21 = 0;
   double a3 = r_para_.l22 - r_para_.l31;
   double d3 = 0;
   double a4 = 0;
   double d4 = -r_para_.l4;
   double a5 = r_para_.l5;
   double d5 = r_para_.d5;
   double a6 = r_para_.l6;
   double d6 = 0;
   double a7 = r_para_.l7;
   double d7 = 0;
   double a22 = r_para_.l22;
   double d22 = 0;
   double a31 = r_para_.l31;
   double d31 = 0;
   double a32 = r_para_.l32;
   double d32 = 0;
   // int i=0;
   // q1 = *(A+i);
   // i++;
   // q21 = *(A+i);
   // i++;
   // q31 = *(A+i);
   // i++;
   // q4 = *(A+i);
   // i++;
   // q5 = *(A+i);


// Calculate transformation matrices
   Matrix4d H0_1 = T_robot(q1, alpha_1, a1, d1);
   Matrix4d H1_21 = T_robot(q2_1, alpha_21, a21, d21);
   Matrix4d H21_22 = T_robot(q22, alpha_22, a3, d3);
   Matrix4d H22_4 = T_robot(q4, alpha_4, a4, d4);
   Matrix4d H4_5 = T_robot(q5, alpha_5, a5, d5);
   Matrix4d H5_6 = T_robot(c_pi / 2, alpha_6, a6, d6);
   Matrix4d H6_7 = T_robot(c_pi, alpha_7, a7, d7);

   // Perform matrix multiplications to get the desired transformation matrices
   Matrix4d H0_2 = H0_1 * H1_21;
   Matrix4d H0_3 = H0_2 * H21_22;
   Matrix4d H0_4 = H0_3 * H22_4;
   Matrix4d H0_5 = H0_4 * H4_5;
   Matrix4d H0_6 = H0_5 * H5_6;
   Matrix4d H0_7 = H0_6 * H6_7;


// Position vector from robot base to the end-effector
   Matrix4d TM_hr = TM(x,y,z);
   Eigen::Vector4d P0_0;
      P0_0 << 0, 0, 0, 1;
   // calculate center position under human shoulder frame
   Eigen::Vector4d Ps_5 = TM_hr * (H0_5 * P0_0);
   Eigen::Vector3d Ps_c_ = Ps_5.head<3>();
   P_c = Ps_c_;
   // calculate elbow position under human shoulder frame
   Eigen::Vector4d Ps_6 = TM_hr * (H0_6 * P0_0);
   Eigen::Vector3d Ps_e_ = Ps_6.head<3>();
   P_e = Ps_e_;
   // calculate wrist position under human shoulder frame
   Eigen::Vector4d Ps_7 = TM_hr * (H0_7 * P0_0);
   Eigen::Vector3d Ps_w_ = Ps_7.head<3>(); 
   P_w = Ps_w_;
   std::cout << "Ps_c: " << P_c << std::endl;
   std::cout << "Ps_e: " << P_e << std::endl;
   std::cout << "Ps_w: " << P_w << std::endl;
}

void ARAEarm::Robot_IK(double x, double y, double z, double &q1, double &q21, double &q31) {
    double l1 = r_para_.l1;
    double l21 = r_para_.l21;
    double l22 = r_para_.l22;
    double l31 = r_para_.l31;
    double l32 = r_para_.l32;

    double l4 = l22 - l31;
    double wz = z - l1;
    double r = std::sqrt(x * x + y * y);
    double delta = wz * wz + r * r;

    // Find q3
    // Cosine rule
    double c3 = (l21 * l21 + l4 * l4 - delta) / (2 * l21 * l4);
    // Trig identity
    double s3 = std::sqrt(1 - c3 * c3);
    // 4-quadrant atan
    double q_22 = std::atan2(s3, c3);

    // Find q2_1
    // Cosine rule
    double c2 = (l21 * l21 + delta - l4 * l4) / (2 * l21 * std::sqrt(delta));
    // Trig identity
    double s2 = std::sqrt(1 - c2 * c2);
    // 4-quadrant atan
    double q2_1 = std::atan2(wz, r) + std::atan2(s2, c2);

    // Robot joint angles
    q1 = std::atan2(y, x);
    q21 = M_PI / 2 - q2_1;
    q31 = q2_1 + q_22;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> ARAEarm::circcirc(double x1, double y1, double r1, double x2, double y2, double r2) {
    Eigen::Matrix<double, 2, Eigen::Dynamic> intersectionPoints(2, 0);

    double dx = x2 - x1;
    double dy = y2 - y1;
    double d = std::sqrt(dx * dx + dy * dy);

    if (d > r1 + r2 || d < std::abs(r1 - r2)) {
        // Circles do not intersect
        std::cout << "Circles do not intersect"  << std::endl;
        double r1 = d -r2;
        std::cout << "r1: "<< r1 << std::endl;
        double x3 = x1 + (x2 - x1) * r1 / (r1 + r2);
        double y3 = y1 + (y2 - y1) * r1 / (r1 + r2);

        Vector2d intersection1(x3,y3);
        Vector2d intersection2(x3,y3);
        // double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        // double h = std::sqrt(r1 * r1 - a * a);
        // double x3 = x1 + a * (x2 - x1) / d;
        // double y3 = y1 + a * (y2 - y1) / d;

        // double offsetX = h * (y2 - y1) / d;
        // double offsetY = h * (x2 - x1) / d;

        // Eigen::Vector2d intersection1(x3 + offsetX, y3 - offsetY);
        // Eigen::Vector2d intersection2(x3 - offsetX, y3 + offsetY);

        intersectionPoints.conservativeResize(2, 2);
        intersectionPoints.col(0) = intersection1;
        intersectionPoints.col(1) = intersection2;
        
    } else {    
        double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        double h = std::sqrt(r1 * r1 - a * a);
        double x3 = x1 + a * (x2 - x1) / d;
        double y3 = y1 + a * (y2 - y1) / d;

        double offsetX = h * (y2 - y1) / d;
        double offsetY = h * (x2 - x1) / d;

        Eigen::Vector2d intersection1(x3 + offsetX, y3 - offsetY);
        Eigen::Vector2d intersection2(x3 - offsetX, y3 + offsetY);

        intersectionPoints.conservativeResize(2, 2);
        intersectionPoints.col(0) = intersection1;
        intersectionPoints.col(1) = intersection2;}

    return intersectionPoints;
}
void ARAEarm::SagittalPlane(Eigen::Vector3d Pp_e, Eigen::Vector3d Pp_w, Eigen::Vector3d &P_s,  Vector3d &Ps_e, Vector3d &Ps_w)
{
    double U = h_para_.ul;
    std::cout << "U: " << U << std::endl;
    double l_SH = h_para_.th;
    std::cout << "l_SH: " << l_SH << std::endl;
    double x_hp = h_para_.tw;
    double a = Pp_e.x();
    std::cout << "Pp_e.x(): " << a << std::endl;
    double b = Pp_e.y();
    double c = Pp_e.z();

    //###################################################################################################################################
    // Changed from here
    double Eproj_r = std::sqrt(U * U - std::pow(a + x_hp, 2)); // projected radius of elbow position in SH plane
    double l_H_Eproj = std::sqrt(b * b + c * c);
    double estimated_h2 = std::atan2(-b,(l_SH - c));
    std::cout << "Eproj_r: " << Eproj_r << std::endl;
    double cal_HS = c + Eproj_r * std::cos(estimated_h2);
    std::cout << "cal_th: " << cal_HS << std::endl;
    double predicted_y = 0.0;
    double predicted_z = l_SH;
    if (cal_HS >= l_SH) {
        Eigen::Matrix<double, 2, Eigen::Dynamic> intersections = circcirc(0, 0, l_SH, b, c, Eproj_r);
        
        double calculated_y_1 = intersections(0, 0);
        double calculated_z_1 = intersections(1, 0);
        double calculated_y_2 = intersections(0, 1);
        double calculated_z_2 = intersections(1, 1);
        double dist_1 = std::sqrt((calculated_y_1 - predicted_y) * (calculated_y_1 - predicted_y) + (calculated_z_1 - predicted_z) * (calculated_z_1 - predicted_z));
        double dist_2 = std::sqrt((calculated_y_2 - predicted_y) * (calculated_y_2 - predicted_y) + (calculated_z_2 - predicted_z) * (calculated_z_2 - predicted_z));

        if (dist_1 <= dist_2) {
            P_s.x() = -x_hp;
            P_s.y() = calculated_y_1;
            P_s.z() = calculated_z_1;
        }
        else {
            P_s.x() = -x_hp;
            P_s.y() = calculated_y_2;
            P_s.z() = calculated_z_2;
        }

    } else {
        double amend_Eproj_r = (l_SH - c) / std::cos(estimated_h2);
        std::cout << "amend l_SH: "<< amend_Eproj_r << std::endl;
        Eigen::Matrix<double, 2, Eigen::Dynamic> intersections = circcirc(0, 0, l_SH, b, c, amend_Eproj_r);

        double calculated_y_1 = intersections(0, 0);
        double calculated_z_1 = intersections(1, 0);
        double calculated_y_2 = intersections(0, 1);
        double calculated_z_2 = intersections(1, 1);
        double dist_1 = std::sqrt((calculated_y_1 - predicted_y) * (calculated_y_1 - predicted_y) + (calculated_z_1 - predicted_z) * (calculated_z_1 - predicted_z));
        double dist_2 = std::sqrt((calculated_y_2 - predicted_y) * (calculated_y_2 - predicted_y) + (calculated_z_2 - predicted_z) * (calculated_z_2 - predicted_z));

        if (dist_1 <= dist_2) {
            P_s.x() = -x_hp;
            P_s.y() = calculated_y_1;
            P_s.z() = calculated_z_1;
        }
        else {
            P_s.x() = -x_hp;
            P_s.y() = calculated_y_2;
            P_s.z() = calculated_z_2;
        }
    }
    //###################################################################################################################################




    
    Matrix4d T_tls;
    T_tls << 1, 0, 0, -P_s(0),
            0, 1, 0, -P_s(1),
            0, 0, 1, -P_s(2),
            0, 0, 0, 1;
    Vector4d Pp_e_, Pp_w_;
    Pp_e_ << Pp_e, 1;
    Pp_w_ << Pp_w, 1;
    VectorXd Ps_e_ = T_tls * Pp_e_;
    VectorXd Ps_w_ = T_tls * Pp_w_;
    Ps_e = Ps_e_.head(3);
    Ps_w = Ps_w_.head(3);

}
void ARAEarm::Arm_IK(const Vector3d &Ps_e, const Vector3d &Ps_w, double &U_cal, Huamn_angle &ang_k) {
    // Parameters
    const double c_pi = 3.141592653589793115997963468544185161590576171875;
    Vector3d P_s(0, 0, 0);
    U_cal = Ps_e.norm();
    double L = h_para_.fl;
    std::cout << "L: " << L << std::endl;
    double Psw = Ps_w.norm();
    // Solve for h4
    // ang_k.h4 = c_pi / 2 - std::acos((L * L + U_cal * U_cal - (Ps_w - P_s).squaredNorm()) / (2 * L * U_cal));
    ang_k.h4 = c_pi / 2 - std::acos((L * L + U_cal * U_cal - Psw * Psw) / (2 * L * U_cal));
    // shoulder flexion ----X axis
    ang_k.h2 = std::asin(-Ps_e(2) / U_cal);

    // Z axis shoulder abduction
    ang_k.h1 = std::atan2(Ps_e(0) / std::cos(ang_k.h2), -Ps_e(1) / std::cos(ang_k.h2));

    double vf = Ps_w(0) * std::cos(ang_k.h1) + Ps_w(1) * std::sin(ang_k.h1);

    // shoulder rotation
    ang_k.h3 = std::atan2(-(Ps_w(0) * std::sin(ang_k.h1) * std::sin(ang_k.h2) - Ps_w(1) * std::cos(ang_k.h1) * std::sin(ang_k.h2) + Ps_w(2) * std::cos(ang_k.h2)) / (L * std::cos(ang_k.h4)), vf / (L * std::cos(ang_k.h4)));
    std::cout << "h1: " << ang_k.h1 << std::endl;
    std::cout << "h2: " << ang_k.h2 << std::endl;
    std::cout << "h3: " << ang_k.h3 << std::endl;
    std::cout << "h4: " << ang_k.h4 << std::endl;
    std::cout << "U_cal: " << U_cal << std::endl; 
}
void ARAEarm::CalFh_rigid(Huamn_angle ang_k, HumanPara h_para_, const double &U_cal, Vector3d &Ps_c, Vector3d &f1){
    double mul,mfl;
    ARAEarm::UpdateHumanArmProperties(h_para_, mul, mfl, 1.00);
    std::cout << "mUl: " << mul << std::endl;
    std::cout << "mfl: " << mfl << std::endl;
    double k1 = ang_k.h1;
    double k2 = ang_k.h2;
    double k3 = ang_k.h3;
    double k4 = ang_k.h4;
    // Transformation Matrix
    Matrix4d Ts_1, T1_2, T2_3, T2_3g, T3_4g;

    Ts_1 << cos(k1), -sin(k1), 0, 0,
            sin(k1), cos(k1), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    // std::cout << "Ts_1: " << Ts_1 << std::endl;
    T1_2 << 1, 0, 0, 0,
            0, cos(k2), -sin(k2), 0,
            0, sin(k2), cos(k2), 0,
            0, 0, 0, 1;
    // std::cout << "T1_2: " << T1_2 << std::endl;
    T2_3 << cos(k3), 0, sin(k3), 0,
            0, 1, 0, -U_cal,
            -sin(k3), 0, cos(k3), 0,
            0, 0, 0, 1;
    // std::cout << "T2_3: " << T2_3 << std::endl;
    T2_3g << cos(k3), 0, sin(k3), 0,
             0, 1, 0, -h_para_.Ugc * U_cal,
             -sin(k3), 0, cos(k3), 0,
             0, 0, 0, 1;
    // std::cout << "T2_3g: " << T2_3g << std::endl;
    T3_4g << cos(k4), -sin(k4), 0, cos(k4) * h_para_.Lgc * h_para_.fl,
             sin(k4), cos(k4), 0, sin(k4) * h_para_.Lgc * h_para_.fl,
             0, 0, 1, 0,
             0, 0, 0, 1;
    // std::cout << "T3_4g: " << T3_4g << std::endl;   
    // Calculate Ts_3
    Matrix4d Ts_3 = Ts_1 * T1_2 * T2_3;

    // Calculate Ts_3g
    Matrix4d Ts_3g = Ts_1 * T1_2 * T2_3g;

    // Calculate Ts_4g
    Matrix4d Ts_4g = Ts_3 * T3_4g;

    // Define Ps_s as Eigen::Vector4d
    Vector4d Ps_s;
    Ps_s << 0, 0, 0, 1;

    // Calculate Ps_3g
    Vector4d Ps_3g = Ts_3g * Ps_s;
    // std::cout << "Ps_3g: " << Ps_3g << std::endl;
    // Calculate Ps_4g
    Vector4d Ps_4g = Ts_4g * Ps_s;
    // std::cout << "Ps_4g: " << Ps_4g << std::endl;
    Vector3d G(0, 0, -9.81);

    // Calculate the torque due to mU and mL
    Vector3d t_s = (mul * (Ps_3g.head<3>())).cross(G) + (mfl * (Ps_4g.head<3>())).cross(G);

    // Calculate the desired force f1
    Vector3d f1_s;
    f1_s = -t_s.cross(Ps_c.head<3>()) / Ps_c.head(3).squaredNorm();
    
    Matrix3d T_s_r;
    T_s_r << 0, -1, 0,
             1, 0, 0,
             0, 0, 1;
    f1 = T_s_r * f1_s;
    std::cout << "F1: " << f1 << std::endl;
}
void ARAEarm::CalFh_armdynamics(Huamn_angle ang_k, HumanPara h_para_, const double &U_cal, Vector3d &f2, double load, double scale){
    double mU,mL;
    ARAEarm::UpdateHumanArmProperties(h_para_, mU, mL, scale);
    
    double Lr_f = h_para_.fl_c;
    double k1 = ang_k.h1;
    double k2 = ang_k.h2;
    double k3 = ang_k.h3;
    double k4 = ang_k.h4;
    double U = U_cal;
    double L = h_para_.fl;
    double Lgf, Ugf;
    if (h_para_.wt < 10.0) {
        std::cout << "Personlized" << std::endl;
        Lgf = load;
        Ugf = 1.0;
    }
    else {
        std::cout << "Normal" << std::endl;
        Lgf = h_para_.Lgc;
        Ugf = h_para_.Ugc;
    }
    // Human arm jacobian
    // Limit elbow angle to avoid Singularity of Jacobian matrix
    if(k4 > 80.0/180.0*M_PI)
    {
        k4 = 80.0/180.0*M_PI;
    }
    else if(k4 < -80.0/180.0*M_PI)
    {
        k4 = -80.0/180.0*M_PI;
    }
    //std::cout<<"Before J_h.\n";
    double J_h[3][4] = {{U*cos(k1)*cos(k2) - Lr_f*cos(k4)*(cos(k3)*sin(k1) + cos(k1)*sin(k2)*sin(k3)) - Lr_f*cos(k1)*cos(k2)*sin(k4), Lr_f*sin(k1)*sin(k2)*sin(k4) - U*sin(k1)*sin(k2) - Lr_f*cos(k2)*cos(k4)*sin(k1)*sin(k3), -Lr_f*cos(k4)*(cos(k1)*sin(k3) + cos(k3)*sin(k1)*sin(k2)), - Lr_f*sin(k4)*(cos(k1)*cos(k3) - sin(k1)*sin(k2)*sin(k3)) - Lr_f*cos(k2)*cos(k4)*sin(k1)}, 
    {U*cos(k2)*sin(k1) + Lr_f*cos(k4)*(cos(k1)*cos(k3) - sin(k1)*sin(k2)*sin(k3)) - Lr_f*cos(k2)*sin(k1)*sin(k4), U*cos(k1)*sin(k2) - Lr_f*cos(k1)*sin(k2)*sin(k4) + Lr_f*cos(k1)*cos(k2)*cos(k4)*sin(k3), -Lr_f*cos(k4)*(sin(k1)*sin(k3) - cos(k1)*cos(k3)*sin(k2)), Lr_f*cos(k1)*cos(k2)*cos(k4) - Lr_f*sin(k4)*(cos(k3)*sin(k1) + cos(k1)*sin(k2)*sin(k3))},
    {0, Lr_f*cos(k2)*sin(k4) - U*cos(k2) + Lr_f*cos(k4)*sin(k2)*sin(k3), -Lr_f*cos(k2)*cos(k3)*cos(k4), Lr_f*cos(k4)*sin(k2) + Lr_f*cos(k2)*sin(k3)*sin(k4)}};

    //std::cout<<"After J_h.\n";
    MatrixXd J_hm(3,4);
    for(int i=0;i<3;i++){
        for(int j=0;j<4;j++){
            J_hm(i,j) = J_h[i][j];
        }
    }
    double t1g,t2g,t3g,t4g,g;
    g = 9.81;
    t1g = 0;
    t2g = (mL*(L*Lgf*cos(k2)*sin(k4) - U*cos(k2) + L*Lgf*cos(k4)*sin(k2)*sin(k3)) - U*Ugf*mU*cos(k2))*g;
    t3g = (-L*Lgf*mL*cos(k2)*cos(k3)*cos(k4))*g;
    t4g = (mL*(L*Lgf*cos(k4)*sin(k2) + L*Lgf*cos(k2)*sin(k3)*sin(k4)))*g;
    Vector4d tgm;
    tgm(0) = t1g;
    tgm(1) = t2g;
    tgm(2) = t3g;
    tgm(3) = t4g;
    
    //std::cout<<"J_hm= "<<J_hm<<std::endl;
    // std::cout<<"tgm= "<<tgm<<std::endl;
    MatrixXd J_hmT = J_hm.transpose();
    // std::cout << "J_hmT: " << J_hmT << std::endl;
    MatrixXd pv_J_h_tmp = J_hm * J_hmT;
    MatrixXd pv_J_h = pv_J_h_tmp.inverse();
    // std::cout << "pv_J_h: " << pv_J_h << std::endl;
    //std::cout<<"pv_J_h= "<<pv_J_h<<std::endl;
    Matrix3d T_s_r;
    T_s_r << 0, -1, 0,
             1, 0, 0,
             0, 0, 1;
    // std::cout << "T_s_r: " << T_s_r << std::endl;
    MatrixXd pv_J_h_ = pv_J_h * J_hm;
    // std::cout << "pv_J_h_: " << pv_J_h_ << std::endl;
    Vector3d F_r2_s = pv_J_h_ * tgm;
    // std::cout << "F_r2_s: " << F_r2_s << std::endl;
    // Avoiding pulling back force
    if(F_r2_s(1)>0)
        F_r2_s(1) = 0;
    Vector3d F_r2 = T_s_r * F_r2_s;
    f2 = F_r2;
    std::cout << "F2: " << f2 << std::endl;
}
Vector3d ARAEarm::RobotTorque(double *P, int cA, const Eigen::Vector3d &end_f, double load)
{
    const double c_pi = 3.141592653589793115997963468544185161590576171875; // double PI
    // const float c_pi = 3.1415927410125732421875
    // declare fixed values
    int i;
    double q1, q_dot_1, q_ddot_1, q21, q_dot_21, q_ddot_21, q31, q_dot_31, q_ddot_31;

    Vector3d Tr(0,0,0);

    i=0;

    q1 = *(P+i);

    i++;

    q_dot_1 = *(P+i);

    i++;

    q_ddot_1 = *(P+i);

    i++;

    q21 = *(P+i);

    i++;

    q_dot_21 = *(P+i);

    i++;

    q_ddot_21 = *(P+i);

    i++;

    q31 = *(P+i);

    i++;

    q_dot_31 = *(P+i);

    i++;

    q_ddot_31 = *(P+i);

    i=0;

   

    // std::cout<<"Reading P array\n";

    // std::cout<<std::setprecision(7)<<"q1 = "<<q1<<"\t";

    // std::cout<<std::setprecision(7)<<"q_dot_1 = "<<q_dot_1<<"\t";

    // std::cout<<std::setprecision(7)<<"q_ddot_1 = "<<q_ddot_1<<std::endl;

    // std::cout<<std::setprecision(7)<<"q21 = "<<q21<<"\t";

    // std::cout<<std::setprecision(7)<<"q_dot_21 = "<<q_dot_21<<"\t";

    // std::cout<<std::setprecision(7)<<"q_ddot_21 = "<<q_ddot_21<<std::endl;

    // std::cout<<std::setprecision(7)<<"q31 = "<<q31<<"\t";

    // std::cout<<std::setprecision(7)<<"q_dot_31 = "<<q_dot_31<<"\t";

    // std::cout<<std::setprecision(7)<<"q_ddot_31 = "<<q_ddot_31<<std::endl;

 

    // declare fixed values

    double l1, l21, l22, l31, l32, l4;

    double x1, x21,y21,x22,x31,x32,y22,lg22,r22,lg21,r21;

    l1 = r_para_.l1;

    l21 = r_para_.l21;

    l22 = r_para_.l22;

    l31 = r_para_.l31;

    l32 = r_para_.l32;

    x1 = 0.05432;

    x21 = 0.21431;

    y21 = 0.01885;

    lg21 = sqrt(pow(x21, 2) + pow(y21, 2));

    r21 = atan2(y21, x21);

    x22 = 0.33271;

    y22 = 0.0;

    lg22 = sqrt(pow(x22, 2) + pow(y22, 2));

    r22 = atan2(y22, x22);

    x31 = 0.04632;

    x32 = 0.215;

 

    double m1, m21, m22, m31, m32, g, wl4;

    m1 = 1.51806;

    m21 = 0.25668;

    m22 = 0.55976;

    m31 = 0.09410;

    m32 = 0.14479;

    g = 9.81;

    wl4 = 0.6774;

    double lm_x = end_f(0);

    double lm_y = end_f(1);

    double lm_z = end_f(2);

    // double F[3][1] = {{lm_x}, {lm_y}, {wl4*g+lm_z+load*g}};

    Eigen::Vector3d F;

    if (h_para_.wt < 10.0) {
        F << lm_x, lm_y, (wl4 * g + lm_z);
    }
    else {
        F << lm_x, lm_y, (wl4 * g + lm_z + load * g);
    }
    
    // std::cout << "F" << F << std::endl;
    Eigen::Matrix3d J;

    J(0, 0) = -l21 * cos(q21 - c_pi/2) * sin(q1) - cos(q21 + q31 + c_pi/2) * cos(q21 - c_pi/2) * sin(q1) * (l22 - l31) - sin(q21 + q31 + c_pi/2) * sin(q1) * sin(q21 - c_pi/2) * (l22 - l31);

    J(0, 1) = -l21 * cos(q1) * sin(q21 - c_pi/2);

    J(0, 2) = cos(q21 + q31 + c_pi/2) * cos(q1) * sin(q21 - c_pi/2) * (l22 - l31) - sin(q21 + q31 + c_pi/2) * cos(q1) * cos(q21 - c_pi/2) * (l22 - l31);

 

    J(1, 0) = l21 * cos(q1) * cos(q21 - c_pi/2) + cos(q21 + q31 + c_pi/2) * cos(q1) * cos(q21 - c_pi/2) * (l22 - l31) + sin(q21 + q31 + c_pi/2) * cos(q1) * sin(q21 - c_pi/2) * (l22 - l31);

    J(1, 1) = -l21 * sin(q1) * sin(q21 - c_pi/2);

    J(1, 2) = cos(q21 + q31 + c_pi/2) * sin(q1) * sin(q21 - c_pi/2) * (l22 - l31) - sin(q21 + q31 + c_pi/2) * cos(q21 - c_pi/2) * sin(q1) * (l22 - l31);

 

    J(2, 0) = 0;

    J(2, 1) = -l21 * cos(q21 - c_pi/2);

    J(2, 2) = cos(q21 + q31 + c_pi/2) * cos(q21 - c_pi/2) * (l22 - l31) + sin(q21 + q31 + c_pi/2) * sin(q21 - c_pi/2) * (l22 - l31);

    // double J[3][3] ={

    //     {- l21*cos(q21 - c_pi/2)*sin(q1) - cos(q21 + q31 + c_pi/2)*cos(q21 - c_pi/2)*sin(q1)*(l22 - l31) - sin(q21 + q31 + c_pi/2)*sin(q1)*sin(q21 - c_pi/2)*(l22 - l31), -l21*cos(q1)*sin(q21 - c_pi/2), cos(q21 + q31 + c_pi/2)*cos(q1)*sin(q21 - c_pi/2)*(l22 - l31) - sin(q21 + q31 + c_pi/2)*cos(q1)*cos(q21 - c_pi/2)*(l22 - l31)},

    //     {l21*cos(q1)*cos(q21 - c_pi/2) + cos(q21 + q31 + c_pi/2)*cos(q1)*cos(q21 - c_pi/2)*(l22 - l31) + sin(q21 + q31 + c_pi/2)*cos(q1)*sin(q21 - c_pi/2)*(l22 - l31), -l21*sin(q1)*sin(q21 - c_pi/2), cos(q21 + q31 + c_pi/2)*sin(q1)*sin(q21 - c_pi/2)*(l22 - l31) - sin(q21 + q31 + c_pi/2)*cos(q21 - c_pi/2)*sin(q1)*(l22 - l31)},

    //     {0, -l21*cos(q21 - c_pi/2), cos(q21 + q31 + c_pi/2)*cos(q21 - c_pi/2)*(l22 - l31) + sin(q21 + q31 + c_pi/2)*sin(q21 - c_pi/2)*(l22 - l31)}

    //       };

    Vector3d JF = J.transpose() * F; 
    // std::cout << "JF" << JF << std::endl;
    // double JT[3][3];

    // err = transpose(J[0], JT[0], 3, 3);

    // double JF[3] = {0,0,0};

    // err = mulMat(JT,F,JF);

    // if(err){

    //     std::cout<<"Error in torque cal mulmat.\n";

    //     return Tr;

    // }

   double t1, t21, t31;

    //  std::cout<<"Torque required using ";

    // no Angular

    if (cA==0){

        std::cout<<"No Angular KE equation:\n";

        t1  = JF(0) + (m32*pow((l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)),2) + m32*pow((l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)),2) + m31*pow(x31,2)*pow(cos(q1),2)*pow(cos(q31),2) + m31*pow(x31,2)*pow(cos(q31),2)*pow(sin(q1),2) + pow(lg21,2)*m21*pow(cos(r21 - q21 + c_pi/2),2)*pow(cos(q1),2) + pow(lg21,2)*m22*pow(cos(r21 - q21 + c_pi/2),2)*pow(cos(q1),2) + pow(lg21,2)*m21*pow(cos(r21 - q21 + c_pi/2),2)*pow(sin(q1),2) + pow(lg21,2)*m22*pow(cos(r21 - q21 + c_pi/2),2)*pow(sin(q1),2))*q_ddot_1 \

        + (- q_dot_31*(2*l31*m32*cos(q1)*sin(q31)*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) + 2*m31*pow(x31,2)*pow(cos(q1),2)*cos(q31)*sin(q31) + 2*m31*pow(x31,2)*cos(q31)*pow(sin(q1),2)*sin(q31) + 2*l31*m32*sin(q1)*sin(q31)*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))) + q_dot_21*(2*m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31))*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) + 2*m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) + pow(lg21,2)*m21*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(cos(q1),2)*2 + pow(lg21,2)*m22*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(cos(q1),2)*2 + pow(lg21,2)*m21*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(sin(q1),2)*2 + pow(lg21,2)*m22*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(sin(q1),2)*2)*2)*q_dot_1 \

        + q_dot_31*(l31*m32*sin(q1)*sin(q31)*(q_dot_21*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31)) - l31*q_dot_31*cos(q1)*sin(q31)) - l31*m32*cos(q1)*sin(q31)*(q_dot_21*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) - l31*q_dot_31*sin(q1)*sin(q31)) - l31*m32*q_dot_31*cos(q31)*sin(q1)*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) + l31*m32*q_dot_31*cos(q1)*cos(q31)*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))) - 2*q_dot_21*(m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))*(q_dot_21*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31)) - l31*q_dot_31*cos(q1)*sin(q31)) - m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31))*(q_dot_21*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) - l31*q_dot_31*sin(q1)*sin(q31)) + m32*q_dot_21*(x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) - m32*q_dot_21*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31))*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))) - q_ddot_31*(l31*m32*sin(q1)*sin(q31)*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) - l31*m32*cos(q1)*sin(q31)*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))) + 2*q_ddot_21*(m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) - m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31))*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)));

        t21 = JF(1) + (2*m32*pow((x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31)),2) + 2*m32*pow((x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)),2) + 2*m32*pow((x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q31)),2) + pow(lg21,2)*m21*pow(cos(r21 - q21 + c_pi/2),2)*2 + pow(lg21,2)*m22*pow(cos(r21 - q21 + c_pi/2),2)*2 + pow(lg21,2)*m21*pow(sin(r21 - q21 + c_pi/2),2)*pow(sin(q1),2)*2 + pow(lg21,2)*m22*pow(sin(r21 - q21 + c_pi/2),2)*pow(sin(q1),2)*2 + pow(lg21,2)*m21*pow(sin(r21 - q21 + c_pi/2),2)*pow(cos(q1),2)*2 + pow(lg21,2)*m22*pow(sin(r21 - q21 + c_pi/2),2)*pow(cos(q1),2)*2)*q_ddot_21 \

        + (- 3*m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31))*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31)) - 3*m32*(x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) + 3*m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q31))*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)) + pow(lg21,2)*m21*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*3 + pow(lg21,2)*m22*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*3 - pow(lg21,2)*m21*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(cos(q1),2)*3 - pow(lg21,2)*m22*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(cos(q1),2)*3 - pow(lg21,2)*m21*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(sin(q1),2)*3 - pow(lg21,2)*m22*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(sin(q1),2)*3)*pow(q_dot_21,2) \

        + (m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31))*(q_dot_1*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) + l31*q_dot_31*cos(q1)*sin(q31)) - m32*(x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))*(q_dot_1*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) - l31*q_dot_31*sin(q1)*sin(q31)) - l31*m32*q_dot_31*cos(q31)*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)))*q_dot_21 \

        + (- m22*(l32*sin(q21 + q31 - (5*c_pi)/2)*sin(q31) + l32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)) - m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q31)) - lg21*m21*cos(r21 - q21 + c_pi/2))*g \

        - q_ddot_31*(l31*m32*cos(q31)*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q31)) + l31*m32*cos(q1)*sin(q31)*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31)) + l31*m32*sin(q1)*sin(q31)*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))) + q_ddot_1*(m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) - m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31))*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))) + q_dot_31*(m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31))*(l31*q_dot_1*sin(q1)*sin(q31) - l31*q_dot_31*cos(q1)*cos(q31)) - m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))*(l31*q_dot_1*cos(q1)*sin(q31) + l31*q_dot_31*cos(q31)*sin(q1)) + l31*m32*q_dot_31*sin(q31)*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q31))) - m32*q_dot_1*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31))*(q_dot_1*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) - l31*q_dot_31*sin(q1)*sin(q31)) - m32*q_dot_1*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))*(q_dot_1*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) + l31*q_dot_31*cos(q1)*sin(q31)) - pow(lg21,2)*m21*pow(q_dot_1,2)*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(cos(q1),2) - pow(lg21,2)*m22*pow(q_dot_1,2)*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(cos(q1),2) - pow(lg21,2)*m21*pow(q_dot_1,2)*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(sin(q1),2) - pow(lg21,2)*m22*pow(q_dot_1,2)*cos(r21 - q21 + c_pi/2)*sin(r21 - q21 + c_pi/2)*pow(sin(q1),2);

        t31 = JF(2) + (m32*pow(l31,2)*pow(cos(q1),2)*pow(sin(q31),2) + m32*pow(l31,2)*pow(cos(q31),2) + m32*pow(l31,2)*pow(sin(q1),2)*pow(sin(q31),2) + m31*pow(x31,2)*pow(cos(q1),2)*pow(sin(q31),2) + m31*pow(x31,2)*pow(cos(q31),2) + m31*pow(x31,2)*pow(sin(q1),2)*pow(sin(q31),2))*q_ddot_31 \

        + (m32*cos(q31)*sin(q31)*pow(l31,2)*pow(cos(q1),2) + m32*cos(q31)*sin(q31)*pow(l31,2)*pow(sin(q1),2) - m32*cos(q31)*sin(q31)*pow(l31,2) + m31*cos(q31)*sin(q31)*pow(x31,2)*pow(cos(q1),2) + m31*cos(q31)*sin(q31)*pow(x31,2)*pow(sin(q1),2) - m31*cos(q31)*sin(q31)*pow(x31,2))*pow(q_dot_31,2) \

        + (m22*(l31*cos(q31) + lg22*cos(q21 + q31 + r22 + c_pi/2)*(cos(q21 + q31 - (5*c_pi)/2)*cos(q31) + sin(q21 + q31 - (5*c_pi)/2)*sin(q31)) - lg22*sin(q21 + q31 + r22 + c_pi/2)*(cos(q21 + q31 - (5*c_pi)/2)*sin(q31) - sin(q21 + q31 - (5*c_pi)/2)*cos(q31))) + l31*m32*cos(q31) + m31*x31*cos(q31))*g \

        + 2*q_dot_21*(l31*m32*cos(q1)*sin(q31)*(q_dot_1*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) + q_dot_21*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31))) + l31*m32*sin(q1)*sin(q31)*(q_dot_21*(x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) - q_dot_1*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31))) - l31*m32*q_dot_21*cos(q31)*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31))) - 2*q_ddot_21*(l31*m32*cos(q31)*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q31)) + l31*m32*cos(q1)*sin(q31)*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31)) + l31*m32*sin(q1)*sin(q31)*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))) - q_ddot_1*(l31*m32*sin(q1)*sin(q31)*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31)) - l31*m32*cos(q1)*sin(q31)*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1))) + l31*m32*q_dot_1*cos(q1)*sin(q31)*(q_dot_21*(x32*cos(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) + q_dot_1*(l31*cos(q1)*cos(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31))) + m31*pow(q_dot_1,2)*pow(x31,2)*pow(cos(q1),2)*cos(q31)*sin(q31) + l31*m32*q_dot_1*sin(q1)*sin(q31)*(q_dot_1*(l31*cos(q31)*sin(q1) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q1)*sin(q31) + x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)*sin(q1)) - q_dot_21*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q1)*sin(q31) - x32*sin(q21 + q31 - (5*c_pi)/2)*cos(q1)*cos(q31))) + m31*pow(q_dot_1,2)*pow(x31,2)*cos(q31)*pow(sin(q1),2)*sin(q31);

 

           }

    // gravity only

    else if(cA == 1){

        // std::cout<<"Gravity only equation:\n";

        t1 = JF(0);

        t21 = JF(1) + (- m22*(l32*sin(q21 + q31 - (5*c_pi)/2)*sin(q31) + l32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31)) - m32*(x32*cos(q21 + q31 - (5*c_pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*c_pi)/2)*sin(q31)) - lg21*m21*cos(r21 - q21 + c_pi/2))*g;

        t31 = JF(2) + (m22*(l31*cos(q31) + lg22*cos(q21 + q31 + r22 + c_pi/2)*(cos(q21 + q31 - (5*c_pi)/2)*cos(q31) + sin(q21 + q31 - (5*c_pi)/2)*sin(q31)) - lg22*sin(q21 + q31 + r22 + c_pi/2)*(cos(q21 + q31 - (5*c_pi)/2)*sin(q31) - sin(q21 + q31 - (5*c_pi)/2)*cos(q31))) + l31*m32*cos(q31) + m31*x31*cos(q31))*g;

    }
    else {

        std::cout<<"Invalid cA. Exiting...\n";

        return Tr;

    }

    Tr(0) = t1;

    Tr(1) = t21;

    Tr(2) = t31;
    std::cout << "Tr: "<< Tr << std::endl;
    // double Tr[3] = {t1, t21, t31};

 

    // std::vector<double>tqcal;                      

    // for(int i=0; i<3;i++){

    //     tqcal.push_back(Tr[i]);

    //     if(tqcal[i] == Tr[i]);

    //     else {

    //         std::cout<<"Data transfer of Torque values fail.";

    //         return a;

    //     }

    // }

    // for (int i =0;i<tqcal.size();i++){

    //     std::cout<<tqcal[i]<<"\t";

    // }
    return Tr;

}


