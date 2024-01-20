#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <fstream>

#include "robotmodel.h"
#include "trajectory.h"
#include "custommath.h"
#include "quadraticprogram.h"
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

class CController
{

public:
    CController();
    virtual ~CController();	

    void read(double time, double* q, double* qdot, double* sensordata);
    void read(double time, double* q, double* qdot, double* sensordata, double* qacc);
    void control_mujoco();
    void write(double* torque);

    void addExternal_ForceTorque(const char** body_name, double* torque, double* time);

private:
    void Initialize();
    void ModelUpdate();
    void motionPlan();

    void reset_target(double motion_time, VectorXd target_joint_position);
    void reset_target(double motion_time, VectorXd target_joint_position, VectorXd target_joint_velocity);
    void reset_target(double motion_time, Vector3d target_pos, Vector3d target_ori);
    bool ParticleYamlRead(std::string path, Eigen::VectorXd& unit); // for yaml
    VectorXd LinearInterpolation(VectorXd init_pos, VectorXd goal_pos, double init_time, double goal_time, double cur_time);
    VectorXd _q; // joint angle
	VectorXd _qdot; // joint velocity
    VectorXd _torque; // joint torque

    int dof; // DOF = 7

    bool _bool_init;
    double _t;
    double _dt;
	double _init_t;
	double _pre_t;

    //controller
	double _kpj, _kdj; //joint P,D gain
    double _x_kp; // task control P gain

    void JointControl();
    void CLIK();

    // robotmodel
    CModel Model;

    int _cnt_plan;
	VectorXd _time_plan;
	VectorXi _bool_plan;

    int _control_mode; //1: joint space, 2: operational space
    VectorXd _q_home; // joint home position

    //motion trajectory
	double _start_time, _end_time, _motion_time;

    CTrajectory JointTrajectory; // joint space trajectory
    // HTrajectory HandTrajectory; // task space trajectory
    CTrajectory HandTrajectory; // task space trajectory

    CTrajectory MpcTrajectory;
    bool _bool_joint_motion, _bool_ee_motion; // motion check

    VectorXd _q_des, _qdot_des; 
    VectorXd _q_goal, _qdot_goal;
    VectorXd _x_des_hand, _xdot_des_hand;
    VectorXd _x_goal_hand, _xdot_goal_hand;
    Vector3d _pos_goal_hand, _rpy_goal_hand;

    MatrixXd _A_diagonal; // diagonal inertia matrix
    MatrixXd _J_hands; // jacobian matrix 6x7
    MatrixXd _J_bar_hands; // pseudo inverse jacobian matrix  7x6

    VectorXd _x_hand, _xdot_hand; // End-effector


    VectorXd _x_err_hand;
    Eigen::Matrix<double, 3, 3> _R_des_hand;

    // MatrixXd _I; // Identity matrix 7x7
    Eigen::Matrix<double, 6, 6> _Id_6;
    Eigen::Matrix<double, 7, 7> _Id_7;
    Eigen::Matrix<double, 12, 12> _Id_12;
    Eigen::Matrix<double, 14, 14> _Id_14;
    Eigen::Matrix<double, 18, 18> _Id_18;
    void OperationalSpaceControl();
    Eigen::Matrix<double, 6, 6> _lambda;
    Eigen::Matrix<double, 3, 3> _Rdot_des_hand;
    Eigen::Matrix<double, 3, 3> _Rdot_hand;
    Eigen::Matrix<double, 7, 7> _Null_hands;

    Eigen::Matrix<double, 7, 6> _J_T_hands;
    Eigen::Matrix<double, 6, 7> _J_bar_T_hands;
    Eigen::Matrix<double, 6, 1> _Jdot_qdot;
    Eigen::Matrix<double, 6, 7> _Jdot_hands;
    Eigen::Matrix<double, 6, 7> _pre_J_hands;
    Eigen::Matrix<double, 6, 7> _pre_Jdot_hands;
    Eigen::Matrix<double, 6, 1> _xddot_ref;
    Eigen::Matrix<double, 6, 1> _pre_xdot;

    VectorXd _xdot_err_hand;

    void MPC();
    // void MPC(VectorXd _q_goal);
    void Initialize_MPC();
    int task_dof; // task DOF = 6
    double Np; // time horizon
    double dT; // sampling time for horzon
    double threshold;
    VectorXd X;      // q,qdot 14x1
    VectorXd X_next;      // q,qdot 14x1
	
	MatrixXd A_d;    // 14x14
	MatrixXd B_d;    // 14x7
    MatrixXd C_d;    // 7x14
	VectorXd D_d;    // 14x1

    VectorXd temp_V;    
    VectorXd V;
    MatrixXd temp_F;
    MatrixXd F;
    MatrixXd temp_phi;
    MatrixXd temp_phi_B;
    MatrixXd Phi;
    MatrixXd Q_temp;
    MatrixXd Q_temp_last;
    MatrixXd Q;
    MatrixXd R;

    VectorXd Y_k;
    VectorXd Y_ref;
    VectorXd Y_c;

    CQuadraticProgram qp;
    int max_iter;
    MatrixXd _H1;
    VectorXd _g1;
    MatrixXd _A1;
    VectorXd _ubA1;
    VectorXd _lbA1;

    VectorXd _lb1;
    VectorXd _ub1;
    bool boolll;
    VectorXd U_old;
    VectorXd del_u;
    VectorXd U;
    MatrixXd diag_A;
    
    bool a;
    bool end;
    int freq_check;

    fstream fout;
    VectorXd pred_q;
    VectorXd prev_q;
    VectorXd pred_qdot;
    VectorXd log;

    VectorXd pred_X;
    VectorXd pred_Xdot;
    bool first;
    VectorXd _Cq_old;
    VectorXd _Cqdot_des;
    VectorXd _Cq;
    VectorXd Check;
    VectorXd Y_des;
    VectorXd Ydot_des;
    VectorXd Yddot_des;
    VectorXd F_des;
    VectorXd F_r;
    VectorXd Ydes;

    VectorXd C_p;
    VectorXd C_m;
    VectorXd temp_C_p;
    VectorXd temp_C_m;

    VectorXd constraint;
    VectorXd constraint_p;
    VectorXd constraint_m;
    Eigen::Matrix<double, 6,1> FTdata;
    Eigen::Matrix<double, 6,1> _FTdata;
    Eigen::Matrix<double, 6,6> Kd;
    Eigen::Matrix<double, 6,6> Dd;
    Eigen::Matrix<double, 6,6> Md;
    double _tM;
    VectorXd _xddot_star;

    VectorXd Y_des_q;
    VectorXd Ydot_des_qdot;

    VectorXd C_V;    // 7x14

    Eigen::Matrix<double, 6, 1> _xdot_ref;
    Eigen::Matrix<double, 3, 3> _pre_Rdot_ref;

    Eigen::Matrix<double, 3, 1> goal;
    Eigen::Matrix<double, 7, 1> _qacc; 

    Eigen::Matrix<double, 6, 1> _xacc;
    Eigen::Matrix<double, 6, 1> _xacc_goal;
    Eigen::Matrix<double, 6, 1> pre_xdot_hand;

    Eigen::Matrix<double, 6, 1> _xacc_des;
    Eigen::Matrix<double, 6, 1> _pre_xacc;

    bool ftdata_first;
    Eigen::Matrix<double, 6, 1> FT_first;
    Eigen::Matrix<double, 7, 1> _torque_null;
    Eigen::Matrix<double, 7, 1> _torque_friction;
    VectorXd tanh(VectorXd _qdot);


    Eigen::Matrix<double, 7, 1> _p;
    bool isrun;
    Eigen::Matrix<double, 7, 1> _p0;
    Eigen::Matrix<double, 7, 1> _p_hat;
    Eigen::Matrix<double, 7, 1> _p_dot_hat;

    Eigen::Matrix<double, 7, 1> _Beta;
    Eigen::Matrix<double, 7, 1> _torque_observer;
    Eigen::Matrix<double, 7, 7> K0;

    Eigen::Matrix<double, 7, 1> _CT_qdot;
    Eigen::Matrix<double, 7, 7> M_dot;
    Eigen::Matrix<double, 7, 7> M_prev;

    bool taskfirst;

    Eigen::Matrix<double, 6, 1> _xddot_des_hand;
    Eigen::Matrix<double, 6, 1> _xddot_goal_hand;

    Eigen::Matrix<double, 6, 1> _pre_x_goal_hand;

    Eigen::Matrix<double, 6, 1> _pre_FT;

    // MPIC .. //
    MatrixXd K;
    MatrixXd KK;

    MatrixXd R_;
    MatrixXd _Id_np;


    // addExternal_ForceTorque //
    char _body_name[10];
    Eigen::Matrix<double, 6, 1> _force_e;
    Eigen::Matrix<double, 3, 1> _force_s;
    Eigen::Matrix<double, 3, 1> _torque_s;
    Eigen::Matrix<double, 6, 1> _FTdata1;
    Eigen::Matrix<double, 6, 1> _FTdata2;
    Eigen::Matrix<double, 6, 1> _FTdata3;
    Eigen::Matrix<double, 6, 1> _FTdata4;
    Eigen::Matrix<double, 6, 1> _FTdata5;
    Eigen::Matrix<double, 6, 1> _FTdata6;
    Eigen::Matrix<double, 6, 1> _FTdata7;
    double _time_e;

    Eigen::Matrix<double, 3, 3> _rotxx;

    double _roll;
    double _pitch;
    double _yaw;

    VectorXd min_;
    VectorXd max_;

    Eigen::Matrix<double, 7, 1> _q_min;
    Eigen::Matrix<double, 7, 1> _q_max;
    Eigen::Matrix<double, 7, 1> _qdot_min;
    Eigen::Matrix<double, 7, 1> _qdot_max;
    Eigen::Matrix<double, 14, 1> _joint_min;
    Eigen::Matrix<double, 14, 1> _joint_max;
    VectorXd min_constraint;
    VectorXd max_constraint;
    VectorXd _eq_min_constraint;
    VectorXd _eq_max_constraint;

    Eigen::Matrix<double, 7, 1> _opt_u; // optimal u
    Eigen::Matrix<double, 7, 1> _qdes_inter; // optimal u
    Eigen::Matrix<double, 7, 1> _qdotdes_inter; // optimal u
    double _mpc_init_t;

    Eigen::Matrix<double, 7, 1> _prev_q;
    Eigen::Matrix<double, 7, 1> _pred_q;

};

#endif
