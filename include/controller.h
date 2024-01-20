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
#include <future>

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
    HTrajectory _HandTrajectory; // task space trajectory
    CTrajectory HandTrajectory; // task space trajectory

    
    bool _bool_joint_motion, _bool_ee_motion; // motion check

    VectorXd _q_des, _qdot_des; 
    VectorXd _q_goal, _qdot_goal, _qddot_goal;
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
    Eigen::Matrix<double, 20, 20> _Id_20;
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

    fstream fout;
    VectorXd log;

    Eigen::Matrix<double, 6, 1> _xddot_star;

    Eigen::Matrix<double, 6, 1> _xdot_ref;
    Eigen::Matrix<double, 3, 3> _pre_Rdot_ref;

    Eigen::Matrix<double, 3, 1> goal;
    Eigen::Matrix<double, 7, 1> _qacc; 

    Eigen::Matrix<double, 6, 1> _xacc;
    Eigen::Matrix<double, 6, 1> _xacc_goal;
    Eigen::Matrix<double, 6, 1> pre_xdot_hand;

    Eigen::Matrix<double, 6, 1> _xacc_des;
    Eigen::Matrix<double, 6, 1> _pre_xacc;

    Eigen::Matrix<double, 7, 1> _torque_null;


    Eigen::Matrix<double, 6, 1> _xddot_des_hand;
    Eigen::Matrix<double, 6, 1> _xddot_goal_hand;

    Eigen::Matrix<double, 6, 1> _pre_x_goal_hand;

    Eigen::Matrix<double, 6, 1> _pre_FT;


    // addExternal_ForceTorque //
    char _body_name[10];
    Eigen::Matrix<double, 6, 1> _force_e;
    Eigen::Matrix<double, 3, 1> _force_s;
    Eigen::Matrix<double, 3, 1> _torque_s;
    double _time_e;


    Eigen::Matrix<double, 6, 1> _x_hand_esti;
    Eigen::Matrix<double, 7, 1> _q_des_esti;

    Eigen::Matrix<double, 6, 1> _F_esti;
    Eigen::Matrix<double, 6, 1> _pre_F_esti;
    Eigen::Matrix<double, 6, 1> _lpf_F_esti;
    Eigen::Matrix<double, 6, 6> _K_esti;
    Eigen::Matrix<double, 6, 1> _filtered_F_esti;
    Eigen::Matrix<double, 6, 1> _f_des;

    Eigen::Matrix<double, 6, 1> _pre_FTdata;
    Eigen::Matrix<double, 6, 1> _lpf_FTdata;
    // Eigen::Matrix<double, 6, 1> _FTdata;
    Eigen::Matrix<double, 6, 1> FTdata;

    VectorXd _FTdata;

    Eigen::Matrix<double, 7, 1> _lpf_torque;
    Eigen::Matrix<double, 7, 1> _pre_torque;

    Eigen::Matrix<double, 7, 1> _pre_q_goal;
    Eigen::Matrix<double, 7, 1> _pre_qdot_goal;
    

    VectorXd ControlBarrierFunction(const VectorXd q, const VectorXd q_min, const VectorXd q_max);
    VectorXd ControlBarrierFunctionDot(const VectorXd q, VectorXd qdot, const VectorXd q_min, const VectorXd q_max);
    void CBF_Initialize();
    void CbfController();
    int cons;
    double threshold;
    int max_iter;
    CQuadraticProgram qp;
    MatrixXd _H;
    VectorXd _g;
    MatrixXd _A;
    VectorXd _lbA;
    VectorXd _ubA;
    VectorXd _lb;
    VectorXd _ub;
    void QPTaskSpaceControl();
    void QPTaskSpaceControl_Initialize();

    VectorXd _q_min;
    VectorXd _q_max;
    VectorXd _opt_q;
    MatrixXd Q;

    Eigen::Matrix<double, 7, 1> _torqueForce;
    Eigen::Matrix<double, 6, 1> _Force_error;
    VectorXd _F_desired;
    VectorXd _int_F_desired_;
    double _kpf, _kif;

    VectorXd VirtualTank(VectorXd xdot, VectorXd xdot_des, Eigen::Matrix<double, 6, 1> force_ext, Eigen::Matrix<double, 6, 1> force_ref, Vector3d gamma);
    double _state_x;
    double _state_xdot;
    double _E_t;
    Vector3d _gamma;
    Vector3d gamma;
    Vector3d _Power_in;

    VectorXd _input;
    VectorXd _modify_xdot_des_hands;
    VectorXd _modify_x_des_hands;

    VectorXd _m_x_err;
    VectorXd _m_xdot_err;
    Eigen::Matrix<double, 6, 1> F_temp;
    Eigen::Matrix<double, 3, 3> _m_R_des_hand;
    
    double _P_total;
    double _P_up, _P_low;
    Vector3d _p_up_re, _p_low_re;
    double _E_up, _E_low;
    double E_threshold;
    double cos_var;

    double s_dot;
    double s;


    Vector3d ValveGainScheduler(Vector3d _Power_in, double P_total, Vector3d gamma, double _E_t);
};

#endif
