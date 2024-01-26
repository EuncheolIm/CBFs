#include "controller.h"
#include <chrono>

#include <fstream> // ifstream header
#include <iostream>
#include <string> // getline header

CController::CController()
{
	dof = 7;
	Initialize();
	// QPTaskSpaceControl_Initialize();

	// CBF_Initialize();

	
}

CController::~CController()
{
}
bool CController::ParticleYamlRead(std::string path, Eigen::VectorXd& unit)
{
    YAML::Node yaml_node;
    try
    {
        yaml_node = YAML::LoadFile(path.c_str());
        unit(0) = yaml_node["Impedance_gain"]["kpf"].as<double>();
        unit(1) = yaml_node["Impedance_gain"]["kif"].as<double>();

	}
    catch(const std::exception& e)
    {
        std::cerr << "fail to read particle yaml file" <<std::endl;
        return false;
    }
    return true;
}
void CController::addExternal_ForceTorque(const char** body_name, double* force, double* time)
{
	*body_name = "temp";
	_time_e = 5.0;
	_force_e << 0, 0, 0, 0, 0 ,0;
	for(int i=0; i<6; i++)
	{
		force[i] = _force_e(i);
	}
	*time = _time_e;
}
void CController::read(double t, double* q, double* qdot, double* sensordata)
{	
	_t = t;
	if (_bool_init == true)
	{
		_init_t = _t;
		_bool_init = false;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < dof; i++)
	{
		_q(i) = q[i];
		_qdot(i) = qdot[i];		
	}
	for (int i = 0; i < 6; i++)
	{
		FTdata(i) = sensordata[i];
	}
}

void CController::write(double* torque)
{
	for (int i = 0; i < dof; i++)
	{
		torque[i] = _torque(i);
		// torque[i] = 0;
	}
}

void CController::control_mujoco()
{
    ModelUpdate();
    motionPlan();

	if(_control_mode == 1) // joint space control
	{	

		if (_t - _init_t < 0.1 && _bool_joint_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;

			JointTrajectory.reset_initial(_start_time, _q, _qdot);
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);

			// JointTrajectory.reset_initial(_start_time, _q, _pre_qdot_goal, _qddot_init);
			// JointTrajectory.update_goal(_q_goal, _qdot_goal, _qddot_goal, _end_time);
			_bool_joint_motion = true;
		}
		
		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();
		
		JointControl();
		// CbfController();
		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
	}
	else if(_control_mode == 2) // task space control
	{	
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			_HandTrajectory.reset_initial(_start_time, _x_hand, _xdot_hand);
			_HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, _end_time); 

			// _HandTrajectory.reset_initial(_start_time, _pre_x_goal_hand, _xdot_goal_hand);
			// _HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, _end_time); 
			_bool_ee_motion = true;
		}


		_HandTrajectory.update_time(_t);
		_x_des_hand.head(3) = _HandTrajectory.position_cubicSpline();
		_R_des_hand = _HandTrajectory.rotationCubic();
		_x_des_hand.segment<3>(3) = CustomMath::GetBodyRotationAngle(_R_des_hand);
		_xdot_des_hand.head(3) = _HandTrajectory.velocity_cubicSpline();
		_xdot_des_hand.segment<3>(3) = _HandTrajectory.rotationCubicDot();

		// OperationalSpaceControl();
		
		// CLIK();

		TankController();
		// QPTaskSpaceControl();
		
		if (_HandTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
	}
}
void CController::ModelUpdate()
{
    Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
    Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	_J_hands = Model._J_hand;
	_J_T_hands = _J_hands.transpose();
	_J_bar_hands = CustomMath::pseudoInverseQR(_J_hands);

	//calc Jacobian dot (with lowpass filter)
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < dof; j++)
		{
			_Jdot_hands(i,j) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_J_hands(i,j), _J_hands(i,j), _pre_Jdot_hands(i,j)); //low-pass filter

			_pre_J_hands(i, j) = _J_hands(i, j);
			_pre_Jdot_hands(i, j) = _Jdot_hands(i, j);
		}
	}
	_Jdot_qdot = _Jdot_hands * _qdot;


	_x_hand.head(3) = Model._x_hand;
	_x_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_hand);

	_xdot_hand = Model._xdot_hand;

	_force_s = FTdata.head(3);
	_torque_s = FTdata.tail(3);

	_FTdata.segment(0,3) = CustomMath::rotateWithX(-90*DEG2RAD)*CustomMath::rotateWithY(-90*DEG2RAD)*_force_s;
	_FTdata.segment(3,3) = CustomMath::rotateWithX(-90*DEG2RAD)*CustomMath::rotateWithY(-90*DEG2RAD)*_torque_s;

	for(int i = 0; i < 7; i++){
		_A_diagonal(i,i) = Model._A(i,i);
	}
}

void CController::motionPlan()
{	
	// time_plan default = 5.0 seconds
	_time_plan(1) = 1.0; // move home position
	_time_plan(2) = 5.0; // wait
	_time_plan(3) = 10.0; // wait
	_time_plan(4) = 10.0; // wait

	if (_bool_plan(_cnt_plan) == 1)
	{
		cout << "_cnt_plan : "<<_cnt_plan<<endl;
		_cnt_plan = _cnt_plan + 1;
		if (_cnt_plan == 1)
		{	
			_q_goal.setZero(dof);
			_q_goal = _q_home;
			reset_target(_time_plan(_cnt_plan), _q_goal);
		}
		// if (_cnt_plan == 2)
		// {	
		// 	_q_goal(2) += 20*DEG2RAD;
		// 	_q_goal(4) += 20*DEG2RAD;
		// 	_q_goal(6) += 20*DEG2RAD;
		// 	reset_target(_time_plan(_cnt_plan), _q_goal);
		// }
		else if (_cnt_plan == 2)
		{
			_pos_goal_hand(0) =  0.50 ;//_x_hand(0) ;
			_pos_goal_hand(1) =  0.0;//_x_hand(1) +0.2;
			_pos_goal_hand(2) =  0.75 ;//_x_hand(2) ;

			_rpy_goal_hand(0) = 1.5647;
			_rpy_goal_hand(1) = -0.789178; //- 0.2;
			_rpy_goal_hand(2) = 1.57512;//+ 0.5;

			reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		}
		else if (_cnt_plan == 3)
		{
			_pos_goal_hand(0) =  0.626 ;//_x_hand(0) ;
			_pos_goal_hand(1) =  0.0;//_x_hand(1) +0.2;
			_pos_goal_hand(2) =  0.75 ;//_x_hand(2) ;

			_rpy_goal_hand(0) = 1.5647;
			_rpy_goal_hand(1) = -0.789178; //- 0.2;
			_rpy_goal_hand(2) = 1.57512;//+ 0.5;

			reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		}
		else if (_cnt_plan == 4)
		{
			_pos_goal_hand(0) =  0.75 ;//_x_hand(0) ;
			_pos_goal_hand(1) =  0.0;//_x_hand(1) +0.2;
			_pos_goal_hand(2) =  0.75 ;//_x_hand(2) ;

			_rpy_goal_hand(0) = 1.5647;
			_rpy_goal_hand(1) = -0.789178; //- 0.2;
			_rpy_goal_hand(2) = 1.57512;//+ 0.5;

			reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		}
		
	}
}


void CController::reset_target(double motion_time, VectorXd target_joint_position)
{
	_control_mode = 1;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_goal = target_joint_position.head(7);
	_qdot_goal.setZero();
}
void CController::reset_target(double motion_time, Vector3d target_pos, Vector3d target_ori)
{
	_control_mode = 2;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_hand.head(3) = target_pos;
	_x_goal_hand.tail(3) = target_ori;
	_xdot_goal_hand.setZero();
}

VectorXd CController::VirtualTank(VectorXd xdot, VectorXd xdot_des, Eigen::Matrix<double, 6, 1> force_ext, 
													Eigen::Matrix<double, 6, 1> force_ref, Vector3d cal_gamma)
{
	_E_t = pow(_state_x,2) / 2;
	_Power_in(0) = -xdot.transpose()*force_ref;
	_Power_in(1) = -xdot_des.transpose()*-force_ref;
	_Power_in(2) = -xdot_des.transpose()*-force_ext;
	cout << "---------------"<<endl;
	cout << "_Power_in: "<<_Power_in.transpose()<<endl;
	cout << "_E_t: "<<_E_t<<endl;

	_P_total = cal_gamma.transpose()*_Power_in;
	_state_xdot = _P_total;///_state_x; 
	_state_x = _state_x + _state_xdot*_dt;

	cout << "xdot_des: "<<xdot_des.rows()<<" "<<xdot_des.cols()<<" "<<xdot_des.transpose()<<endl;

	_input = xdot_des;
	for(int i=0; i<3; i++)
    {
        if (_Power_in(i) < 0 && _E_t <=_E_low ){
            // _input(i) = cal_gamma(i)*_input(i);
			_Power_in(i) = 0.0;

        }
        else{_Power_in(i) = _Power_in(i); }
    }
	cout << "Port Individual Power Limit: "<<_Power_in.transpose() << endl;
	for(int i=0; i<3; i++)
    {
        if (_Power_in(i) < _P_up){
            gamma(i) = _P_up/_Power_in(i);
			// cout << "i"<<i<<" "<<gamma.transpose() << endl;
        }
        else if (_Power_in(i) >_p_low_re(i)){
            gamma(i) = _p_low_re(i)/_Power_in(i);
			// cout << "i"<<i<<" "<<gamma.transpose() << endl;
        }
        else{gamma(i) = gamma(i); 
		// cout << "i"<<i<<" "<<gamma.transpose() << endl;
		}
    }
	
	_Power_in = gamma.transpose()*_Power_in;
	cout << "Tank Power Limit gamma: "<<gamma.transpose() << endl;

	cout << "_Power_in: "<<_Power_in.rows()<<" "<<_Power_in.cols()<<" "<<_Power_in.transpose()<<endl;

	return	_input;
}
Vector3d CController::ValveGainScheduler(Vector3d Power_in, double P_total, Vector3d gamma, double E_t)
{	// Port Individual Power Limit
	for(int i=0; i<3; i++)
    {
        if (gamma(i)*Power_in(i) > _P_up){
            gamma(i) = _P_up/Power_in(i);
			// cout << "i"<<i<<" "<<gamma.transpose() << endl;
        }
        else if (gamma(i)*Power_in(i) <_p_low_re(i)){
            gamma(i) = _p_low_re(i)/Power_in(i);
			// cout << "i"<<i<<" "<<gamma.transpose() << endl;
        }
        else{gamma(i) = gamma(i); 
		// cout << "i"<<i<<" "<<gamma.transpose() << endl;
		}
    }
	cout << "Port Individual Power Limit gamma: "<<gamma.transpose() << endl;
	
	// Tank Power Limit (EGS)
	// for(int i=0; i<3; i++)
    // {
    //     if (gamma(i)*Power_in(i) > _P_up){
    //         gamma(i) = gamma(i)*_P_up/P_total;
    //     }
    //     else if (_gamma(i)*Power_in(i) <_P_low){
    //         gamma(i) = gamma(i)*_P_low/P_total;
    //     }
    //     else{gamma(i) = gamma(i);}
    // }
	for(int i=0; i<3; i++)
    {
        if (gamma.transpose()*Power_in > _P_up){
            gamma(i) = gamma(i)*_P_up/(gamma.transpose()*Power_in);
        }
        else if (gamma.transpose()*Power_in <_P_low){
            gamma(i) = gamma(i)*_P_low/(gamma.transpose()*Power_in);
        }
        else{gamma(i) = gamma(i);}
    }
	cout << "Tank Power Limit gamma: "<<gamma.transpose() << endl;

	// // Tank Energy Lower Limit 
	// for(int i=0; i<3; i++)
    // {
	// 	if(Power_in(i) <=0 && E_t<=_E_low){
	// 		cout << " p1"<<endl;
	// 		gamma(i) = 0.0;
	// 	}
	// 	else if(Power_in(i) <=0 && _E_low < E_t <= _E_low + E_threshold){
	// 		cos_var = PI*((E_t-_E_low)/E_threshold);
	// 		gamma(i) = (1-cos(cos_var)) / 2;
	// 		cout << " p2"<<endl;
	// 	}
	// 	else{gamma(i) = gamma(i); cout << " p3"<<endl;}
	// }
	// cout << "Tank Energy Lower Limit gamma: "<<gamma.transpose() << endl;

	// // Tank Energy Upper Limit 
	for(int i=0; i<3; i++)
    {
		if(E_t >= _E_up){
			gamma(i) = 0.0;
		}
		else{gamma(i) = gamma(i);}
	}

	cout << "Tank Energy Upper Limit gamma: "<<gamma.transpose() << endl;
	return	gamma;
	
}
void CController::JointControl()
{	
	_torque.setZero();
	_torque = Model._A*(400*(_q_des - _q) + 40*(_qdot_des - _qdot)) + Model._bg;

}
void CController::CLIK()
{
	_torque.setZero();	

	_x_err_hand.segment(0,3) = _x_des_hand.head(3) - _x_hand.head(3);
	// _R_des_hand = CustomMath::GetBodyRotationAngle(_x_des_hand(3),_x_des_hand(4),_x_des_hand(5))
	_x_err_hand.segment(3,3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);

	_qdot_des = _J_bar_hands*(_xdot_des_hand + _x_kp*(_x_err_hand));
	_q_des = _q + _dt*_qdot_des;

	_torque = _A_diagonal*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot)) + Model._bg;
	// _torque =  Model._A*(_kp*(_q_des - _q) + _kd*(_qdot_des - _qdot)) + Model._bg;
}
void CController::OperationalSpaceControl()
{
	_torque.setZero();
	_lambda =  CustomMath::pseudoInverseQR(_J_hands.transpose())*Model._A*_J_bar_hands;
	_Null_hands = _Id_7 - _J_T_hands * _lambda * _J_hands * Model._A.inverse();

	_x_err_hand.segment(0,3) = _x_des_hand.head(3) - _x_hand.head(3);
	_x_err_hand.segment(3,3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);
	
	_xdot_err_hand.segment(0, 3) = _xdot_des_hand.head(3) - _xdot_hand.head(3);
	_xdot_err_hand.segment(3, 3) = -_xdot_hand.tail(3);

	// Operational space
	// _torque = _J_T_hands * _lambda * (_kpj *_x_err_hand + _kdj * _xdot_err_hand ) + Model._bg ;

	// Operational space + Null space control
	// _q_des = _q;
	// _torque_null = _Null_hands * Model._A * (400 * (_q_des - _q)) ;
	// _torque = _J_T_hands * _lambda * (_kpj *_x_err_hand + _kdj * _xdot_err_hand ) + _torque_null + Model._bg ;

	_Force_error = (_FTdata - _F_desired );
	_int_F_desired_ = _int_F_desired_ + _Force_error*_dt;

	_torqueForce = _J_T_hands * (_kpf*_Force_error + _kif*_int_F_desired_);

	_torque = _J_T_hands * _lambda * (_kpj *_x_err_hand + _kdj * _xdot_err_hand ) + _torqueForce + Model._bg ;


}
void CController::TankController()
{
	_lambda =  CustomMath::pseudoInverseQR(_J_hands.transpose())*Model._A*_J_bar_hands;
	_Null_hands = _Id_7 - _J_T_hands * _lambda * _J_hands * Model._A.inverse();

	_x_err_hand.segment(0,3) = _x_des_hand.head(3) - _x_hand.head(3);
	_x_err_hand.segment(3,3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);
	
	_xdot_err_hand.segment(0, 3) = _xdot_des_hand.head(3) - _xdot_hand.head(3);
	_xdot_err_hand.segment(3, 3) = -_xdot_hand.tail(3);

	_Force_error = (_FTdata - _F_desired );
	_int_F_desired_ = _int_F_desired_ + _Force_error*_dt;

	_torqueForce = _J_T_hands * (_kpf*_Force_error + _kif*_int_F_desired_);

	F_temp =  _lambda * (_kpj *_x_err_hand + _kdj * _xdot_err_hand ) + CustomMath::pseudoInverseQR(_J_hands.transpose())*(_torqueForce + Model._bg);

	_modify_xdot_des_hands = VirtualTank(_xdot_hand, _xdot_des_hand, _FTdata, F_temp, _gamma);
	// _gamma = ValveGainScheduler(_Power_in, _P_total, _gamma, _E_t);

	_modify_x_des_hands = _x_hand + _modify_xdot_des_hands * _dt;
	cout << "_modify_xdot_des_hands: "<<_modify_xdot_des_hands.rows()<<" "<<_modify_xdot_des_hands.cols()<<" "<<_modify_xdot_des_hands.transpose()<<endl;

	_m_R_des_hand = CustomMath::GetBodyRotationMatrix(_modify_xdot_des_hands(3), _modify_xdot_des_hands(4), _modify_xdot_des_hands(5));

	_m_x_err.head(3) = _modify_x_des_hands.head(3) - _x_hand.head(3);
	_m_x_err.tail(3) = -CustomMath::getPhi(Model._R_hand, _m_R_des_hand);

	_m_xdot_err.head(3) = _modify_xdot_des_hands.head(3) - _xdot_hand.head(3);
	_m_xdot_err.tail(3) =  - _xdot_hand.tail(3);
	cout << "_modify_x_des_hands: "<<_modify_x_des_hands.rows()<<" "<<_modify_x_des_hands.cols()<<" "<<_modify_x_des_hands.transpose()<<endl;

	_torque = _J_T_hands * _lambda * (_kpj *_m_x_err + _kdj * _m_xdot_err ) + _torqueForce + Model._bg ;
}
VectorXd CController::ControlBarrierFunction(const VectorXd q, const VectorXd q_min, const VectorXd q_max)
{
	VectorXd h(q.size());
	int alpha = 1.0;
	h = (q - q_min)(q - q_max);

	return alpha *h;
}
VectorXd CController::ControlBarrierFunctionDot(const VectorXd q, VectorXd qdot, const VectorXd q_min, const VectorXd q_max)
{
	VectorXd hdot(q.size());
	VectorXd dh_dq(q.size());
	dh_dq = (q_max - q) - (q - q_min);
	hdot = dh_dq * qdot;

	return hdot;
}
void CController::CbfController()
{	
	// set control barrier function
	VectorXd alpha_h,hdot;
	alpha_h.setZero(7);
	hdot.setZero(7);
	alpha_h = ControlBarrierFunction(_q, _q_min, _q_max);
	hdot = ControlBarrierFunctionDot(_q, _qdot, _q_min, _q_max);

	qp.InitializeProblemSize(7, 7); // consider joint
	// _H = _q.transpose() * Q * _q;
	// _g = -2*_q.transpose() * Q * _q_des;
	// _H = Q ;
	// _g = -2* Q * _q_des;

	_H.setIdentity();
	_g = -2* _q_des;
	qp.UpdateMinProblem(_H,_g);

	//set A*x <= b
	// _A.diagonal() = 2*_qdot;
	_A.diagonal() <<1,1,1,1,1,1,1;

	for (int i = 0; i < 7; i++)
	{
		_lbA(i) = -hdot(i) - alpha_h(i);
		_ubA(i) = 1e5 ;
	}
	qp.UpdateSubjectToAx(_A, _lbA, _ubA); // _A1,_lbA1,_ubA1 update to HQP_P1

	//Solve
	qp.EnableEqualityCondition(0.0001);
	qp.EnablePrintOptionDebug();
	qp.SolveQPoases(max_iter);
	for (int i = 0; i <7; i++)
	{
		_opt_q(i) = qp._Xopt(i);
	}

	_torque = 100 * (_opt_q - _q) + 10 * (- _qdot);

}
void CController::Initialize()
{	

	s_dot = 0.0; s = 0.0;
	std::remove("../data/panda_qddot.txt");
    _control_mode = 1; //1: joint space, 2: task space(CLIK)

	Eigen::VectorXd param;
    param.setZero(2);
	ParticleYamlRead("../libs/param.yaml", param);
	_state_x = 0;
	gamma.setZero();
	_gamma << 1.0, 1.0, 1.0;
	_input.setZero(6);
	_modify_x_des_hands.setZero(6);
	_modify_xdot_des_hands.setZero(6);

	_m_x_err.setZero(6);
	_m_xdot_err.setZero(6);

	_kpf = param(0); _kif = param(1);
	_p_up_re.setZero(3);
	_p_low_re<< -2,-2,-2;
	_P_up = 0.0; _P_low = -10.0;
	_E_up = 120; _E_low = 0;
	E_threshold = 0.5;


	log.setZero(14);
	_bool_init = true;
	_t = 0.0;
	_init_t = 0.0;
	_pre_t = 0.0;
	_dt = 0.0;

	_kpj = 400.0;
	_kdj = 40.0;

	_x_kp = 20.0;

    _q.setZero(dof);
	_qdot.setZero(dof);
	_torque.setZero(dof);

	_J_hands.setZero(6,dof);
	_J_bar_hands.setZero(dof,6);

	_x_hand.setZero(6);
	_xdot_hand.setZero(6);

	_cnt_plan = 0;
	_bool_plan.setZero(30);
	_time_plan.resize(30);
	_time_plan.setConstant(5.0);

	_q_home.setZero(dof);

	_q_home(0) = 0;
    _q_home(1) = -M_PI_4;
    _q_home(2) = 0;
    _q_home(3) = -3 * M_PI_4;
    _q_home(4) = 0;
    _q_home(5) = M_PI_2;
    _q_home(6) = M_PI_4;


	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_des.setZero(dof);
	_qdot_des.setZero(dof);
	_q_goal.setZero(dof);
	_qdot_goal.setZero(dof);

	_x_des_hand.setZero(6);
	_xdot_des_hand.setZero(6);
	_x_goal_hand.setZero(6);
	_xdot_goal_hand.setZero(6);

	_pos_goal_hand.setZero(); // 3x1 
	_rpy_goal_hand.setZero(); // 3x1

	JointTrajectory.set_size(dof);
	_A_diagonal.setZero(dof,dof);

	_x_err_hand.setZero(6);
	_R_des_hand.setZero();

	_xdot_err_hand.setZero(6);

	_Id_7.setIdentity();
	_Id_14.setIdentity();

	_lambda.setZero();
	_Rdot_des_hand.setZero();
	_Rdot_hand.setZero();
	_Null_hands.setZero();
	_J_T_hands.setZero();

	_force_s.setZero();
	_torque_s.setZero();
	_FTdata.setZero();

	_qddot_goal.setZero(dof);
	_torque_null.setZero();

	_F_desired.setZero(6);
	
	_int_F_desired_.setZero(6);
	_FTdata.setZero(6);
}
void CController::CBF_Initialize()
{
	cons = 7; // number of constraints
	max_iter = 1000;
	threshold = 0.0001;
	_opt_q.setZero(7);
	// qp.InitializeProblemSize(20, 13); // consider joint
	qp.InitializeProblemSize(7, 7); // consider joint
	_H.setZero(qp._num_var, qp._num_var);
	_g.setZero(qp._num_var);
	_A.setZero(qp._num_cons, qp._num_var);
	_lbA.setZero(qp._num_cons);
	_ubA.setZero(qp._num_cons);
	_lb.setZero(qp._num_var);
	_ub.setZero(qp._num_var);

	Q.setZero(7,7);
	Q.diagonal() << 0.1,0.1,0.1,0.1,0.1,0.1,0.1;

	_q_min = Model._min_joint_position;
	_q_max = Model._max_joint_position;
}
void CController::QPTaskSpaceControl_Initialize()
{
	max_iter = 1000;
	threshold = 0.0001;
	qp.InitializeProblemSize(20, 13); // consider joint
	_H.setZero(qp._num_var, qp._num_var);
	_g.setZero(qp._num_var);
	_A.setZero(qp._num_cons, qp._num_var);
	_lbA.setZero(qp._num_cons);
	_ubA.setZero(qp._num_cons);
	_lb.setZero(qp._num_var);
	_ub.setZero(qp._num_var);
}
void CController::QPTaskSpaceControl()
{
	_torque.setZero();
	_x_err_hand.head(3) = _x_des_hand.head(3) - Model._x_hand;
	_R_des_hand = CustomMath::GetBodyRotationMatrix(_x_des_hand(3), _x_des_hand(4), _x_des_hand(5));
	_x_err_hand.tail(3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);

	_xdot_err_hand.head(3) = _xdot_des_hand.head(3) - Model._xdot_hand.segment(0, 3);
	_xdot_err_hand.tail(3) = -Model._xdot_hand.segment(3, 3); //only daming for orientation

	_xddot_star.segment(0, 3) = 400 * _x_err_hand + 40 * _xdot_err_hand;//left hand position control
	_xddot_star.segment(3, 3) = 400 * _x_err_hand + 40 * _xdot_err_hand;//left hand orientation control

	////////////////////////////////////////////////////////////////
	///////// solve QP /////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	double threshold = 0.0001;
	int max_iter = 1000;
	//set cost function x^T*H*x + g
	_H.setZero();
	for (int i = 0; i < 14; i++)
	{
		_H(i, i) = 0.0001;
	}
	for (int i = 14; i < 20; i++)
	{
		_H(i, i) = 1.0;
	}

	_g.setZero();
	qp.UpdateMinProblem(_H,_g);

	//set A*x <= b
	_A.setZero();
	_lbA.setZero();
	_ubA.setZero();
	_A.block<7,7>(0,0) = Model._A;
	_A.block<7,7>(0,7) = -_Id_7;
	_A.block<6,7>(7,0) = _J_hands;
	_A.block<6,6>(7,14) = -_Id_6;
	for (int i = 0; i < 7; i++)
	{
		_lbA(i) = -Model._bg(i) - threshold; // model._bg =  cori + central + gravity
		_ubA(i) = -Model._bg(i) + threshold;
	}
	for (int i = 0; i < 6; i++) //i=15~26
	{
		_lbA(i + 7) = -_Jdot_qdot(i) + _xddot_star(i) - threshold; // xddot-Jdot_qdot
		_ubA(i + 7) = -_Jdot_qdot(i) + _xddot_star(i) + threshold;
	}
	qp.UpdateSubjectToAx(_A, _lbA, _ubA); // _A1,_lbA1,_ubA1 update to HQP_P1

	//set lb <= x <= ub
	_lb.setZero();
	_ub.setZero();
	//joint acceleration limit (for joint position and velocity) i=0~14
	for (int i = 0; i < 7; i++)
	{
		_lb(i) = -500.0;
		_ub(i) = 500.0;
	}
	//joint torque limit i=15~29
	for (int i = 0; i < 7; i++)
	{
		_lb(i + 7) = -2000.0;
		_ub(i + 7) = 2000.0;
	}
	//task limit i=30~41 == slack variables (w)
	for (int i = 0; i < 6; i++)
	{
		_lb(i + 14) = -500.0;
		_ub(i + 14) = 500.0;
	}
	qp.UpdateSubjectToX(_lb, _ub); //update x constraint range , lb,ub,

	//Solve
	qp.EnableEqualityCondition(0.0001);
	// QP.EnablePrintOptionDebug();
	qp.SolveQPoases(max_iter);
	for (int i = 0; i <7; i++)
	{
		_torque(i) = qp._Xopt(i+7);
	}

}