#include "controller.h"
#include <chrono>

#include <fstream> // ifstream header
#include <iostream>
#include <string> // getline header

CController::CController()
{
	dof = 7;
	Initialize();
	Initialize_MPC();
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
        unit(0) = yaml_node["MPC_1124"]["ee_pos"].as<double>();
		unit(1) = yaml_node["MPC_1124"]["ee_pos_rot"].as<double>();
        unit(2) = yaml_node["MPC_1124"]["ee_vel"].as<double>();
		unit(3) = yaml_node["MPC_1124"]["ee_vel_rot"].as<double>();
        unit(4) = yaml_node["MPC_1124"]["force"].as<double>();
		unit(5) = yaml_node["MPC_1124"]["R"].as<double>();

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
	}
}

void CController::control_mujoco()
{
    ModelUpdate();
    motionPlan();

	if(_control_mode == 1) // joint space control
	{	
		// if (taskfirst == true)
		// {	
		// 	_pre_q_goal = _q;
		// 	_pre_qdot_goal.setZero();
		// 	taskfirst = false;
		// }
		if (_t - _init_t < 0.1 && _bool_joint_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;

			// JointTrajectory.reset_initial(_start_time, _q, _qdot);
			// JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);

			JointTrajectory.reset_initial(_start_time, _q, _pre_qdot_goal, _qddot_init);
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _qddot_goal, _end_time);

			// JointTrajectory.reset_initial(_start_time, _pre_q_goal, _pre_qdot_goal, _qddot_init);
			// JointTrajectory.update_goal(_q_goal, _qdot_goal, _qddot_goal, _end_time);
			// _pre_q_goal = _q_goal;

			_bool_joint_motion = true;
		}
		if ( PathTracking == "yes"){
			for(int i=0; i<Np; i++)
			{	
				_t = _t + i*0.001;
				JointTrajectory.update_time(_t);
				_q_des = JointTrajectory.position_cubicSpline();
				_qdot_des = JointTrajectory.velocity_cubicSpline();
				Y_des.segment(7*(2*i),7) = _q_des;
				Y_des.segment(7*(2*i + 1),7) = _qdot_des;
			}
		}
		else if ( PathTracking == "no")
		{
			for(int i = 0; i <Np; i++){
			Y_des.segment(7*(2*i),7) = _q_goal;
			}
		}
		// MPC();

		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();
		_qddot_des = JointTrajectory.acceleration_cubicSpline();
		JointControl();
		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}

	}
	else if(_control_mode == 2) // task space control
	{	
		if (taskfirst == true)
		{	
			_pre_x_goal_hand = _x_hand;
			_xdot_des_hand = _xdot_hand;
			taskfirst = false;
		}
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			// _HandTrajectory.reset_initial(_start_time, _x_hand, _xdot_hand);
			// _HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, _end_time); 

			_HandTrajectory.reset_initial(_start_time, _pre_x_goal_hand, _xdot_goal_hand);
			_HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, _end_time); 
			_bool_ee_motion = true;
		}

		if (freq_check == loop)
		{	
			b_100hz_t = true;
			if (b_100hz_t == true)
			{
				_100hz_t = _t;
				_100hz_t = _100hz_t - 0.01;	
				_f_des.setZero();
				b_100hz_t = false;
			}
			for(int i=0; i<Np; i++)
			{	
				_100hz_t = _100hz_t + 0.01;
				_HandTrajectory.update_time(_100hz_t);
				_x_des_hand.head(3) = _HandTrajectory.position_cubicSpline();
				_R_des_hand = _HandTrajectory.rotationCubic();
				_x_des_hand.segment(3,3) = CustomMath::GetBodyRotationAngle(_R_des_hand);

				_xdot_des_hand.head(3) = _HandTrajectory.velocity_cubicSpline();
				_xdot_des_hand.segment(3,3) = _HandTrajectory.rotationCubicDot();

				Y_temp.segment(0,6) = _x_des_hand; 		// x_des
				Y_temp.segment(6,6) = _xdot_des_hand; 	// xdot_des
				Y_des.segment(12*i,12) = Y_temp;
			}
		}

		// _HandTrajectory.update_time(_t);
		// _x_des_hand.head(3) = _HandTrajectory.position_cubicSpline();
		// _R_des_hand = _HandTrajectory.rotationCubic();
		// _x_des_hand.segment<3>(3) = CustomMath::GetBodyRotationAngle(_R_des_hand);
		// _xdot_des_hand.head(3) = _HandTrajectory.velocity_cubicSpline();
		// _xdot_des_hand.segment<3>(3) = _HandTrajectory.rotationCubicDot();
		// OperationalSpaceControl();
		// CLIK();

		MPC();
		
		if (_HandTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
			cout << "_q_now: "<<_q.transpose()<<endl;
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
		for (int j = 0; j < 7; j++)
		{
			_Jdot_hands(i,j) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_J_hands(i,j), _J_hands(i,j), _pre_Jdot_hands(i,j)); //low-pass filter

			_pre_J_hands(i, j) = _J_hands(i, j);
			_pre_Jdot_hands(i, j) = _Jdot_hands(i, j);
		}
	}
	_Jdot_qdot = _Jdot_hands * _qdot;
	_lambda =  CustomMath::pseudoInverseQR(_J_hands.transpose())*Model._A*_J_bar_hands;


	_x_hand.head(3) = Model._x_hand;
	_x_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_hand);

	_xdot_hand = Model._xdot_hand;

	_force_s = FTdata.head(3);
	_torque_s = FTdata.tail(3);
	// finger is rotated Y-axis : -90 deg + X-axis : -90 deg
	_FTdata.segment(0,3) = CustomMath::rotateWithX(-90*DEG2RAD)*CustomMath::rotateWithY(-90*DEG2RAD)*_force_s;
	_FTdata.segment(3,3) = CustomMath::rotateWithX(-90*DEG2RAD)*CustomMath::rotateWithY(-90*DEG2RAD)*_torque_s;

	for (int i = 0; i < 6; i++)
	{
		_lpf_FTdata(i) = CustomMath::LowPassFilter(_dt, 2.0 * PI * 10.0, _FTdata(i), _pre_FTdata(i)); // 8 ~ 10
		_pre_FTdata(i) = _lpf_FTdata(i);
	}
	_lpf_FTdata = _lpf_FTdata;

	for(int i = 0; i < 7; i++){
		_A_diagonal(i,i) = Model._A(i,i);
	}
}

void CController::motionPlan()
{	
	_time_plan(1) = 3.0; // move home position
	_time_plan(2) = 4.0; // wait
	_time_plan(3) = 4.0; // joint goal motion
	_time_plan(4) = 4.0; // wait
	_time_plan(5) = 4.0; // task goal motion
	_time_plan(6) = 4.0; // wait
	_time_plan(7) = 4.0; // wait
	if (_bool_plan(_cnt_plan) == 1)
	{	
		cout << "_cnt_plan : "<<_cnt_plan<<endl;
		_cnt_plan = _cnt_plan + 1;
		if (_cnt_plan == 1)
		{	
			_q_goal.setZero(dof);
			_q_goal = _q_home;
			// _q_goal(0) = 0.0;
			// _q_goal(1) = 0.0;
			// _q_goal(2) = 0.0;
			// _q_goal(3) = -1.64;
			// _q_goal(4) = 0.0;
			// _q_goal(5) = 3.3;
			// _q_goal(6) = -0.782;
			// _q_goal(0) = 0.00941294	*DEG2RAD;
			// _q_goal(1) = 2.70191	*DEG2RAD;
			// _q_goal(2) = -0.0180558*DEG2RAD;
			// _q_goal(3) = -98.3671	*DEG2RAD;
			// _q_goal(4) = -0.0449125	*DEG2RAD;
			// _q_goal(5) = 191.069	*DEG2RAD;
			// _q_goal(6) = -44.9551	*DEG2RAD;

			reset_target(_time_plan(_cnt_plan), _q_goal);
		}
		// else if (_cnt_plan == 2)
		// {	
		// 	_pos_goal_hand(0) = _x_hand(0);
		// 	_pos_goal_hand(1) = _x_hand(1) ;//+ 0.2;
		// 	_pos_goal_hand(2) = _x_hand(2);//- 0.1;

		// 	_rpy_goal_hand(0) = _x_hand(3);
		// 	_rpy_goal_hand(1) = _x_hand(4); //- 0.2;
		// 	_rpy_goal_hand(2) = _x_hand(5);//+ 0.5;

		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		// }
		else if (_cnt_plan == 2)
		{	
			// _pos_goal_hand(0) = 0.515;
			// _pos_goal_hand(1) = 0.0 ;//+ 0.2;
			// _pos_goal_hand(2) = 0.74;//- 0.1;
			// _pos_goal_hand(0) = 0.4;
			// _pos_goal_hand(1) = 0.0 ;//+ 0.2;
			// _pos_goal_hand(2) = 0.5;//- 0.1;

			// _rpy_goal_hand(0) = 90 *DEG2RAD;
			// // _rpy_goal_hand(1) = -45*DEG2RAD; //- 0.2;
			// _rpy_goal_hand(2) = 90 *DEG2RAD;//+ 0.5;

			_pos_goal_hand(0) = 0.672919;
			_pos_goal_hand(1) = 0.0314588 ;//+ 0.2;
			_pos_goal_hand(2) = 0.258;//- 0.1;

			_rpy_goal_hand(0) = 3.14159;
			_rpy_goal_hand(1) = 0.0; //- 0.2;
			_rpy_goal_hand(2) = 1.48353;//+ 0.5;

			reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		}
		//  else if (_cnt_plan == 3)
		// {
		// 	_pos_goal_hand(0) =  0.53 ;//_x_hand(0) ;
		// 	_pos_goal_hand(1) =  0.1;//_x_hand(1) +0.2;
		// 	_pos_goal_hand(2) =  0.55 ;//_x_hand(2) ;

		// 	_rpy_goal_hand(0) = 1.5647;
		// 	_rpy_goal_hand(1) = -0.789178; //- 0.2;
		// 	_rpy_goal_hand(2) = 1.57512;//+ 0.5;

		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		// }
		// else if (_cnt_plan == 3)
		// {
		// 	_pos_goal_hand(0) =  0.515 ;//_x_hand(0) ;
		// 	_pos_goal_hand(1) =  0.0;//_x_hand(1) +0.2;
		// 	_pos_goal_hand(2) =  0.75 ;//_x_hand(2) ;

		// 	_rpy_goal_hand(0) = 1.5647;
		// 	_rpy_goal_hand(1) = -0.789178; //- 0.2;
		// 	_rpy_goal_hand(2) = 1.57512;//+ 0.5;

		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		// }
		// else if (_cnt_plan == 4)
		// {
		// 	_pos_goal_hand(0) =  0.515 ;//_x_hand(0) ;
		// 	_pos_goal_hand(1) =  0.1;//_x_hand(1) ;
		// 	_pos_goal_hand(2) =  0.75 ;//_x_hand(2) -0.2;

		// 	_rpy_goal_hand(0) = 1.5647;
		// 	_rpy_goal_hand(1) = -0.789178; //- 0.2;
		// 	_rpy_goal_hand(2) = 1.57512;//+ 0.5;

		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		// }
		// else if (_cnt_plan == 5)
		// {
		// 	_pos_goal_hand(0) = 0.515;// _x_hand(0) ;
		// 	_pos_goal_hand(1) = 0.1;// _x_hand(1) -0.2;
		// 	_pos_goal_hand(2) = 0.65;// _x_hand(2) ;

		// 	_rpy_goal_hand(0) = 1.5647;
		// 	_rpy_goal_hand(1) = -0.789178; //- 0.2;
		// 	_rpy_goal_hand(2) = 1.57512;//+ 0.5;

		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		// }
		// else if (_cnt_plan == 6)
		// {
		// 	_pos_goal_hand(0) = 0.515;//_x_hand(0);
		// 	_pos_goal_hand(1) = 0.0;//_x_hand(1);
		// 	_pos_goal_hand(2) = 0.65;//_x_hand(2) + 0.2;

		// 	_rpy_goal_hand(0) = 1.5647;
		// 	_rpy_goal_hand(1) = -0.789178; //- 0.2;
		// 	_rpy_goal_hand(2) = 1.57512;//+ 0.5;

		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		// }
		// else if (_cnt_plan == 7)
		// {
		// 	_pos_goal_hand(0) = 0.515;//_x_hand(0);
		// 	_pos_goal_hand(1) = 0.0;//_x_hand(1);
		// 	_pos_goal_hand(2) = 0.75;//_x_hand(2) + 0.2;

		// 	_rpy_goal_hand(0) = 1.5647;
		// 	_rpy_goal_hand(1) = -0.789178; //- 0.2;
		// 	_rpy_goal_hand(2) = 1.57512;//+ 0.5;

		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		// }
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
	if(taskfirst == false){
		_pre_x_goal_hand = _x_goal_hand;
	}

	_x_goal_hand.head(3) = target_pos;
	_x_goal_hand.tail(3) = target_ori;
	_xdot_goal_hand.setZero();
}

void CController::JointControl()
{	
	_torque.setZero();
	_torque = Model._A*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot)) + Model._bg;
	// cout << "FTdata : "<<_FTdata.transpose()<<endl<<endl;
	// for (int i=0; i< 7; i++){
	// 		log(i) = _q(i);
	// 		log(i+7) = _q_des(i);
	// 		// log(i+14) = _q_goal(i);
	// }
	// fout.open("/home/kist/euncheol/Dual-arm/data/Sim_data/panda_qddot.txt",ios::app);
	// fout << log.transpose() <<endl;
	// fout.close();
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
	cout << "_q : "<<(_q*RAD2DEG).transpose() <<endl;

	for (int i=0; i< 6; i++){
			log(i) = _x_hand(i);
			log(i+6) = _x_des_hand(i);
	}
	fout.open("/home/kist/euncheol/Dual-arm/data/Sim_data/comp_clik.txt",ios::app);
	fout << log.transpose() <<endl;
	fout.close();
}
void CController::OperationalSpaceControl()
{
	_torque.setZero();

	// _lambda =  CustomMath::pseudoInverseQR(_J_hands.transpose())*Model._A*_J_bar_hands;
	_Null_hands = _Id_7 - _J_T_hands * _lambda * _J_hands * Model._A.inverse();

	_x_err_hand.segment(0,3) = _x_des_hand.head(3) - _x_hand.head(3);
	_x_err_hand.segment(3,3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);
	// _x_err_hand.segment(3,3) = -_x_hand.tail(3);
	
	_xdot_err_hand.segment(0, 3) = _xdot_des_hand.head(3) - _xdot_hand.head(3);
	_xdot_err_hand.segment(3, 3) = -_xdot_hand.tail(3);

	// Operational space
	_torque = _J_T_hands * _lambda*(400 *_x_err_hand + 40 * _xdot_err_hand ) + Model._bg;

	// _K_esti.diagonal() << 1,5,5,2,2,2;
	_K_esti.diagonal() << 1,1,1,1,1,1;
	_F_esti = _K_esti*( _x_des_hand - _x_hand);
	for(int i=0; i<6; i++)
	{
		_lpf_F_esti(i) = CustomMath::LowPassFilter(_dt, 2.0 * PI * 8.0, _F_esti(i), _pre_F_esti(i)); // 8 ~ 10
		_pre_F_esti(i) = _lpf_F_esti(i);
	}
	// cout <<"_FTdata     : "<<_FTdata.transpose() <<endl;
	// cout <<"_lpf_F_esti : "<<_lpf_F_esti.transpose()<<endl<<endl;

	for (int i = 0; i < 6; i++)
	{
		_lpf_F_esti(i) = CustomMath::LowPassFilter(_dt, 2.0 * PI * 8.0, _F_esti(i), _pre_F_esti(i)); // 8 ~ 10
		_pre_F_esti(i) = _lpf_F_esti(i);
	}

	// cout << "_F_esti : "<< _lpf_F_esti.transpose() <<endl;

	Md.diagonal() << 1,1,1,1,1,1;
	Kd.diagonal() << 40,500,500,300,300,500;
	for (int i=0; i<6; i++){
		// Critically Damped
		Dd(i,i) = 2 * sqrt( Md(i,i) * Kd(i,i) );
	}
	
	// _torque = _J_T_hands * _lambda*(Md.inverse()*(Dd*_xdot_err_hand + Kd*_x_err_hand -_lpf_F_esti)) + Model._bg + _J_T_hands*_lpf_F_esti;
	// Operational space + Null space control
	// _q_des = _q;
	// _torque_null = _Null_hands * Model._A * (400 * (_q_des - _q)) ;
	// _torque = _J_T_hands * _lambda * (_kpj *_x_err_hand + _kdj * _xdot_err_hand ) + _torque_null + Model._bg ;

	for (int i=0; i< 6; i++){
			log(i) = _x_des_hand(i);
			log(i+6) = _x_hand(i);
			// log(i+14) = Y_ref(i);
	}
	fout.open("/home/kist/euncheol/Dual-arm/data/Sim_data/panda_qddot.txt",ios::app);
	fout << log.transpose() <<endl;
	fout.close();

}
void CController::MPC()
{	
	// auto start = std::chrono::high_resolution_clock::now();
	if ( freq_check == loop){
		_mpc_init_t = _t;
		_lambda =  CustomMath::pseudoInverseQR(_J_hands.transpose())*Model._A*_J_bar_hands;
		_J_T_hands_lambda_bar =CustomMath::pseudoInverseQR(_J_T_hands*_lambda);
		///////////////////////////////// set state matrix// x^T = [ q, x, xdot, F_ext ]^T
		// A_d = 25x25
		// B_d = 25x7
		// 1. C_d = 12x25 [x, xdot]^T  
		// D_d = 25x1
		A_d.block<7,7>(0,0) = _Id_7;	A_d.block<7,6>(0,13) = dT * _J_bar_hands;
										A_d.block<6,6>(7,7) = _Id_6 ;      			A_d.block<6,6>(7,13) = dT*_Id_6 ;	
										A_d.block<6,6>(13,13) = _Id_6;				A_d.block<6,6>(13,19) = -dT*_J_T_hands_lambda_bar*_J_T_hands;
										// A_d.block<6,6>(13,13) = _Id_6;			A_d.block<6,6>(13,19) = -dT*_J_hands*Model._A.inverse()*_J_T_hands;
																				A_d.block<6,6>(19,19) = _Id_6;
		// B_d.block<6,7>(13,0) = dT * _J_hands * Model._A.inverse();
		// D_d.segment(13,6) = dT * (_Jdot_qdot - _J_hands * Model._A.inverse() * Model._bg);

		B_d.block<6,7>(13,0) = dT * _J_T_hands_lambda_bar;
		D_d.segment(13,6) = -1 * dT * _J_T_hands_lambda_bar*(Model._bg); //	D_d.segment(19,6) = -1 *_prev_F;
		// cout << "A" <<"\n" <<A_d<<endl;

		C_d.block<6,6>(0,7) = _Id_6;	C_d.block<6,6>(6,13) = _Id_6;	// y_k^T = [x, xdot]^T
		////////////////////////////////////////////////////////////////////////////////////////
		// defin F matrix
		F.block<12,25>(0,0) = C_d*A_d;
		for (int i = 1; i <Np; i++){
			F.block<12,25>(12*i,0) = F.block<12,25>(12*(i-1),0) * A_d; // 12 x 25
		}
		// define Phi_temp matrix
		temp_phi.block<12,25>(0,0) = C_d;
		for (int i = 1; i <Np; i++){
			temp_phi.block<12,25>(12*i,0) = temp_phi.block<12,25>(12*(i-1),0) * A_d; // 12 x 25
		}
		// define Phi_temp_B matrix
		for (int i = 0; i <Np; i++){
			temp_phi_B.block<12,7>(12*i,0) = temp_phi.block<12,26>(12*i,0) * B_d; // 12 x 7
		}// // temp_phi_B = temp_phi * B_d;
		// define Phi matrix
		for (int i = 0; i <Np; i++){
			for (int j = 0; j <Np; j++){
				if( j<=i){
					Phi.block<12,7>(12*i, 7*j) = temp_phi_B.block<12,7>(12*(i-j),0);
				}
			}
		}
		// define temp_V Vector
		temp_V.segment(0,25) = D_d;
		for (int i = 1; i <Np; i++){
			temp_V.segment(25*i,25) = A_d * temp_V.segment(25*(i-1),25) + D_d;// 25*Np
		}
		// define V Vector 
		for (int i = 0; i <Np; i++){
			V.segment(12*i,12) = C_d * temp_V.segment(25*i,25);// 12*Np
		}

		// calculate MPC
		// PathTracking = "yes";
		Y_ref = Y_des;

		max_iter = 1000;
		//////////////////////////
		X.segment(0,7) = _q;
  		X.segment(7,6) = _x_hand;
		X.segment(13,6) = _xdot_hand;
		X.segment(19,6) = _lpf_FTdata;
		//////////////////////////
		// qp.InitializeProblemSize(Np*dof, 0);
		qp.InitializeProblemSize(Np*dof, Np*12);
		_H1.setZero(qp._num_var, qp._num_var);
		_g1.setZero(qp._num_var);
		_A1.setZero(qp._num_cons, qp._num_var);
		_lbA1.setZero(qp._num_cons);
		_ubA1.setZero(qp._num_cons);
		_lb1.setZero(qp._num_var);
		_ub1.setZero(qp._num_var);
		// set cost function x^T*H*x + x^T*g
		_H1.noalias() =  Phi.transpose()* Q * Phi + R;
		// _H1.noalias() =  Phi.transpose()* Q * Phi;
		_g1 =  2*Phi.transpose()*Q*(F*X + Phi*U_old + V - Y_ref);


		qp.UpdateMinProblem(_H1,_g1);

		_A1 = Phi;
		_eq_min_constraint = min_constraint - F*X - V - Phi*U_old;
		_eq_max_constraint = max_constraint - F*X - V - Phi*U_old;

		for (int i = 0; i < qp._num_cons; i++)
		{
			_lbA1(i) = _eq_min_constraint(i); 
			_ubA1(i) = _eq_max_constraint(i);
		}
		qp.UpdateSubjectToAx(_A1, _lbA1, _ubA1);  
		// // torque limit
		for (int i = 0; i < qp._num_var; i++){ 
			_lb1(i) =  -1000.0 ;
			_ub1(i) =  1000.0 ;
		}
		qp.UpdateSubjectToX(_lb1, _ub1); 
		// //Solve
		qp.EnableEqualityCondition(0.0001);
		// qp.EnablePrintOptionDebug(); // qpOASES check debug
		qp.SolveQPoases(max_iter);
		_opt_u = qp._Xopt;
		U = U_old + _opt_u;
		
		X_next = A_d * X + B_d * U.segment(0,7) + D_d;
		// _prev_torque = U_old.segment(0,7);
		
		for (int i=0; i<Np; i++){
			U_old.segment(7 * i,7) = U.segment(0,7);
		}
		_pred_torque = U.segment(0,7);
		_prev_F = X.segment(19,6);
		prev_q = X.segment(0,7);
		freq_check=0;
		// cout << "Q_p : "<<"\n"<<Q_p.block<19,19>(0,0)<<endl;
	}
	freq_check++;

	for (int i = 0; i < 7; i++)
	{
		_lpf_torque(i) = CustomMath::LowPassFilter(_dt, 2.0 * PI * 8.0, _pred_torque(i), _pre_torque(i)); // 8 ~ 10
		_pre_torque(i) = _lpf_torque(i);
	}
	_torque = _lpf_torque ;

	cout << "_lpf_F_esti : "<<_lpf_F_esti.transpose()<<endl;
	// _xdes_inter =  CustomMath::LinearInterpolation(X, X_next, _t+0.001, _mpc_init_t, _mpc_init_t+dT).segment(7,6);
	// _xdot_des_inter =  CustomMath::LinearInterpolation(X, X_next, _t+0.001, _mpc_init_t, _mpc_init_t+dT).segment(13,19);
	// _torque = _J_T_hands*_lambda*(400*(_xdes_inter-_x_hand) + 40*(_xdot_des_inter-_xdot_hand)) + Model._bg;// + _J_T_hands*_lpf_F_esti;

	for (int i=0; i< 6; i++){
			log(i) = _x_hand(i);
			log(i+6) = Y_ref(i);
	}
	for (int i=0; i< 7; i++){
			log(i+12) = _q(i);
	}
	for (int i=0; i< 6; i++){
			log(i+19) = _lpf_FTdata(i);
	}
	fout.open("/home/kist/euncheol/Dual-arm/data/Sim_data/panda_qddot.txt",ios::app);
	fout << log.transpose() <<endl;
	fout.close();
}
void CController::Initialize()
{	
	std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/panda_qddot.txt");
    _control_mode = 1; //1: joint space, 2: task space(CLIK)

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
	_Jdot_hands.setZero();
	_pre_J_hands.setZero();
	_pre_Jdot_hands.setZero();
	_Jdot_qdot.setZero();

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
	// HandTrajectory.set_size(6);
	_A_diagonal.setZero(dof,dof);

	_x_err_hand.setZero(6);
	_R_des_hand.setZero();

	_xdot_err_hand.setZero(6);

	_Id_6.setIdentity();
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

	_error_i.setZero();
	_qddot_des.setZero();
	_qddot_goal.setZero(dof);
	_qddot_init.setZero(dof);
	_torque_null.setZero();
}
void CController::Initialize_MPC()
{	
	// std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/comp_clik.txt");
	Eigen::VectorXd param;
	param.setZero(6);
    ParticleYamlRead("../libs/data/param.yaml", param);

	Np = 10;
	dT = 0.01;
	loop = 10;
	freq_check = loop;
	_state_size = dof*1 + 6*3; // dof =7 / total : 25
	X.setZero(_state_size);
	X_next.setZero(_state_size);

	A_d.setZero(_state_size, _state_size);
	B_d.setZero(_state_size, dof);
	D_d.setZero(_state_size);

	C_d.setZero(12,_state_size); 

	temp_V.setZero(_state_size*Np);
	V.setZero(12*Np);

	F.setZero(12*Np, _state_size);
	temp_phi.setZero(12*Np, _state_size);
	temp_phi_B.setZero(12*Np, 7);
	Phi.setZero(12*Np, 7*Np);


	Q.setZero(12*Np, 12*Np);
	Q_temp.setZero(12, 12);

	for(int i=0; i<3; i++){
		Q_temp(i,i) = param(0);			// end-effector pos weight
		Q_temp(i+3,i+3) = param(1);			// end-effector rot weight
	}
	for(int i=0; i<3; i++){
		Q_temp(i+6,i+6) = param(2);			// end-effector vel weight
		Q_temp(i+9,i+9) = param(3);			// end-effector angular vel weight
	}

	for(int i=0; i<Np; i++){
		Q.block<12,12>(12*i, 12*i) = Q_temp; 
	}
	cout << Q_temp <<endl<<endl;
	R.setZero(7*Np, 7*Np);
	for(int i=0; i<dof*Np; i++){
		R(i,i) = param(5);
	}  
	cout << R.block<7,7>(0,0) <<endl<<endl;
	Y_temp.setZero(Np*12);
	Y_des.setZero(Np*12);
	Y_ref.setZero(Np*12);// Y_ref is used to calculate MPC costs 

	_const_min.setZero(12);
	_const_max.setZero(12);
	_const_min.segment(0,6) = Model._min_ee_pos; 	// min ee pose
	_const_min.segment(6,6) = Model._min_ee_vel;	// min ee vel

	_const_max.segment(0,6) = Model._max_ee_pos;	// max ee pose
	_const_max.segment(6,6) = Model._max_ee_vel;	// max ee vel
	min_constraint.setZero(12*Np);
	max_constraint.setZero(12*Np);			
	_eq_min_constraint.setZero(12*Np);
	_eq_max_constraint.setZero(12*Np);

	for(int i=0; i<Np; i++)
	{
		min_constraint.segment(i*12,12) = _const_min;
		max_constraint.segment(i*12,12) = _const_max;
	}

	U_old.setZero(dof*Np);
	U.setZero(dof*Np);

	a = true;
	Check.setZero(dof);
	end = true;

	pred_q.setZero(7);
	pred_qdot.setZero(7);
	// log.setZero(19);
	log.setZero(25);
	// log.setZero(12);
	// log.setZero(14);
	
	_opt_u.setZero();
	FTdata.setZero();

	_qdes_inter.setZero();
	_qdotdes_inter.setZero();
	_torque_inter.setZero();
	_prev_torque.setZero();
	_pred_torque.setZero();
	_xdes_inter.setZero();

	taskfirst = true;
	_Null.setZero();
	Y_temp.setZero();

	_pre_x_goal_hand.setZero();
	b_100hz_t = true;

	_F_esti.setZero();
	_K_esti.setZero();
	_filtered_F_esti.setZero();
	_pre_torque.setZero();
	_lpf_torque.setZero();
	prev_q.setZero(7);
	_prev_F.setZero(6);


}