#include "robotmodel.h"
#define JDOF 7

CModel::CModel()
{
	Initialize();
}

CModel::~CModel()
{
}

void CModel::Initialize()
{
	_bool_model_update = false;
	_bool_kinematics_update = false;
	_bool_dynamics_update = false;
	_bool_Jacobian_update = false;

    _k = JDOF;
    _id_hand = 7;
	_id_ee = 7;

	_max_joint_torque.setZero(_k);
	_min_joint_torque.setZero(_k);
	_max_joint_velocity.setZero(_k);
	_min_joint_velocity.setZero(_k);
	_max_joint_position.setZero(_k);
	_min_joint_position.setZero(_k);

	_max_joint_del_torque.setZero(_k);
	_min_joint_del_torque.setZero(_k);

	_max_joint_acceleration.setZero(_k);
	_min_joint_acceleration.setZero(_k);

	_max_ee_pos.setZero(6);
	_min_ee_pos.setZero(6);
	_max_force.setZero(6);
	_min_force.setZero(6);

    _q.setZero(_k);
    _qdot.setZero(_k);
    _zero_vec_joint.setZero(_k);

    _A.setZero(_k,_k);
    _g.setZero(_k);
	_b.setZero(_k);
	_bg.setZero(_k);

    _J_hand.setZero(6,_k);
    _J_tmp.setZero(6,_k);
	_hand_ori.setZero(3,3);

    _position_local_task_hand.setZero(); // 3x1
    _x_hand.setZero(); // 3x1
    _R_hand.setZero(); // 3x3

    _xdot_hand.setZero(6);

	_max_ee_vel.setZero(6);
	_min_ee_vel.setZero(6);

	set_robot_config();
    load_model();
}


void CModel::load_model()
{   
	RigidBodyDynamics::Addons::URDFReadFromFile("../model/fr3.urdf", &_model, false, true);
	// RigidBodyDynamics::Addons::URDFReadFromFile("../model/panda_example_robot_data/panda.urdf", &_model, false, true);
    cout << endl << endl << "Model Loaded for RBDL." << endl << "Total DoFs: " << _model.dof_count << endl << endl;
	if (_model.dof_count != _k)
	{
		cout << "Simulation model and RBDL model mismatch!!!" << endl << endl;
	}

    _bool_model_update = true; //check model update

	cout << "Model Loading Complete." << endl << endl;

}
void CModel::update_kinematics(VectorXd & q, VectorXd & qdot)
{
	_q = q;
	_qdot = qdot;

	if (_bool_model_update == true)
	{
		RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q, &_qdot, NULL); // update kinematics
	}
	else
	{
		cout << "Robot model is not ready. Please load model first." << endl << endl;
	}
	_bool_kinematics_update = true; // check kinematics update
}

void CModel::update_dynamics()
{
	if (_bool_kinematics_update == true)
	{	
		RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, _q, _A, false); // update inertia matrix
		RigidBodyDynamics::InverseDynamics(_model, _q, _zero_vec_joint, _zero_vec_joint, _g, NULL); // get _g
		RigidBodyDynamics::InverseDynamics(_model, _q, _qdot, _zero_vec_joint, _bg, NULL); // get _g+_b
		_b = _bg - _g; //get _b
		for(int i =0; i < _k; i++)
		{
			_A(i,i) += 1.0;
		}
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
	_bool_dynamics_update = true; // check kinematics update
}

void CModel::calculate_EE_Jacobians()
{
	if (_bool_kinematics_update == true)
	{
		_J_hand.setZero();
		_J_tmp.setZero();	

		RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_hand, _position_local_task_hand, _J_tmp, false); // update kinematc : false
		_J_hand.block<3, 7>(0, 0) = _J_tmp.block<3, 7>(3, 0); // linear : last three entries -> first three entries
		_J_hand.block<3, 7>(3, 0) = _J_tmp.block<3, 7>(0, 0); // angular : first three entries -> last three entries

		_bool_Jacobian_update = true;
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}

}

void CModel::calculate_EE_positions_orientations()
{
    if (_bool_kinematics_update == true)
	{
		_x_hand.setZero();
		_R_hand.setZero();

		_x_hand = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_hand, _position_local_task_hand, false);
		_R_hand = RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q, _id_hand, false).transpose();

		_hand_ori = RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q, _id_ee, false).transpose();

		// cout << "_hand_ori : "<< "\n"<<_hand_ori<<endl; 

	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
}

void CModel::calculate_EE_velocity()
{
	if (_bool_Jacobian_update == true)
	{
		_xdot_hand = _J_hand * _qdot;
	}
	else
	{
		cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl << endl;
	}
}

void CModel::set_robot_config(){
	_position_local_task_hand(2) = +0.211;

	_max_joint_velocity(0) = 2.62;
	_max_joint_velocity(1) = 2.62;
	_max_joint_velocity(2) = 2.62;
	_max_joint_velocity(3) = 2.62;
	_max_joint_velocity(4) = 5.26;
	_max_joint_velocity(5) = 4.18;
	_max_joint_velocity(6) = 5.26;

	// _max_joint_velocity = _max_joint_velocity/2; // 0.5 scaling

	_min_joint_velocity = -_max_joint_velocity;

	_max_joint_position(0) = 2.7437;
	_min_joint_position(0) = -2.7437; //-0.6

	_max_joint_position(1) = 1.7837;
	_min_joint_position(1) = -1.7837;

	_max_joint_position(2) = 2.9007;

	_min_joint_position(2) = -2.9007;

	_max_joint_position(3) = -0.1518;
	_min_joint_position(3) = -3.0421;

	_max_joint_position(4) = 2.8065;
	_min_joint_position(4) = -2.8065;

	_max_joint_position(5) = 4.5169;
	_min_joint_position(5) = 0.5445;
	
	_max_joint_position(6) = 3.0159;
	_min_joint_position(6) = -3.0159;

	_max_joint_torque(0) = 87; // Units : Nm
	_max_joint_torque(1) = 87;
	_max_joint_torque(2) = 87;
	_max_joint_torque(3) = 87;
	_max_joint_torque(4) = 12;
	_max_joint_torque(5) = 12;
	_max_joint_torque(6) = 12;

	_min_joint_torque = -_max_joint_torque;

	for(int i = 0; i <_k; i++)
	{
		_max_joint_del_torque(i) = 1000; // Units : Nm/s
	}
	_min_joint_del_torque = -_max_joint_del_torque;

	_max_ee_pos(0) = 0.7;	 
	_max_ee_pos(1) = 0.5;
	// _max_ee_pos(1) = 0.08;
	_max_ee_pos(2) = 1.0;
	// _max_ee_pos(2) = 0.76;
	_max_ee_pos(3) = 3.14;
	_max_ee_pos(4) = 3.14;
	_max_ee_pos(5) = 3.14;

	_min_ee_pos(0) = - 0.7;
	_min_ee_pos(1) = -0.5;
	_min_ee_pos(2) = 0.0;
	_min_ee_pos(3) = -3.14;
	_min_ee_pos(4) = -3.14;
	_min_ee_pos(5) = -3.14;

	_max_joint_acceleration << 10, 10, 10, 10, 10, 10, 10;
	_min_joint_acceleration = -_max_joint_acceleration;

	_max_ee_vel(0) = 3.0;
	_max_ee_vel(1) = 3.0;
	_max_ee_vel(2) = 3.0;
	_max_ee_vel(3) = 2.5;
	_max_ee_vel(4) = 2.5;
	_max_ee_vel(5) = 2.5;
	_min_ee_vel = -_max_ee_vel;

	_max_force<< 1e3, 1e3, 1e3, 1e3, 1e3, 1e3;
	_min_force = -_max_force;
}
