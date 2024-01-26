#pragma once
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "custommath.h"

using namespace std;
using namespace Eigen;

class CTrajectory
{

public:
	CTrajectory();
	virtual ~CTrajectory();

	void set_size(int dof);
	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel);
	void update_time(double time);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time);
	int check_trajectory_complete();

	VectorXd position_cubicSpline();
	VectorXd velocity_cubicSpline();
	VectorXd acceleration_cubicSpline();
	
	VectorXd position_cubicSplineStack(double time_horizon, double time);
	VectorXd velocity_cubicSplineStack(double time_horizon, double time);

	VectorXd position_quinticSpline();
	VectorXd velocity_quinticSpline();
	VectorXd acceleration_quinticSpline();

	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel, VectorXd init_acc);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, VectorXd goal_acc, double goal_time);

private:
	void Initialize();
	void check_vector_size(VectorXd X);
	
	int _vector_size;
	double _dt;
	double _duration_time;
	double _time_start, _time, _time_end;	
	VectorXd _init_pos, _init_vel, _goal_pos, _goal_vel;
	VectorXd _init_acc, _goal_acc;
	
	bool _bool_trajectory_complete;
	double _motion_threshold;
	// update_ref_Y
	int _dof;
	double _time_horizon;
	VectorXd ref_x;
	VectorXd StackedXd;
	VectorXd StackedXdotd;
	double _time_h;
	double _time_hd;
	VectorXd init_posM;
	VectorXd goal_posM;
	VectorXd _b0, _b1, _b2, _b3, _b4, _b5;
};


class HTrajectory
{

public:
	HTrajectory();
	virtual ~HTrajectory();

	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel);
	void update_time(double time);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time);

	VectorXd position_cubicSpline();
	VectorXd velocity_cubicSpline();

	Matrix3d rotationCubic();
	Vector3d rotationCubicDot();

	int check_trajectory_complete();

	VectorXd position_quinticSpline();
	VectorXd velocity_quinticSpline();
	VectorXd acceleration_quinticSpline();

	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel, VectorXd init_acc);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, VectorXd goal_acc, double goal_time);

private:
	void Initialize();

	double cubic(double x_0, double x_f, double x_dot_0, double x_dot_f);
	double cubicDot(double x_0, double x_f, double x_dot_0, double x_dot_f);

	CTrajectory XTrajectory;

	double _time_start, _time, _time_end;

	Matrix3d rotation_0, rotation_f;
	VectorXd _init_pos, _init_vel;
	VectorXd _goal_pos, _goal_vel;
	VectorXd _init_acc, _goal_acc;

	Vector3d w_0, a_0;

	bool _bool_trajectory_complete;
};
#endif