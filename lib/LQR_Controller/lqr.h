/*
This will be an Arduino library for LQR controller
Author: Beck Pang, Nov. 8th 2016
*/
#ifndef LQR_H
#define LQR_H

#include <Arduino.h>
#include <Eigen/Dense>

class LQR
{
private:
	// TODO: change the state-space model to libcontrol
	// state-space model
	Vector3d x;
	Matrix3d A;
	Vector3d B;
	// RowVector3d C;
	// double D;

	// LQR tuning parameters
	Matrix3d Q;
    Matrix3d R;

	// dynamic Riccati equation
	Matrix3d Pk_1; // P_k+1
	Matrix3d Pk_0; // P_k

	// gains
	double Kd;

	// LQR feedback
	Vector3d u;
public:
	LQR();

	// compute the lqr_gains using the LAPACK and libcontrol
	// to solve the algebraic Riccati equation
	lqr_gains();

	lqr_feedback();

	// TODO: build the Ad and Bd from the the Taylor series approach from libcontrol
	
}

#endif
