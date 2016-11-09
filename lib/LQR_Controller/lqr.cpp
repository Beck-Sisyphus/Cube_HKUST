#include "lqr.h"

// Constructor
LQR::LQR()
{
	A = Identity(3,3);
	B = Identity(3,1);
	Q << 1, 0, 0,
		 0, 0, 0,
		 0, 0, 1;
	R = Identity(3,3);
}

//compute the optimal LQR control policy based on the plant model
//returns K that are used by reference
void LQR::lqr_gains()
{
	// TODO: to solve the algebraic Riccati equation using libcontrol

    //gains
    K = (B.transpose()*Pk_1*A) / (R+B.transpose()*Pk_0*B);
}


//compute the controller output
Matrix3d LQR::lqr_feedback()
{
    //LQR with setpoint tracking
    return u = -K * x;
}
