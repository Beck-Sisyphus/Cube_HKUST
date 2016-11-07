#ifndef CONSTANT_H
#define CONSTANT_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "constant.h"

/// All constants needed to build the model
// typedef	struct Constant {
	const double l = 0.085;  // m
	const double lb = 0.075; // m
	const double mb = 0.419; // kg
	const double mw = 0.204; // kg
	const double Ib = 3.34E-3; // kg.m^2
	const double Iw = 0.57E-3; // kg.m^2
	const double Cb = 1.02E-3; // kg.m^2.s^-1
	const double Cw = 0.05E-3; // kg.m^2.s^-1
	const double Kw = 36.9E-3;  // Nm.A^-1
	const double g = 9.81; // m.s^-2
// } Constant;

double ang_vel_w_jump()
{
	return sqrt((2 - sqrt(2)) * (Iw + Ib + mw * pow(l,2)) \
					  * (mb*lb + mw*l) * g / pow(Iw,2));
}

#endif
