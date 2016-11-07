#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "constant.h"

int main(int argc, char const *argv[]) {
	// Constant* c;
	double ang_vel_w_jump_sqrt = 0;
	// ang_vel_w_jump = (2 - sqrt(2)) * (c.Iw + c.Ib + c.mw * (c.l)^2) \
	//  				  * (c.mb*c.lb + c.mw*c.l) * c.g / (c.Iw)^2;
	ang_vel_w_jump_sqrt = ang_vel_w_jump();

	printf("%f\n", ang_vel_w_jump_sqrt);
	return 0;
}
