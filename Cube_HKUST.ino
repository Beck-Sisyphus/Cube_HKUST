#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "constant.h"
#include <Maxon.h>

// Maxon DEC 50/5 motor
Maxon maxon;

void setup()
{
	maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_STATUS);
	maxon.setMode(SPEED_MODE_OPEN);
	maxon.setLEDDir(LED_SOURCE);
	maxon.disable();
}

void loop()
{

}
/*
int main(int argc, char const *argv[]) {

	double ang_vel_w_jump_sqrt = 0;

	ang_vel_w_jump_sqrt = ang_vel_w_jump();

	return 0;
}*/
