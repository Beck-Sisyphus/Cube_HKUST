#ifndef CONSTANT_H
#define CONSTANT_H

#include <math.h>

//Maxon motor controller pins, running in 24V, PWM controlled
#define P_MAXON_SPEED 5 // PWM pin, pin 26 on the drive
#define P_MAXON_DIR 6  // pin 23 on the drive
#define P_MAXON_EN 7  // pin 22
#define P_MAXON_IN2 8 // pin 21
#define P_MAXON_IN1 9 // pin 20
#define P_MAXON_READY 10// pin 19
#define P_MAXON_FEEDBACK A0 // pin 18

#define P_MAXON_STATUS 13

// Nidec Motor, running in 12V, PWM controlled, with stop and direction control
#define NIDEC_STOP_PIN 8 // Yellow wire, 3rd
#define NIDEC_PWM_PIN 9 // White wire, 4th
#define NIDEC_SPEED_MODE_SLOW 10 // Green wire, 5th

// MPU6050
// pinout: 3.3v, GND, A4(SDA), A5(SCL), D2(external interrupt #0 pin)
#define INTERRUPT_PIN0 2
#define INTERRUPT_PIN1 3
#define I2C_SDA A4
#define I2C_SCL A5
const int16_t GYRO_SENSITIVITY = 131; // LSB / degree/s
const float degreeToRadian = 0.0174533; // radian / degree

const int16_t imu_1_radius = 150; // millimeter
const int16_t imu_2_radius =  34; // millimeter
const int16_t IMU_SENSITIVITY = 8192;    // IMU scale for 1g

// LED for debugging
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

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
