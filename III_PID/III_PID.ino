#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <String.h>
#include "pinout.h"
#include "MPU6050.h"
#include "helper_3dmath.h"
#include "maxon.h"
#include "Cube_Controller.h"
#include "TimerOne.h"

#define DEBUG false

// Maxon DEC 50/5 motor
Maxon maxon;

MPU6050 mpu1;
MPU6050 mpu2(0x69);

int16_t ax1, ay1, az1, gx1, gy1, gz1;
int16_t ax2, ay2, az2, gx2, gy2, gz2;

float body_angle, body_angle_dot;
float body_angle_LPF = 0;
float wheel_angle_dot;
float input_current;
const float alpha_body_angle = 0.3;


void setup()
{
	// Join I2C bus, open up serial to computer
	Wire.begin();
	Serial.begin(115200);
	while (!Serial.available()) ;

	// Initialize MPU, attach interrupt to pin 2
	// Initialize two MPU in the same buss
	mpu1.initialize();
	mpu2.initialize();

	// Calibrate both MPU-6050 in acelX acelY acelZ gyroX gyroY gyroZ
	setMPUofffset(mpu1, -2761,  -1161, 864, 91,  -2,  -7); //
	setMPUofffset(mpu2, -2743,  -283,  747, 42,  -32, 24);//

	// Initialize the motor, Nidec and Maxon;
	maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_FEEDBACK, P_MAXON_STATUS);
	maxon.setMode(SPEED_MODE_OPEN);
  maxon.enable();

	// TODO3: Test the servo

	// TODO4: Test the encoder

	// Final TODO5: implement the LQR controller, online or offline
	Cube_Controller_SetUp();

  // MPU initial update
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  
  // Debugging timer
  pinMode(P_MAXON_STATUS, OUTPUT);
  digitalWrite(P_MAXON_STATUS, LOW);

	// Set up the timer
	Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
	Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  delay(1000);

}

void loop()
{
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
}

/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void timerIsr()
{
// Get the calculated angle and angle dot dot
//  body_angle = tilt_estimation(&ax1, &ax2, &ay1, &ay2, &body_angle);
  body_angle = tilt_estimation(ax1, ax2, ay1, ay2);

  body_angle_LPF = (1 - alpha_body_angle) * body_angle_LPF + alpha_body_angle * body_angle;

  input_current = Cube_LQR_Controller(body_angle,body_angle_dot,wheel_angle_dot);

	// Set the maxon current every 20ms
  maxon.setMotor(20);
}
