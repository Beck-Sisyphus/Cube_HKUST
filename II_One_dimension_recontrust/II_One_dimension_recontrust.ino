#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <String.h>
#include "pinout.h"
#include "MPU6050.h"
#include "helper_3dmath.h"
#include "maxon.h"
#include "Cube_Controller.h"

#define MAXON_ON false
#define DEBUG false

// Maxon DEC 50/5 motor
Maxon maxon;

MPU6050 mpu1;
MPU6050 mpu2(0x69);

float body_angle,  body_angle_dot;
float wheel_angle_dot;
float input_current;

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

	// verify connection
	Serial.println("Testing device connections...");
	Serial.println(mpu1.testConnection() ? "MPU6050 1 connection successful" : "MPU6050 1 connection failed");
	Serial.println(mpu2.testConnection() ? "MPU6050 2 connection successful" : "MPU6050 2 connection failed");

	// Calibrate both MPU-6050 in acelX acelY acelZ gyroX gyroY gyroZ
	setMPUofffset(mpu1, -2761,  -1161, 864, 91,  -2,  -7); //
	setMPUofffset(mpu2, -2743,  -283,  747, 42,  -32, 24);//

	// Initialize the motor, Nidec and Maxon;
	#if MAXON_ON
		maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_FEEDBACK, P_MAXON_STATUS);
		maxon.setMode(SPEED_MODE_SLOW);
	#else
		nidec_motor_init();
	#endif

	// TODO3: Test the servo

	// TODO4: Test the encoder
    Serial1.begin(38400);

	// Final TODO5: implement the LQR controller, online or offline
  Cube_Controller_SetUp();
}

void loop()
{
		#if MAXON_ON
		// Set up Maxon motor
		maxon.enable();
		maxon.setMotor(0);

		wheel_angle_dot = maxon.getSpeedFeedback();
		#else
		int encoder_feedback = 0;
		String incomeByte;
		// Get info from encoder Serial

		if (Serial1.available()) {
			incomeByte = Serial1.readStringUntil('\r');
		}

		encoder_feedback = incomeByte.toInt();
		wheel_angle_dot = (float)(2 * M_PI * encoder_feedback * 100) / 1024;

		// Set Nidec motor speed
		input_current = 200;
		nidec_speed(input_current);
		#endif

		#if DEBUG
		Serial.print(encoder_feedback);
		Serial.print("\t");
		Serial.println(wheel_angle_dot);
		#endif

	// Get the calculated angle and angle dot dot
	tilt_estimation(mpu1, mpu2, &body_angle);

	// TODO: Add a Jacobian for frame transformation
	// For right now assume mpu2 give a simular reading for angular velocity
	int16_t gx2 = mpu2.getRotationX();
	body_angle_dot = (float)(gx2 * degreeToRadian) / (float)GYRO_SENSITIVITY;
	#if DEBUG
		Serial.print("measured body angle: \t");
		Serial.println(body_angle);
		Serial.print("measured body angle velocity: \t");
		Serial.print(gx2);
		Serial.print("\t");
		Serial.println(body_angle_dot);
	#endif
  Cube_Controller (body_angle,body_angle_dot,wheel_angle_dot);
  Serial.print(body_angle,6);
  //Serial.print(body_angle_dot);
  //Serial.print(wheel_angle_dot);
  Serial.print("\r\n");
}
