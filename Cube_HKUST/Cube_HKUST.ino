#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "pinout.h"
#include <Wire.h>
// #include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "MPU6050_6Axis_MotionApps20.h"
#include <maxon.h>

// Maxon DEC 50/5 motor
Maxon maxon;

MPU6050 mpu1;
MPU6050 mpu2(0x69);

	// MPU control/status vars
	bool dmpReady_1 = false;  // set true if DMP init was successful
	uint16_t packetSize_1;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount_1;     // count of all bytes currently in FIFO
	bool dmpReady_2 = false;  // set true if DMP init was successful
	uint16_t packetSize_2;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount_2;     // count of all bytes currently in FIFO

	VectorInt16 aa1;         // [x, y, z]            accel sensor measurements
	VectorInt16 aa2;         // [x, y, z]            accel sensor measurements
	int32_t gyro1[3];
	int32_t gyro2[3];

float body_angle,  body_angle_dot;
float wheel_angle, wheel_angle_dot;

// Interrupt Detection Routine
volatile bool mpuInterrupt_1 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady_1() {
    mpuInterrupt_1 = true;
}
volatile bool mpuInterrupt_2 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady_2() {
    mpuInterrupt_2 = true;
}


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

	dmpReady_1 = test_dmp_connection(mpu1, packetSize_1);
	dmpReady_2 = test_dmp_connection(mpu2, packetSize_2);

	attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN0), dmpDataReady_1, RISING);
	attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN1), dmpDataReady_2, RISING);

	// Calibrate both MPU-6050 in acelX acelY acelZ gyroX gyroY gyroZ
	setMPUofffset(mpu1, -2761,  -1161, 864, 91,  -2,  -7); //
	setMPUofffset(mpu2, -2743,  -283,  747, 42,  -32, 24);//


	// Initialize the motor, Nidec and Maxon;
	// nidec_motor_init();
	maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_FEEDBACK, P_MAXON_STATUS);
	maxon.setMode(SPEED_MODE_SLOW);
	maxon.setLEDDir(LED_SOURCE);
	maxon.disable();

	// TODO3: Test the servo

	// TODO4: Test the encoder

	// Final TODO5: implement the LQR controller, online or offline

	// configure LED for debugging
	pinMode(LED_PIN, OUTPUT);
}

void loop()
{
	// wait for MPU interrupt or extra packet(s) available
	while ((!mpuInterrupt_1 && fifoCount_1 < packetSize_1) \
			|| (!mpuInterrupt_2 && fifoCount_2 < packetSize_2)) {
		// Set up Maxon motor
		double motor_speed = 0; // TODO: turn unit to radian.s^-1
		maxon.enable();
		maxon.setMotor((int)motor_speed);

		wheel_angle_dot = maxon.getSpeedFeedback();
		Serial.println(wheel_angle_dot);

		// Set Nidec motor speed
		// nidec_speed(20);
	}

	// if programming failed, don't try to do anything
	if (dmpReady_1) {
		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt_1 = false;

		process_mpu_data(mpu1, packetSize_1, fifoCount_1, 1, &aa1, gyro1);
	}

	if (dmpReady_2) {

		mpuInterrupt_2 = false;

		process_mpu_data(mpu2, packetSize_2, fifoCount_2, 2, &aa2, gyro2);
	}

	// TODO: build a new library depends on MPU6050_6Axis_MotionApps20 to get the accurate angle
	// Get the calculated angle and angle dot dot
	tilt_estimation(&aa1, &aa2, &body_angle);

	// TODO: Add a Jacobian for frame transformation
	// For right now assume mpu2 give a simular reading for angular velocity
	int16_t gx2 = mpu2.getRotationX();
	body_angle_dot = gx2 * degreeToRadian / GYRO_SENSITIVITY;
	Serial.print("measured body angle: \t");
	Serial.println(body_angle);
	Serial.print("measured body angle velocity: \t");
	Serial.println(body_angle_dot);
}
