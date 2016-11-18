#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "constant.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include <maxon.h>

// Maxon DEC 50/5 motor
// Maxon maxon;

// Nidec Motor, running in 12V, PWM controlled, with stop and direction control
#define STOP_PIN 7 // Yellow wire, 3rd
#define DIREC_PIN 8 // Green wire, 5th
#define NIDEC_PWM_PIN 9 // White wire, 4th

// LED for debugging
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU6050
// pinout: 3.3v, GND, A4(SDA), A5(SCL), D2(external interrupt #0 pin)
#define INTERRUPT_PIN0 2
#define INTERRUPT_PIN1 3
#define I2C_SDA A4
#define I2C_SCL A5
MPU6050 mpu1;
MPU6050 mpu2(0x69);

	// MPU control/status vars
	bool dmpReady_1 = false;  // set true if DMP init was successful
	uint16_t packetSize_1;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount_1;     // count of all bytes currently in FIFO
	bool dmpReady_2 = false;  // set true if DMP init was successful
	uint16_t packetSize_2;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount_2;     // count of all bytes currently in FIFO


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

	// Set up Maxon motor
	// maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_STATUS);
	// maxon.setMode(SPEED_MODE_OPEN);
	// maxon.setLEDDir(LED_SOURCE);
	// maxon.disable();

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

	attachInterrupt(0, dmpDataReady_1, RISING);
	attachInterrupt(1, dmpDataReady_2, RISING);

	// TODO: Calibrate both MPU-6050
	setMPUofffset(mpu1, 220, 76, -85, 1788); // XGyro, YGyro, ZGyro, ZAccel
	setMPUofffset(mpu2, 220, 76, -85, 1788); // XGyro, YGyro, ZGyro, ZAccel


	// Initialize the Nidec motor
	nidec_motor_init();

	// TODO3: Test the servo

	// TODO4: Test the encoder

	// Final TODO5: implement the LQR controller, online or offline

	// configure LED for debugging
	pinMode(LED_PIN, OUTPUT);
}

void loop()
{
	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt_1 && fifoCount_1 < packetSize_1 \
			&& !mpuInterrupt_2 && fifoCount_2 < packetSize_2) {
		// Set up Maxon motor
		// double motor_speed = 0; // in radian.s^-1
		// maxon.enable();
		// maxon.setMotor((int)motor_speed);
	}

	// if programming failed, don't try to do anything
	if (dmpReady_1) {
		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt_1 = false;

		process_mpu_data(mpu1, packetSize_1, fifoCount_1);
	}
	else if (dmpReady_2) {

		mpuInterrupt_2 = false;

		process_mpu_data(mpu2, packetSize_2, fifoCount_2);
	}
	else {
		Serial.println("MPU failed");
	}
}
