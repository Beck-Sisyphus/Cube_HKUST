#include <Arduino.h>
#include <math.h>
#include "pinout.h"
#include <Wire.h>
// #include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "MPU6050_6Axis_MotionApps20.h"
#include "maxon.h"
#include <SoftwareSerial.h>
#include <String.h>

#define MAXON_ON false
#define DEBUG false
#define MEGA_ADK true

// Maxon DEC 50/5 motor
Maxon maxon;

// Encoder serial
#if !MEGA_ADK
  SoftwareSerial encoderSerial(SOFT_RX, SOFT_TX);
#endif

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
float input_current;

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
	#if MAXON_ON
	maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_FEEDBACK, P_MAXON_STATUS);
	maxon.setMode(SPEED_MODE_SLOW);
	#else
	nidec_motor_init();
	#endif

	// TODO3: Test the servo

	// TODO4: Test the encoder
    #if MEGA_ADK
    Serial1.begin(38400);
    #else
    encoderSerial.begin(38400);
    #endif

	// Final TODO5: implement the LQR controller, online or offline

}

#if MAXON_ON
int i = 0;
bool notflip = false;
#endif

void loop()
{
	// wait for MPU interrupt or extra packet(s) available
	while ((!mpuInterrupt_1 && fifoCount_1 < packetSize_1) \
		|| (!mpuInterrupt_2 && fifoCount_2 < packetSize_2)) {

		#if MAXON_ON
			// Set up Maxon motor
			if (i >= 255) { notflip = false;}
			else if ( i <= 0 ) { notflip = true; }
			if (notflip) { i++; }
			else { i--; }

			maxon.enable();
			maxon.setMotor(i);

			wheel_angle_dot = maxon.getSpeedFeedback();

			Serial.print(i);
		#else
      int encoder_feedback = 0;
      String incomeByte;
			// Get info from encoder Serial
      #if MEGA_ADK
      if (Serial1.available()) {
        incomeByte = Serial1.readStringUntil('\r');
      }
      #else
      if (encoderSerial.available()) {
        incomeByte = encoderSerial.readStringUntil('\r');
      }
      #endif
      encoder_feedback = incomeByte.toInt();
      wheel_angle_dot = (float)(2 * M_PI * encoder_feedback * 100) / 1024;


			// Set Nidec motor speed
			input_current = 200;
			nidec_speed(input_current);
		#endif

    //#if DEBUG
    Serial.print(encoder_feedback);
		Serial.print("\t");
		Serial.println(wheel_angle_dot);
    //#endif
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
	body_angle_dot = (float)(gx2 * degreeToRadian) / (float)GYRO_SENSITIVITY;
	#if DEBUG
	Serial.print("measured body angle: \t");
	Serial.println(body_angle);
	Serial.print("measured body angle velocity: \t");
  Serial.print(gx2);
  Serial.print("\t");
	Serial.println(body_angle_dot);
  #endif
//  delay(100);
}
