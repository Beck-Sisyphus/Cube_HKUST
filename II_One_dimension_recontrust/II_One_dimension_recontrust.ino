#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <String.h>
#include "pinout.h"
#include "imu.h"
#include "FilterChLp2.h"
#include "MPU6050.h"
#include "helper_3dmath.h"
#include "maxon.h"
#include "Cube_Controller.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define DEBUG false

// Maxon DEC 50/5 motor
Maxon maxon;

MPU6050 mpu1;
MPU6050 mpu2(0x69);

int16_t ax1, ay1, az1, gx1, gy1, gz1;
int16_t ax2, ay2, az2, gx2, gy2, gz2;

float body_angle, body_angle_LPF, body_angle_filtered;
float body_angle_dot, body_angle_dot_LPF;
float wheel_angle_dot, wheel_angle_dot_filtered;
float input_current;
const float alpha_body_angle = 0.2;
const float alpha_body_angle_dot = 0.2;

FilterChLp2 body_angle_dot_Filter;
FilterChLp1_100Hz wheel_angle_dot_Filter;

unsigned volatile int countTimer = 0;
unsigned volatile int countHz = 0;
int velocity_set_point = 0;

void setup()
{
	// Join I2C bus in 100kHz
	Wire.begin(); // In Beck's Macbook Air, the I2C bus is set to 400kHz
	Serial.begin(9600);

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
    //Serial1.begin(38400);

	// Final TODO5: implement the LQR controller, online or offline
	Cube_Controller_SetUp();

    // MPU initial update
    mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);


    // Set up the timer as edge detection
    pinMode(ICP4, INPUT);

    // disable global interrupts
    cli();

    enableTimer1Interrupt();
    enableTimer4EdgeDetection();

    // enable global interrupts:
    sei();
    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
}
unsigned long timeIntoI2C;
unsigned long timeOutI2C;
void loop()
{
//    timeIntoI2C = micros();
//    Serial.println(timeIntoI2C - timeOutI2C);
    mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
//    timeOutI2C = micros();
//    Serial.println(timeOutI2C - timeIntoI2C);
//    Serial.print(body_angle_dot * 100);
//    Serial.print( " " );
    //Serial.print(input_current);
    //Serial.print( " " );
    Serial.println(wheel_angle_dot_filtered);
//    Serial.println(velocity_set_point);
}


ISR(TIMER1_OVF_vect)
{
    // Clear the interrupt flag
    TCNT1 = 0xB1DF;

    // digitalWrite(7, HIGH);

    // Get the calculated angle and angle dot dot
    //  body_angle = tilt_estimation(&ax1, &ax2, &ay1, &ay2, &body_angle);
    body_angle = tilt_estimation(ax1, ax2, ay1, ay2, body_angle);

    body_angle_LPF = (1 - alpha_body_angle) * body_angle_LPF + alpha_body_angle * body_angle;

    // For right now assume mpu2 give a simular reading for angular velocity
    // TODO: filter out gyroscope raw data
    
    body_angle_dot = ((float) gz2 * gyroToRadian) / 100000.0;

    body_angle_dot_LPF = body_angle_dot_Filter.step(body_angle_dot) - gyro_offset;

    body_angle_filtered = complementaryFilteredAngle( body_angle_LPF, body_angle_dot_LPF);

    wheel_angle_dot = maxon.getSpeedFeedback(countHz * 100);

    wheel_angle_dot_filtered = wheel_angle_dot_Filter.step(wheel_angle_dot);

    input_current = Cube_LQR_Controller(body_angle_filtered, body_angle_dot_LPF, wheel_angle_dot_filtered);

    velocity_set_point = (int)input_current;

    // Set the maxon current every 20ms
//    maxon.setMotor(velocity_set_point);
    maxon.setMotor(-40);

    // digitalWrite(7, LOW);
}

ISR(TIMER4_OVF_vect)
{
    TCNT4 = 0xB1DF;
    countHz = countTimer;
    countTimer = 0;
}

// Capture the rising edge
ISR(TIMER4_CAPT_vect)
{
    countTimer++;
}
