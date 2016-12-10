#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <String.h>
#include "pinout.h"
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

#define encoder0PinA 2
#define encoder0PinB 3
volatile unsigned int encoder0Pos = 0;

int16_t ax1, ay1, az1, gx1, gy1, gz1;
int16_t ax2, ay2, az2, gx2, gy2, gz2;

float body_angle, body_angle_dot;
float body_angle_LPF = 0;
float wheel_angle_dot;
float input_current;
const float alpha_body_angle = 0.3;

unsigned volatile int countTimer = 0;
unsigned volatile int countHz = 0;

ISR(TIMER4_OVF_vect)
{
  countHz = countTimer;
  countTimer = 0;
  TCNT4 = 0xB1DF;
}

// Capture the rising edge
ISR(TIMER4_CAPT_vect)
{
  countTimer++;
}

float get_Serial() {
  String inString = "";
  //  Serial.println("Input New Value:");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  delay(1);
  while (Serial.available() > 0 ) {
    inString += (char)Serial.read();
  }
  return inString.toFloat();
}

extern float Kp,Kd,Ki;
extern float Hook_Value;
float mortor_speed_freg;
void setup()
{
  // Join I2C bus, open up serial to computer
  Wire.begin();
  Serial.begin(115200);
/*  Serial.println("Input New Value:");
  switch (Serial.read())
  {
  case 'h': Hook_Value = get_Serial(); Serial.println(Hook_Value, 4);break;
  case 'p': Kp = get_Serial(); Serial.println(Kp, 4);break;
  case 'i': Ki = get_Serial(); Serial.println(Ki, 4);break;
  case 'd': Kd = get_Serial(); Serial.println(Kd, 4);break; 
  default: break;
  }
  while (!Serial.available());*/
  // Initialize MPU, attach interrupt to pin 2
  // Initialize two MPU in the same buss
  mpu1.initialize();
  mpu2.initialize();

  // verify connection
//  Serial.println("Testing device connections...");
//  Serial.println(mpu1.testConnection() ? "MPU6050 1 connection successful" : "MPU6050 1 connection failed");
//  Serial.println(mpu2.testConnection() ? "MPU6050 2 connection successful" : "MPU6050 2 connection failed");

  // Calibrate both MPU-6050 in acelX acelY acelZ gyroX gyroY gyroZ
  setMPUofffset(mpu1, -2761,  -1161, 864, 91,  -2,  -7); //
  setMPUofffset(mpu2, -2743,  -283,  747, 42,  -32, 24);//

  // Initialize the motor, Nidec and Maxon;
  maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_FEEDBACK, P_MAXON_STATUS);
  maxon.setMode(SPEED_MODE_OPEN);
  maxon.enable();

  // TODO3: Test the servo

  // TODO4: Test the encoder
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, RISING);
  // Final TODO5: implement the LQR controller, online or offline
  Cube_Controller_SetUp();

  // MPU initial update
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  
  // Debugging timer
  pinMode(P_MAXON_STATUS, OUTPUT);
  digitalWrite(P_MAXON_STATUS, LOW);

  // Set up the timer
  //  Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  //  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  pinMode(ICP4, INPUT);
  pinMode(7, OUTPUT);

  // initialize Timer4
  cli();         // disable global interrupts
  TCCR1A = 0;    // set entire TCCR1A register to 0
  TCCR1B = 0;    // set entire TCCR1B register to 0 
  
  // count from 45535, for 16-bit counter, 45535 to 65535 get 20000 count
  // minus 1480 more to compensate the calculation time, 0.74 ms
  TCNT1 = 0xEC7A;
  // Set CS10 bit so timer runs at clock speed: 100Hz, 0.01 sec
  // Prescaling: clk/8, CS11 set to 1
  TCCR1B |= (1 << CS11);  // Similar: TCCR1B |= _BV(CS11);

  // and input capture interrupt enable
  TIMSK1 |= (1 << TOIE1); 
  
  TCCR4A = 0;    // set entire TCCR1A register to 0
  TCCR4B = 0;    // set entire TCCR1B register to 0 
  
  ICR4H = 0;
  ICR4L = 0;

  // count from 45535, for 16-bit counter, 45535 to 65535 get 20000 count
  TCNT4 = 0xB1DF;
  
  // Set CS10 bit so timer runs at clock speed: 100Hz, 0.01 sec
  // Prescaling: clk/8, CS11 set to 1
  TCCR4B |= (1 << CS41);  // Similar: TCCR1B |= _BV(CS11);
  //  TCCR4B |= (1 << CS42);  // Similar: TCCR1B |= _BV(CS11);
  
  // ICES1 Input Capture Edge Select flag, 1 for rising, 0 for falling
  TCCR4B |= (1 << ICES4);
  
  // and input capture interrupt enable
  TIMSK4 |= (1 << ICIE4); 
  // enable Timer overflow interrupt:
  TIMSK4 |= (1 << TOIE4); 
  
  // enable global interrupts:
  sei();
  delay(1000);
}

void loop()
{
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
}

ISR(TIMER1_OVF_vect)
{
  digitalWrite(7, HIGH);
  TCNT1 = 0xEC7A;
  body_angle = tilt_estimation(ax1, ax2, ay1, ay2);
  body_angle_LPF = (1 - alpha_body_angle) * body_angle_LPF + alpha_body_angle * body_angle;
  body_angle_dot = (float)(gz2 * 133.0) / 100000.0;
  //wheel_angle_dot = maxon.getSpeedFeedback(countHz * 100);
  input_current = Cube_Controller(body_angle,body_angle_dot,wheel_angle_dot);
  maxon.setMotor(input_current);
  //maxon.setMotor(mortor_speed_freg);
  digitalWrite(7, LOW);
}

void doEncoderA()
{
    // check channel B to see which way encoder is turning
      if (digitalRead(encoder0PinB) == LOW) 
     {  
       encoder0Pos = encoder0Pos + 1;         // CW
      } 
      else 
      {
       encoder0Pos = encoder0Pos - 1;         // CCW
      }
}

