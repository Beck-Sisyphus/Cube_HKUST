#include "maxon.h"

// Maxon DEC 50/5 motor
Maxon maxon;
//Maxon motor controller pins, running in 24V, PWM controlled
#define P_MAXON_SPEED 44 //5 // PWM pin, pin 26 on the drive
#define P_MAXON_DIR 42//6  // pin 23 on the drive
#define P_MAXON_EN  40//7  // pin 22
#define P_MAXON_IN2 38//8 // pin 21
#define P_MAXON_IN1 36//9 // pin 20
#define P_MAXON_READY 34//10// pin 19
#define P_MAXON_FEEDBACK 32//A0 // pin 18

#define P_MAXON_STATUS 13

#define SPEED_MODE_OPEN 0
#define SPEED_MODE_SLOW 2
#define SPEED_MODE_MED 1
#define SPEED_MODE_FAST 3

void setup() {
    Serial.begin(115200);
    // while (!Serial.available()) ;

    maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_FEEDBACK, P_MAXON_STATUS);
    // pinMode(P_MAXON_IN1, OUTPUT);
//    pinMode(P_MAXON_IN2, OUTPUT);
//    pinMode(P_MAXON_DIR, OUTPUT);
//    pinMode(P_MAXON_EN, OUTPUT);
//    pinMode(P_MAXON_SPEED, OUTPUT);
//    pinMode(P_MAXON_READY, INPUT);
//    pinMode(P_MAXON_FEEDBACK, INPUT);
//    pinMode(P_MAXON_STATUS, OUTPUT);
    maxon.setMode(SPEED_MODE_OPEN);
    // maxon.setLEDDir(LED_SOURCE);
    // maxon.start();
    // maxon.enable();
//    digitalWrite(P_MAXON_EN, HIGH); 
//    digitalWrite(P_MAXON_DIR, HIGH);
//    digitalWrite(P_MAXON_IN1, HIGH);
//    digitalWrite(P_MAXON_IN2, HIGH);
    delay(100);
    if (digitalRead(P_MAXON_READY)) {
        Serial.println("Maxon initialized");
    }
}

int i = 0;
bool notflip = true;
void loop() {
    // Set up Maxon motor
//    if (i >= 255) {
//      notflip = false;
//    }  else if ( i <= 0 ) {
//      notflip = true;
//    }
//    if (notflip) {
//      i++;
//    } else {
//      i--;
//    }
    // TODO: turn unit to radian.s^-1
    maxon.enable();
    // maxon.setMotor(i);
    maxon.setMotor(127);

    float wheel_angle_dot = maxon.getSpeedFeedback();
    
    Serial.print(i);
    Serial.print("\t");
    Serial.println(wheel_angle_dot);
    delay(100);
}
