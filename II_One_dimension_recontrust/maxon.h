/*
This will be an Arduino driver for the maxon motors 1-Q-EC Amplifier DEC Module 50/5.
Modified: Beck Pang, Nov. 8th, 2016
Credit to: Brian Eccles
*/

#ifndef MAXON_H
#define MAXON_H

#include <Arduino.h>

//Definitions for setting the controller speed mode
//Motor type -  1 pole pair     | 4 pole pair        | 8 pole pair
//     ------------------------------------------------------
//     OPEN -     Open loop speed control 0-100%
//     SLOW -     500-5,000 RPM     | 125-1,250 RPM     | 62-625 RPM
//     MED  -     500-20,000 RPM    | 125-5,000 RPM     | 62-2,500 RPM
//     FAST -     500-80,000 RPM    | 125-20,000 RPM    | 62-10,000 RPM
#define SPEED_MODE_OPEN 0
#define SPEED_MODE_SLOW 2
#define SPEED_MODE_MED 1
#define SPEED_MODE_FAST 3

//LED direction values
#define LED_SOURCE 1
#define LED_SINK 0

class Maxon
{
private:
    //Speed mode pins
    int p_in1, p_in2;
    //Direction control pin
    int p_dir;
    //Enable pin
    int p_en;
    //Speed control pin
    int p_spd;
    //Ready output pin
    int p_rdy;
    //Status led pin
    int p_led;
    // Speed feedback from DEC 50/5 in digital frequency signal
    int p_feedback;
    //LED Direction (see macros above)
    int led_dir;
    //True if begin has been called
    bool started;
    // Store the mode
    int p_mode;
    // Store the target direction, in 1 (true) or -1 (false)
    bool tar_dir_positive;
    // Store the real direction, in 1 (true) or -1 (false)
    bool real_dir_positive;

public:
    Maxon();

    //Setup this object, must call before using other functions
    void begin(int in1, int in2, int dir, int en, int spd, int rdy, int feedback, int led);

    //Sustain status led reading
    void sustain();

    //Drives the motor at the given speed
    void setMotor(int speed);
    void setMotorCurrent(float current); // TODO: Check to delete it

    //Enable or disable motor drive output
    void enable();
    void disable();

    //Sets the direction of the status lED feature (default is 1)
    void setLEDDir(int dir);

    //Set the speed mode (see definitions above)
    void setMode(int mode);

    // Get the calculated motor speed in radian.s^-1
    float getSpeedFeedback(unsigned int sampled_frequency);

    // Get if the target direction is positive
    bool getTargetDirection();

    const int MAX_RPM_SLOW = 625;
    const int MAX_RPM_MID  = 2500;
    const int MAX_RPM_FAST = 10000;
    const int MAX_PWM = 256;
    const float REV_TO_RADIAN = 0.10472;  // revolution per minutes to radian per sec
    const float RADIAN_TO_REV = 9.5493;
    const int CURRENT_TO_REV = 2125;
};

#endif
