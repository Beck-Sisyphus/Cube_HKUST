#include "Maxon.h"

//Constructor
Maxon::Maxon()
{
    started = false;
}

//Setup this object, must call before using other fuctions
void Maxon::begin(int in1, int in2, int dir, int en, int spd, int rdy, int feedback, int led)
{
    started = true;
    p_in1 = in1;
    p_in2 = in2;
    p_dir = dir;
    p_en = en;
    p_spd = spd;
    p_rdy = rdy;
    p_feedback = feedback;
    p_led = led;
    tar_dir_positive = true;
    real_dir_positive = true;

    //Setup pins
    pinMode(p_in1, OUTPUT);
    pinMode(p_in2, OUTPUT);
    pinMode(p_dir, OUTPUT);
    pinMode(p_en, OUTPUT);
    pinMode(p_spd, OUTPUT);
    pinMode(p_rdy, INPUT);
    pinMode(p_feedback, INPUT);
    pinMode(p_led, OUTPUT);

    //Set defaults
    setMode(SPEED_MODE_OPEN);
    setLEDDir(LED_SOURCE);
    disable();
    setMotor(0);
}

//Sustain status led reading
void Maxon::sustain()
{
    if (started)
        digitalWrite(p_led, !(digitalRead(p_rdy) ^ led_dir));
}

//Drives the motor at the given speed
void Maxon::setMotor(int speed)
{
    if (started)
    {
        if (speed > 0)
        {
            enable();
            digitalWrite(p_dir, HIGH);
            analogWrite(p_spd, speed);
            tar_dir_positive = true;
        }
        else if (speed < 0)
        {
            enable();
            speed = speed*-1;
            digitalWrite(p_dir, LOW);
            analogWrite(p_spd, speed);
            tar_dir_positive = false;
        }
        else
        {
            disable();
            analogWrite(p_spd, 0);
            tar_dir_positive = true;
        }
    }
}

// TODO: Check the open loop value
void Maxon::setMotorCurrent(float current)
{
    int command_value;
    float speedInRev = current * CURRENT_TO_REV;
    switch (p_mode) {
        case SPEED_MODE_OPEN:
            command_value = (int)speedInRev > MAX_RPM_FAST ? MAX_PWM : (int)MAX_PWM * speedInRev / MAX_RPM_FAST;
        break;
        case SPEED_MODE_SLOW:
            command_value = (int)speedInRev > MAX_RPM_SLOW ? MAX_PWM : (int)MAX_PWM * speedInRev / MAX_RPM_SLOW;
        break;
        case SPEED_MODE_MED:
            command_value = (int)speedInRev > MAX_RPM_MID  ? MAX_PWM : (int)MAX_PWM * speedInRev / MAX_RPM_MID ;
        break;
        case SPEED_MODE_FAST:
            command_value = (int)speedInRev > MAX_RPM_FAST ? MAX_PWM : (int)MAX_PWM * speedInRev / MAX_RPM_FAST;
        break;
    }
    setMotor(command_value);
}

//Enable the motor drive output
void Maxon::enable()
{
    if (started)
        digitalWrite(p_en, HIGH);
}

//Disable the motor drive output
void Maxon::disable()
{
    if (started)
        digitalWrite(p_en, LOW);
}

//Sets the direction of the status LED feature (default is 1)
void Maxon::setLEDDir(int dir)
{
    led_dir = dir;
}

//Set the speed mode (see definitions in header file)
void Maxon::setMode(int mode)
{
    if (started)
    {
        p_mode = mode;
        switch (p_mode) {
            case SPEED_MODE_OPEN:
                digitalWrite(p_in1, LOW);
                digitalWrite(p_in2, LOW);
            break;
            case SPEED_MODE_SLOW:
                digitalWrite(p_in1, HIGH);
                digitalWrite(p_in2, LOW);
            break;
            case SPEED_MODE_MED:
                digitalWrite(p_in1, LOW);
                digitalWrite(p_in2, HIGH);
            break;
            case SPEED_MODE_FAST:
                digitalWrite(p_in1, HIGH);
                digitalWrite(p_in2, HIGH);
            break;
        }
        // digitalWrite(p_in1, mode & 1);
        // digitalWrite(p_in2, mode & 2);
    }
}

// Get the calculated motor speed in radian.s^-1
// Input:  frequency from timer 4 in rpm
// Output: speed n = f * 20 / z_pol, and turn rpm to radian.s^-s
float Maxon::getSpeedFeedback(unsigned int sampled_frequency)
{
    // Check if a direction switch is made
    if (sampled_frequency < 101) {
        real_dir_positive = tar_dir_positive;
    }
    int z_pol = 8; // number of pole pairs of motor
    float freq_in_Revolution = sampled_frequency * 20 / z_pol;
    float freq_in_Radian = freq_in_Revolution * REV_TO_RADIAN;
    if (!real_dir_positive) {
        freq_in_Radian = -1 * freq_in_Radian;
    }
    return freq_in_Radian;
}

bool Maxon::getTargetDirection()
{
    return tar_dir_positive;
}
