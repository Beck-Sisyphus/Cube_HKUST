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
            digitalWrite(p_dir, HIGH);
            analogWrite(p_spd, speed);
        }
        else if (speed < 0)
        {
            speed = speed*-1;
            digitalWrite(p_dir, LOW);
            analogWrite(p_spd, speed);
        }
        else
            digitalWrite(p_spd, 0);
    }
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
        digitalWrite(p_in1, mode & 1);
        digitalWrite(p_in2, mode & 2);
    }
}

// Get the calculated motor speed in Hz
// n = f * 20 / z_pol, z_pol = number of pole pairs of motor = 3
float Maxon::getSpeedFeedback()
{
    int z_pol = 3; // number of pole pairs of motor
    int H_time = pulseIn(p_feedback, HIGH);
    int L_time = pulseIn(p_feedback, LOW);
    float T_time = H_time + L_time; // in microseconds
    float freq = 1000000 / T_time;
    float freq_in_Hz = freq * 20 / z_pol;
    return freq_in_Hz;
}
