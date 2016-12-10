#include "Cube_Controller.h"
#define PID_Type

#define Angle_offset -0.116
#define Angle_dot_offset 0.15
#define Trust_Value 0.1

float State_now[3];              // State used in Controller
float State_bar[3];              // State measured from IMU and Ecoder;
float State_prev[3];             // State used in Controller
float Voltage_now;
int First_Flag=0;
float Angle_final;
float Angle_final_prev;
int Kd_Counter;
float Angle_dot_Kd;
float Angle_dot_Kd_now;
float Kd_Gain,Kp_Gain;

#ifdef  PID_Type
    float Kp,Kd,Ki;
    float Hook_Value;
#endif

void Cube_Controller_SetUp ( void )
{
    
    #ifdef  PID_Type
        Hook_Value = 1.35;//adjust strength
        Kp = 600;//adjust range 50~400
        Kd = 1500; //2000~30000
        Ki = 0;
        //1.35  6000  750
    #endif
}

int Cube_Controller( float body_angle, float body_angle_dot, float wheel_angle_dot )
{
    float Angle_gyro=0;
    State_bar[0] = body_angle - Angle_offset;
    State_bar[1] = body_angle_dot - Angle_dot_offset;
    State_bar[2] = wheel_angle_dot;
    //Filter
    if (First_Flag ==0 )
    {
        Angle_final = State_bar[0];
        First_Flag = 1;
    }
    Angle_gyro = Angle_final - State_bar[1]*0.001;
    Angle_final = State_bar[0] * Trust_Value + Angle_gyro * (1 - Trust_Value);
    #ifdef  PID_Type
       if (Kd_Counter<=1)
       {
            //Angle_dot_Kd = (Angle_dot_Kd  -Angle_gyro +Angle_final_prev)/2;
            Angle_dot_Kd = State_bar[1]*0.001;
            Kd_Counter++;
       }
       if (Kd_Counter ==1)
       {
            Angle_dot_Kd_now = Angle_dot_Kd;
            Kp_Gain = Kp * (-Angle_final);
            Kd_Gain = Kd * Angle_dot_Kd_now;
            Kd_Counter=0;
       }
       // Voltage_now = Kp * (-State_bar[0] + Angle_offset) + Kd * (State_bar[1]- Angle_dot_offset);
       if (Kp_Gain<=5&&Kp_Gain>=-5) maxon.disable();  //Kp_Gain=0; remy
       else if (Kp_Gain > 5)
       {
          maxon.enable(); //remy
          Kp_Gain = (Kp_Gain - 5)*Hook_Value;
       }
       else if (Kp_Gain < -5)
       {
          maxon.enable();  //remy
          Kp_Gain = (Kp_Gain + 5)*Hook_Value;
       }
       Voltage_now =  Kp_Gain + Kd_Gain;
    #endif
    State_prev[0] = State_bar[0];
    State_prev[1] = State_bar[1];
    State_prev[2] = State_bar[2];
    if (Voltage_now >= 255) { Voltage_now = 255; }
    if (Voltage_now <= -255) { Voltage_now = -255; }
  //Serial.print(Angle_final,4);
  Serial.print((int)(Kp_Gain));
  Serial.print(",");
  //Serial.print(Angle_dot_Kd_now,4);
  Serial.print((int)(Kd_Gain));
  //Serial.print(",");
  //Serial.print((int)Voltage_now);
  Serial.print(",");
  Serial.print((int)(encoder0Pos)*10);
  Serial.print("\r\n");
  encoder0Pos = 0;
    Angle_final_prev = Angle_final;
    return (int)Voltage_now;
}

