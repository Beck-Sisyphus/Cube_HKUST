#include "Cube_Controller.h"
//#define LQR_Type
#define PID_Type

#define Angle_offset -0.12
#define Angle_dot_offset -0.11011
#define Trust_Value 0.1

float State_now[3];              // State used in Controller
float State_bar[3];              // State measured from IMU and Ecoder;
float State_prev[3];             // State used in Controller
float Voltage_now;
int First_Flag=0;
float Angle_final;
float Angle_final_prev;
#ifdef  LQR_Type
    /* Low Pass Filter */
    float State_LPF, State_LPF_Next=0;// Apply LPF in Tilt Angle
    #define  Alpha_LPF 0.02             // 0 Means NO Filter, Typical Value could be 0.02.
    /* LQR Control */
    float Kd[3];
    float u_now;
#endif

#ifdef  PID_Type
    float Kp,Kd;
#endif

void Cube_Controller_SetUp ( void )
{
    #ifdef  LQR_Type
        Kd[0] = -182.7363;
        Kd[1] = -18.0346;
        Kd[2] = -0.6265;
    #endif
    #ifdef  PID_Type
        Kp = 300;
        Kd = 5000;
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


    #ifdef  LQR_Type
        //Low Pass Filter Get Value
        State_LPF = State_LPF_Next;
        //Set State_now
        State_now[0] = State_bar[0] - State_LPF - Angle_offset;//Low Pass Filter
        State_now[1] = State_bar[1];
        State_now[2] = State_bar[2];
        //Implement of LQR
        u_now = Kd[0]*State_now[0] + Kd[1]*State_now[1] + Kd[2]*State_now[2];
        /*if (u_now >= 0)
        {
        Voltage_now = (u_now*1.419 +4)*(u_now*1.419 +4);
        }
        else
        {
        Voltage_now = -(u_now*1.419 -4)*(u_now*1.419 -4);
        }*/
        //Low Pass Filter Updata
        State_LPF_Next = (1- Alpha_LPF)*State_LPF + Alpha_LPF*State_bar[0];
    #endif
    #ifdef  PID_Type
       // Voltage_now = Kp * (-State_bar[0] + Angle_offset) + Kd * (State_bar[1]- Angle_dot_offset);
       Voltage_now = Kp * (-Angle_final) + Kd * (-Angle_final +Angle_final_prev);
    #endif
    State_prev[0] = State_bar[0];
    State_prev[1] = State_bar[1];
    State_prev[2] = State_bar[2];
    Angle_final_prev = Angle_final;
    if (Voltage_now >= 255) { Voltage_now = 255; }
    if (Voltage_now <= -255) { Voltage_now = -255; }
    return (int)Voltage_now;
}

