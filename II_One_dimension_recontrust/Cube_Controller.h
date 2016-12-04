#ifndef __CUBE_CONTROLLER_H
#define __CUBE_CONTROLLER_H

int Cube_Controller ( float body_angle, float body_angle_dot, float wheel_angle_dot );
float Cube_LQR_Controller( float body_angle, float body_angle_dot, float wheel_angle_dot ); 
void Cube_Controller_SetUp ( void );

#endif
