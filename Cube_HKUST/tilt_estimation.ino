// Nov.21st, 2016 The method recorded in the first paper, IROS2012
// Derived for the specific mechanics for our project
float const_g = 9.81;
float radius_1 = 0.180; //meter
uint8_t tilt_estimation(MPU6050 mpu1, MPU6050 mpu2, float *estimated_angle, float *estimated_angle_dot_dot)
{
	VectorInt16 a1; //[x, y, z] in raw measurements
	VectorInt16 a2;
	a1.x = mpu1.getAccelerationX();
	a1.y = mpu1.getAccelerationY();
	a1.z = mpu1.getAccelerationZ();
	a2.x = mpu2.getAccelerationX();
	a2.y = mpu2.getAccelerationY();
	a2.z = mpu2.getAccelerationZ();
	int16_t a2_tan = ( a2.x + a2.y) / sqrt(2); // tangential acceleration
	int16_t a2_rad = (-a2.x + a2.y) / sqrt(2); // radical acceleration
	int16_t mx =  a1.x - a2_rad;
	int16_t my = -a1.y - a2_tan;
	*estimated_angle = atan2(-my , -mx);
	*estimated_angle_dot_dot = -const_g * (a1.y / 8192 + sin(*estimated_angle)) / radius_1;
	// Serial.println(a1.y);
	return 0;
}
