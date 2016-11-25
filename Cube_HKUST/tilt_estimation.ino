// Nov.21st, 2016 The method recorded in the first paper, IROS2012
// Derived for the specific mechanics for our project
float const_g = 9.81;
float radius_1 = 0.180; //meter
uint8_t tilt_estimation(VectorInt16 *a1,VectorInt16 *a2, float *estimated_angle, float *estimated_angle_dot_dot)
{
	// a1.x = mpu1.getAccelerationX();
	// a1.y = mpu1.getAccelerationY();
	// a1.z = mpu1.getAccelerationZ();
	// a2.x = mpu2.getAccelerationX();
	// a2.y = mpu2.getAccelerationY();
	// a2.z = mpu2.getAccelerationZ();
  	Serial.println(a1->x);
  	Serial.println(a1->y);
  	Serial.println(a2->x);
  	Serial.println(a2->y);
	int16_t a2_tan = ( a2->x + a2->y); // tangential acceleration * sqrt(2)
	int16_t a2_rad = (-a2->x + a2->y); // radical acceleration * sqrt(2)

	int16_t mx = a2_rad - a1->x;
	int16_t my = a2_tan + a1->y ;
	*estimated_angle = atan2(-mx, my);
	*estimated_angle_dot_dot = -const_g * (a1->y / 8192 + sin(*estimated_angle)) / radius_1;
	return 0;
}
