// Nov.21st, 2016 The method recorded in the first paper, IROS2012
// Derived for the specific mechanics for our project
// The IMU1 in the upper y-axis points down, x points left
// The IMU2 in the lower y-axis points up,   x points right
// Get the angular acceleration from a weighted sum
// Parameter: filtered acceleration from IMU1 and IMU2
// Return:  estimated angle and angular acceleration for the plane
// TODO: principle components analysis for finding the rotational plane

uint8_t tilt_estimation(VectorInt16 *a1,VectorInt16 *a2, float *estimated_angle)
{
    #if DEBUG
    Serial.print("a1real.x\t");
  	Serial.print(a1->x);
    Serial.print("a1real.y\t");
  	Serial.print(a1->y);
    Serial.print("a2real.x\t");
  	Serial.print(-1 * a2->x);
    Serial.print("a2real.y\t");
    Serial.println(-1 * a2->y);
    #endif

    float mx = -a1->x - (float) (imu_1_radius / imu_2_radius) * a2->x;
    float my = -a1->y - (float) (imu_1_radius / imu_2_radius) * a2->y;
	*estimated_angle = (float)atan2(mx, -my);
	return 0;
}

  // int16_t a2_tan = ( a2->x + a2->y); // tangential acceleration * sqrt(2)
  // int16_t a2_rad = (-a2->x + a2->y); // radical acceleration * sqrt(2)
  // int16_t mx = a2_rad - a1->x;
  // int16_t my = a2_tan + a1->y ;
  //  float angular_acc_1 = -envi_g * ((float)a1->x / imu_g + (float)sin(*estimated_angle)) / imu_1_radius;
//    float angular_acc_2 =  envi_g * ((float)a2->x / imu_g + (float)sin(*estimated_angle)) / imu_2_radius;
//    *angular_acc = (imu_1_radius * angular_acc_1 + imu_2_radius * angular_acc_2) / (imu_1_radius + imu_2_radius);
