// Combined function to tune a MPU 6050
void setMPUofffset(MPU6050 mpu,int acelX,int acelY,int acelZ,int gyroX,int gyroY,int gyroZ) {
	mpu.setXAccelOffset(acelX);
	mpu.setYAccelOffset(acelY);
	mpu.setZAccelOffset(acelZ);
	mpu.setXGyroOffset(gyroX);
	mpu.setYGyroOffset(gyroY);
	mpu.setZGyroOffset(gyroZ);
}

// Nov.21st, 2016 The method recorded in the first paper, IROS2012
// Derived for the specific mechanics for our project
// The IMU1 in the upper y-axis points down, x points left
// The IMU2 in the lower y-axis points up,   x points right
// Get the angular acceleration from a weighted sum
// Parameter: filtered acceleration from IMU1 and IMU2
// Return:  estimated angle and angular acceleration for the plane
// TODO: principle components analysis for finding the rotational plane
//uint8_t tilt_estimation(int16_t *ax1, int16_t *ax2, int16_t *ay1, int16_t *ay2, float *estimated_angle)
float tilt_estimation(int16_t ax1, int16_t ax2, int16_t ay1, int16_t ay2)
{
    float mx = -ax1 - (float) (imu_1_radius / imu_2_radius) * ax2;
    float my = -ay1 - (float) (imu_1_radius / imu_2_radius) * ay2;
//	  estimated_angle = (float)atan2(mx, -my);
    return (float)atan2(mx, -my);
}
