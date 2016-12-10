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
// Parameter: filtered acceleration from IMU1 and IMU2
// Return:  estimated angle and angular acceleration for the plane
// TODO: principle components analysis for finding the rotational plane

//uint8_t tilt_estimation(int16_t *ax1, int16_t *ax2, int16_t *ay1, int16_t *ay2, float *estimated_angle)
float tilt_estimation(int16_t ax1, int16_t ax2, int16_t ay1, int16_t ay2, float body_angle_pre)
{
    float estimated_angle;
    int32_t mx = -ax1 - imu_1_to_imu_2 * ax2 / 10;
    int32_t my = -ay1 - imu_1_to_imu_2 * ay2 / 10;
    if (my == 0) {
        estimated_angle = body_angle_pre;
    } else
    {
	      estimated_angle = atan2(mx, -my);
    }
    return estimated_angle;
}

float Angle_offset = -0.116;
//float Angle_dot_offset =  -0.11011;
float Angle_dot_offset;
float Trust_Value = 0.1;
bool First_IMU_Flag = true;
// Complementary filter
// For two accelerometers estimated angle and gyroscope
float complementaryFilteredAngle(float body_angle, float body_angle_dot)
{
	float Angle_final;
	float body_angle_f = body_angle - Angle_offset;
	float body_angle_dot_f = body_angle_dot - Angle_dot_offset;
	//Filter
	if (First_IMU_Flag)
	{
		Angle_final = State_bar[0];
		First_IMU_Flag = false;
	}
	float Angle_gyro = Angle_final - body_angle_dot_f*0.001;
	Angle_final = body_angle_f * Trust_Value + Angle_gyro * (1 - Trust_Value);
	return Angle_final;
}
