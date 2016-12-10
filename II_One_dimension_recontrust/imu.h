#ifndef __IMU_H
#define __IMU_H

const int16_t imu_1_radius = 150; // millimeter
const int16_t imu_2_radius =  34; // millimeter
const int32_t imu_1_to_imu_2 = 44;
const int32_t IMU_SENSITIVITY = 8192;    // IMU scale for 1g
// const int16_t GYRO_SENSITIVITY = 131; // LSB / degree/s
// const float degreeToRadian = 0.017453; // radian / degree
const float gyroToRadian = 133.0; // E-6radian/s / LSB
const float REV_TO_RADIAN = 6.283185;
const float RADIAN_TO_REV = 0.159155;
const float gyro_offset = 0.15;

#endif


