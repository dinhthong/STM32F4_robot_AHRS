#ifndef KALMAN_AHRS_H
#define KALMAN_AHRS_H
#include "common.h"
#include <math.h>
//#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Matrix.h"
// #define M_PI  (float)3.1415926535
// #define micros() TIM5->CNT
// float safe_asin(float v);
// extern float IMU_values[9];
// extern int16_t MPU6050_raw[6];
// extern volatile float IMU_Pitch, IMU_Roll, IMU_Yaw, ACC_Pitch, ACC_Roll;	 //��λ ��

// extern volatile float IMU_GYROx, IMU_GYROy, IMU_GYROz;	
// extern volatile float acc_vector;  


void Kalman_AHRS_init(void);
void KalmanAHRS_getQ(float * q,volatile float IMU_values[9]);
void Kalman_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void KalmanAHRS_getRollPitchYaw(float * angles,float IMU_data[]);

#endif

