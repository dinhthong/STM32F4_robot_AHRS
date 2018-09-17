#ifndef __IMU_H
#define __IMU_H
#include "common.h"
#include <math.h>
//#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Matrix.h"
#define M_PI  (float)3.1415926535
#define micros() TIM5->CNT
float safe_asin(float v);
extern float IMU_values[9];
extern int16_t MPU6050_raw[6];
extern volatile float IMU_Pitch, IMU_Roll, IMU_Yaw, ACC_Pitch, ACC_Roll;	 //µ¥Î» ¶È

extern volatile float IMU_GYROx, IMU_GYROy, IMU_GYROz;	
extern volatile float acc_vector;  
//Mini IMU AHRS 
void IMU_getValues(float *values);
void IMU_init(void);

void Initialize_Q(void);
void IMU_getQ(float *q,volatile float IMU_values[9]);
void IMU_getRollPitchYaw(float *angles);

void Kalman_AHRS_init(void);
void KalmanAHRS_getQ(float * q,volatile float IMU_values[9]);
void Kalman_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void KalmanAHRS_getRollPitchYaw(float * angles);

float LPF(float x,float pre_value, float CUTOFF,float dt);
void IMU_getAttitude(float *RPY, float *RPY_2,float *rate_RPY,float *RPY_Kalman, float *cf_accZ);
void simple_imu(float *RPY, float *rate_RPY);

#endif

