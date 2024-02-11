#ifndef KALMAN_AHRS_H
#define KALMAN_AHRS_H
#include "common.h"
#include <math.h>
#include <stdlib.h>
#include "Matrix.h"

void Kalman_AHRS_init(void);
void KalmanAHRS_getQ(float * q,volatile float IMU_values[9]);
void Kalman_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void KalmanAHRS_getRollPitchYaw(float * angles,float IMU_data[]);

#endif

