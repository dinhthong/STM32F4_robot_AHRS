#ifndef IMU_AHRS_H
#define IMU_AHRS_H
#include "common.h"
#include <math.h>
#include <stdlib.h>
#include "Matrix.h"

void Initialize_Q(void);
void FreeIMU_AHRSupdate(volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *roll,float *pitch,float *yaw);

#endif

