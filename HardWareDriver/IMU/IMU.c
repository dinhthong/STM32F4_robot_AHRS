/***************************
https://github.com/BobLiu20/IMU/blob/master/BaseDrive/IMU.c
https://code.google.com/archive/p/imumargalgorithm30042010sohm/downloads -> AHRS.zip

AHRS implentation is the same as:
https://resources.oreilly.com/examples/0636920021735/blob/master/ch16/16_10/AHRS.cpp
 
 
ax,ay,az is used by both AHRS update.
halfT is calculated only in IMU_AHRSupdate but used by both functions.

 User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.

 Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
 orientation.  See my report for an overview of the use of quaternions in this application.

 User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
 accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
 radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
 
yaw clockwise -> yaw increase
roll so quad moves to right -> roll decrease
pitch so quad moves to front -> pitch increase
 */

#include "IMU.h"
#define DATA_SIZE 200

float offset_gx,offset_gy,offset_gz;
float offset_ax, offset_ay, offset_az;
// for loop in getValues, same variables
float offset_gyro[3];

volatile double halfT ,elapsedT;
volatile uint32_t lastUpdate, now;


// variables for Madgwick Mahony AHRS.

volatile float exInt, eyInt, ezInt;  
volatile float integralFBx, integralFBy, integralFBz;
volatile float q0, q1, q2, q3, qa0, qa1, qa2, qa3;
volatile float integralFBhand, handdiff;

volatile float acc_vector = 0;  //  M/S^2
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw;

// variables for quaternion based Kalman Filter
volatile float q_0, q_1, q_2, q_3,w1,w2,w3; // È«¾ÖËÄÔªÊý

float P[49]={0.0001,0,0,0,0,0,0,
	        0,0.0001,0,0,0,0,0,
			0,0,0.0001,0,0,0,0,
			0,0,0,0.0001,0,0,0,
			0,0,0,0,0.0002,0,0,
			0,0,0,0,0,0.0002,0,
			0,0,0,0,0,0,0.0002};

  float Q[49]={0.0001,0,0,0,0,0,0,
               0,0.0001,0,0,0,0,0,
			   0,0,0.0001,0,0,0,0,
			   0,0,0,0.0001,0,0,0,
			   0,0,0,0,0.0005,0,0,		 
			   0,0,0,0,0,0.0005,0,	 
			   0,0,0,0,0,0,0.0005} ;  
			    
float R[36]={0.0003,0,0,0,0,0,
               0,0.0003,0,0,0,0,
			   0,0,0.0003,0,0,0,
			   0,0,0,0.0002,0,0,
			   0,0,0,0,0.0002,0,
			   0,0,0,0,0,0.0002} ;	
			   		
float A[49],B[49],E[42],F1[36],X[49],Z[49],Ht[42],Ft[49],K[42],O[49],T[6],F[49],Y[7],P1[49],U1[36],U1t[36],D1[36],X1[36],X2[36];
float H[42]={
			   0,0,0,0,0,0,0,
			   0,0,0,0,0,0,0,
			   0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,													   												  
			   0,0,0,0,0,0,0,
							  };
float I[49]={1,0,0,0,0,0,0,
               0,1,0,0,0,0,0,
			   0,0,1,0,0,0,0,
			   0,0,0,1,0,0,0,
			   0,0,0,0,1,0,0,
			   0,0,0,0,0,1,0,
			   0,0,0,0,0,0,1
			   };

// Fast inverse square-root
/**************************ʵ�ֺ���********************************************

*******************************************************************************/
float invSqrt(float x)
{

	volatile float halfx = 0.5f * x;
	volatile float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*********************************************************************
Initialise MPU6050 and HMC5883, configure gyro and acc readings.

*******************************************************************************/

void IMU_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // I2C
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// DRY 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	MPU6050_initialize();
	HMC5883L_SetUp();
	delay_ms(20);
		//get raw offset.
//	
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
#define new_weight 0.4f
#define old_weight 0.6f
/*
Get values from sensors
MPU6050 returns 6 values.
0, 1, 2 are accelemeter data
3 4 5 are gyro data.

*/
static int16_t MPU6050_raw[6];
void IMU_getValues(float *values)
{
	int16_t accgyroval[6];
	static  volatile float lastacc[3] = {0, 0, 0};
	uint8_t i;
	//
	MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	// added to watch the 'raw' values
	for (i=0;i<6;i++)
	{
	MPU6050_raw[i]=accgyroval[i];
	}
	
	for(i = 0; i < 6; i++)
	{
		if(i < 3)
		{
			// Complementary Filter for new accelemeter data from MPU6050
			values[i] = (float) accgyroval[i] * new_weight + lastacc[i] * old_weight ;
			lastacc[i] = values[i];
		}
		else
		{
			// raw to real deg/sec -> *2000/32767 = 1/16.4
			values[i] = (float) accgyroval[i] / 16.4f; 
		}
	}
	/* pass the address of values[6], after this function finishes, values[6],[7],[8] are filled with mag data
	*/
	HMC58X3_mgetValues(&values[6]);	//

}

/***************************************************************************
IMU_data is used by all function
*******************************************************************************/
float acc_z_comp;
float acc_x,acc_y,acc_z;
static float IMU_data[9];
void IMU_getAttitude(float *RPY,float *RPY_2,float *rate_RPY ,float *RPY_Kalman, float *cf_accZ)
{
	/* This function to get the scaled values from GY-86 should be call here once. 
	used by all algorithms
	*/

	IMU_getValues(IMU_data);
	/*
	time elapsed should be calculated right after IMU data is read
	*/
	now = micros(); 
	if(now < lastUpdate) 
	{
		elapsedT =  (float)(now + (0xffffffff - lastUpdate));
	}
	else
	{
		elapsedT =  (now - lastUpdate);
	}
	//convert us to second
	  elapsedT = elapsedT * 0.000001f;
	  halfT = elapsedT / 2.0f;
	  lastUpdate = now;
	
	  simple_imu(RPY, rate_RPY);
	
	  IMU_getRollPitchYaw(RPY_2);
	  RPY_2[1] = -RPY_2[1];
	  RPY_2[2] = -RPY_2[2];

	  KalmanAHRS_getRollPitchYaw(RPY_Kalman);
	  RPY_Kalman[1] = -RPY_Kalman[1];
	  RPY_Kalman[2] = -RPY_Kalman[2];
		  *cf_accZ = *cf_accZ*0.974 + 0.036*(MPU6050_raw[2] - offset_az);
	  *cf_accZ/=2;

}
// imu code brokking.
/**/
void simple_imu(float *RPY,float *rate_RPY) {
		float gx,gy,gz;
    static float rateroll,ratepitch,rateyaw;
	  float gyroXrate,gyroYrate;
    double angle_roll_acc,angle_pitch_acc;
	  float acc_length;
    double kalAngleX, kalAngleY;
	  acc_x = IMU_data[0];
    acc_y = IMU_data[1];
    acc_z = IMU_data[2];
    gx = MPU6050_raw[3] - offset_gx;
    gy = MPU6050_raw[4] - offset_gy;
    gz = MPU6050_raw[5] - offset_gz;

    rateroll = LPF(gx,rateroll,10,elapsedT);
    ratepitch = LPF(gy,ratepitch,10,elapsedT);
    rateyaw = LPF(gz,rateyaw,10,elapsedT);
	
		rate_RPY[0] = rate_RPY[0] * 0.85f + rateroll * 0.15f/16.4f;
	  rate_RPY[1] = rate_RPY[1] * 0.85f + ratepitch * 0.15f/16.4f;
	  rate_RPY[2] = rate_RPY[2] * 0.85f + rateyaw * 0.15f/16.4f;
	
    gyroXrate = gx / 16.4f;
    gyroYrate = gy / 16.4f;

	  /*
		angle += rate * elapsedTime * ( sensor conversion coeffient )
		T*2000/32767 = ...
	  */
    RPY[0] += rateroll * elapsedT * 2000/32767;// 
    RPY[1] += ratepitch * elapsedT * 2000/32767;
    RPY[2] += rateyaw * elapsedT * 2000/32767;
    //(1/250/65.5)*pi/180=0.000001066
		// elapsedT * ( sensor conversion coeffient ) * M_PI/180.0f
    RPY[1]-= RPY[0]*sin(rateyaw * elapsedT * 2000/32767 * M_PI/180);
    RPY[0]+= RPY[1]*sin(rateyaw * elapsedT * 2000/32767 * M_PI/180);

    acc_length = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
    if(fabs(acc_x) < acc_length) {
        angle_pitch_acc=-asin((float)acc_x/acc_length)*57.296f;
    }
    if(fabs(acc_y) < acc_length) {
        angle_roll_acc=asin((float)acc_y/acc_length)*57.296f;
    }

    kalAngleX = Kalman_getAngle_roll(angle_roll_acc,gyroXrate, elapsedT);
    kalAngleY = Kalman_getAngle_pitch(angle_pitch_acc,gyroYrate, elapsedT);

    RPY[1] = RPY[1] * 0.975f +  kalAngleY * 0.025f;
    RPY[0] =  RPY[0] * 0.975f +   kalAngleX * 0.025f ;
		
//		acc_z_comp = acc_z_comp*0.9 + acc_z*0.1;
//		*rate_z += acc_z_comp*elapsedT;
}

	
void Initialize_Q(void)
{
	float acc[9];
	int i;
	volatile float temp, roll, pitch, yaw, yh, xh;
	volatile float  acc_X, acc_Y, acc_Z, acc_MX, acc_MY, acc_MZ; //
	
	// initialize quaternion
	q0 = 1.0f;  //
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	qa0 = 1.0f;  //
	qa1 = 0.0f;
	qa2 = 0.0f;
	qa3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	integralFBx = 0.0;
	integralFBy = 0.0;
	integralFBz	= 0.0;
	/*
    Get DATA_SIZE raw samples and calculate the average.
		Calculate the initial quaternion values using traditional method. 
	*/
	for(i = 0; i < DATA_SIZE; i++) {
		IMU_getValues(acc);
		acc_X += acc[0];
		acc_Y += acc[1];
		acc_Z += acc[2];
		acc_MX += acc[6];
		acc_MY += acc[7];
		acc_MZ += acc[8];
	}
	acc_X /= DATA_SIZE;
	acc_Y /= DATA_SIZE;
	acc_Z /= DATA_SIZE;
	acc_MX /= DATA_SIZE;
	acc_MY /= DATA_SIZE;
	acc_MZ /= DATA_SIZE;

	temp = acc_X * invSqrt((acc_Y * acc_Y + acc_Z * acc_Z));
	pitch = atan(temp) * 57.3;

	temp = acc_Y * invSqrt((acc_X * acc_X + acc_Z * acc_Z));
	roll = atan(temp) * 57.3;

	yh = acc_MY * cos(roll) + acc_MZ * sin(roll);
	xh = acc_MX * cos(pitch) + acc_MY * sin(roll) * sin(pitch) - acc_MZ * cos(roll) * sin(pitch);
	yaw = atan2(yh, xh);
	// Initial quaternion values
	q0 = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q1 = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q2 = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
	q3 = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
	// halfT(lastUpdate,...)
//	lastUpdate = micros();
}

/****************************************************************
Requirements:
previous q0...q3
Based on MahonyAHRS

*******************************************************************************/
#define Kp 3.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.03f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az, volatile float mx, volatile float my, volatile float mz)
{

	volatile  float norm;
	volatile float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz, wx, wy, wz;
	volatile float ex, ey, ez;
	volatile float temp0, temp1, temp2, temp3;
	volatile float temp;

	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	temp = sqrt(ax * ax + ay * ay + az * az);
	temp = (temp / 16384.0f) * 9.8f;   //M/S^2
	acc_vector = acc_vector +   //��ͨ�˲�����ֹƵ��20hz
		     (halfT * 2.0f / (7.9577e-3f + halfT * 2.0f)) * (temp - acc_vector);
	
	// Normalise accelerometer measurement
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	//�ü��ٶȼ���roll��pitch
	//	temp = ax * invSqrt((ay * ay + az * az));
	//	ACC_Pitch = atan(temp)* 57.3;
	//
	//	temp = ay * invSqrt((ax * ax + az * az));
	//	ACC_Roll = atan(temp)* 57.3;
// Normalise magnetometer measurement
	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
	
	// Reference direction of Earth's magnetic field
	// compute reference direction of flux

	hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
	hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
	hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
	/*

	*/
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

	// estimated direction of gravity and flux (v and w)

	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	/*

	*/
	wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
	wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
	wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2);

	// error is sum of cross product between reference direction of fields and direction measured by sensors

	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	/*

	*/
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		// adjusted gyroscope measurements

		gx = gx + (Kp * ex + exInt);
		gy = gy + (Kp * ey + eyInt);
		gz = gz + (Kp * ez + ezInt);

	}

	// integrate quaternion rate and normalise
	// ��Ԫ��΢�ַ���
	temp0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	temp1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	temp2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	temp3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	// normalise quaternion
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	q0 = temp0 * norm;
	q1 = temp1 * norm;
	q2 = temp2 * norm;
	q3 = temp3 * norm;
}
/*
Calculate qa0...qa3
*/

#define twoKpDef  (1.0f ) // 2 * proportional gain
#define twoKiDef  (0.2f) // 2 * integral gain

void FreeIMU_AHRSupdate(volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az)
{
	volatile float norm;
	//  float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz;
	volatile float ex, ey, ez;
	volatile float temp0, temp1, temp2, temp3;

	// �Ȱ���Щ�õõ���ֵ���
	volatile float q0q0 = qa0 * qa0;
	volatile float q0q1 = qa0 * qa1;
	volatile float q0q2 = qa0 * qa2;
	volatile float q0q3 = qa0 * qa3;
	volatile float q1q1 = qa1 * qa1;
	volatile float q1q2 = qa1 * qa2;
	volatile float q1q3 = qa1 * qa3;
	volatile float q2q2 = qa2 * qa2;
	volatile float q2q3 = qa2 * qa3;
	volatile float q3q3 = qa3 * qa3;

	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	// estimated direction of gravity and flux (v and w)
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay * vz - az * vy) ;
	ey = (az * vx - ax * vz) ;
	ez = (ax * vy - ay * vx) ;

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{

		integralFBx +=  ex * twoKiDef * halfT;
		integralFBy +=  ey * twoKiDef * halfT;
		integralFBz +=  ez * twoKiDef * halfT;

		gx = gx + twoKpDef * ex + integralFBx;
		gy = gy + twoKpDef * ey + integralFBy;
		gz = gz + twoKpDef * ez + integralFBz;

	}
	// integrate quaternion rate and normalise
	temp0 = qa0 + (double)(-qa1 * gx - qa2 * gy - qa3 * gz) * halfT;
	temp1 = qa1 + (double)(qa0 * gx + qa2 * gz - qa3 * gy) * halfT;
	temp2 = qa2 + (double)(qa0 * gy - qa1 * gz + qa3 * gx) * halfT;
	temp3 = qa3 + (double)(qa0 * gz + qa1 * gy - qa2 * gx) * halfT;

	// normalise quaternion
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	qa0 = temp0 * norm;
	qa1 = temp1 * norm;
	qa2 = temp2 * norm;
	qa3 = temp3 * norm;
}

/****************************************************************************

Get Q using AHRS update
*******************************************************************************/
//float IMU_values[9];
void IMU_getQ(float *q,volatile float IMU_values[9])
{
	// deg to radian
	IMU_AHRSupdate(IMU_values[3] * M_PI / 180, IMU_values[4] * M_PI / 180, IMU_values[5] * M_PI / 180,
		       IMU_values[0], IMU_values[1], IMU_values[2], IMU_values[6], IMU_values[7], IMU_values[8]);

	FreeIMU_AHRSupdate(IMU_values[3] * M_PI / 180, IMU_values[4] * M_PI / 180, IMU_values[5] * M_PI / 180,
			   IMU_values[0], IMU_values[1], IMU_values[2]);

	q[0] = qa0; //���ص�ǰֵ	FreeIMU_AHRSupdate �����������Ԫ�� ���õ�
	q[1] = qa1;
	q[2] = qa2;
	q[3] = qa3;
}
void IMU_getRollPitchYaw(float *angles)
{
	static float q[4];

	IMU_getQ(q, IMU_data); 
	/*
	 Quaternion To Euler function 
	*/

	IMU_Roll = angles[0] = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
				      1 - 2.0f * (q[1] * q[1] + q[2] * q[2]))) * 180 / M_PI;
	//angles[2]-=0.8f;
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	
	IMU_Pitch = angles[1] = -safe_asin(2.0f * (q[0] * q[2] - q[3] * q[1])) * 180 / M_PI;
	IMU_Yaw = angles[2] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) * 180 / M_PI; // yaw
}


void Kalman_AHRS_init(void)
{
	//Initial_Timer3();
//	MPU6050_initialize();
//	HMC5883L_SetUp();
//	delay_ms(50);

  	//ÍÓÂÝÒÇÆ«²î
	w1=0;//0.095f;
	w2=0;//0.078f;
	w3=0;//-0.014f;
	
//  	lastUpdate = micros();//¸üÐÂÊ±¼ä
//  	now = micros();

    q_0=1.0;
    q_1=0;
    q_2=0;
    q_3=0;
}

void KalmanAHRS_getRollPitchYaw(float * angles) {
  float q[4]; //¡¡ËÄÔªÊý
  
  KalmanAHRS_getQ(q, IMU_data); //¸üÐÂÈ«¾ÖËÄÔªÊý
//	angles[2]=
//  angles[2] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw 
//  angles[1] = -safe_asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
//  angles[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	
		IMU_Roll = angles[0] = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
				      1 - 2.0f * (q[1] * q[1] + q[2] * q[2]))) * 180 / M_PI;
	//angles[2]-=0.8f;
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	
	IMU_Pitch = angles[1] = -safe_asin(2.0f * (q[0] * q[2] - q[3] * q[1])) * 180 / M_PI;
	IMU_Yaw = angles[2] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) * 180 / M_PI;
	
 // if(angles[2]<2)angles[2]+=360.0f;  //½« -+180¶È  ×ª³É0-360¶È
}

void KalmanAHRS_getQ(float * q,volatile float IMU_values[9]) {
   Kalman_AHRSupdate(IMU_values[3] * M_PI/180, IMU_values[4] * M_PI/180, IMU_values[5] * M_PI/180,
   IMU_values[0], IMU_values[1], IMU_values[2], IMU_values[6], IMU_values[7], IMU_values[8]);
     
   q[0] = q_0; //·µ»Øµ±Ç°Öµ
   q[1] = q_1;
   q[2] = q_2;
   q[3] = q_3;
}
// Kalman

void Kalman_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  float bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float g=9.79973;
  float Ha1,Ha2,Ha3,Ha4,Hb1,Hb2,Hb3,Hb4;
  float e1,e2,e3,e4,e5,e6;
//  float halfT;

// ÏÈ°ÑÕâÐ©ÓÃµÃµ½µÄÖµËãºÃ
  float q0q0 = q_0*q_0;
  float q0q1 = q_0*q_1;
  float q0q2 = q_0*q_2;
  float q0q3 = q_0*q_3;
  float q1q1 = q_1*q_1;
  float q1q2 = q_1*q_2;
  float q1q3 = q_1*q_3;
  float q2q2 = q_2*q_2;   
  float q2q3 = q_2*q_3;
  float q3q3 = q_3*q_3;      
  //Ê¯¼Ò×¯µØÇø´Å³¡ 
  bx = 0.5500;
  bz = 0.8351; 
//  now = micros();  //¶ÁÈ¡Ê±¼ä
//  if(now<lastUpdate){ //¶¨Ê±Æ÷Òç³ö¹ýÁË¡£
//  halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);
//  }
//  else	{
//  halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//  }
//  lastUpdate = now;	//¸üÐÂÊ±¼ä
  norm = invSqrt(ax*ax + ay*ay + az*az);       					  //¹éÒ»»¯
  ax = ax * norm*g;
  ay = ay * norm*g;
  az = az * norm*g;

  norm = invSqrt(mx*mx + my*my + mz*mz);          					  //¹éÒ»»¯
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;
  
  gx=gx-w1;gy=gy-w2;gz=gz-w3;					 /////////////////		¼õÈ¥ÍÓÂÝÒÇÆ«²î
	
  Ha1=(-q_2)*g; Ha2=q_3*g; Ha3=-q_0*g; Ha4=q_1*g;	 
  Hb1=bx*q_0-bz*q_2;
  Hb2=bx*q_1+bz*q_3;//
  Hb3=-bx*q_2-bz*q_0;
  Hb4=-bx*q_3+bz*q_1;
  
  H[0]= Ha1;H[1]= Ha2;H[2]= Ha3;H[3]= Ha4;
  H[7]= Ha4;H[8]=-Ha3;H[9]= Ha2;H[10]=-Ha1;
  H[14]=-Ha3;H[15]=-Ha4;H[16]= Ha1;H[17]= Ha2;
  
  H[21]= Hb1;H[22]= Hb2;H[23]= Hb3;H[24]= Hb4;      
  H[28]= Hb4;H[29]=-Hb3;H[30]= Hb2;H[31]=-Hb1;
  H[35]=-Hb3;H[36]=-Hb4;H[37]= Hb1;H[38]= Hb2;
  //×´Ì¬¸üÐÂ
  q_0 = q_0 + (-q_1*gx - q_2*gy - q_3*gz)*halfT;
  q_1 = q_1 + (q_0*gx + q_2*gz - q_3*gy)*halfT;
  q_2 = q_2 + (q_0*gy - q_1*gz + q_3*gx)*halfT;
  q_3 = q_3 + (q_0*gz + q_1*gy - q_2*gx)*halfT;  
    // ËÄÔªÊý¹éÒ»
  norm = invSqrt(q_0*q_0 + q_1*q_1 + q_2*q_2 + q_3*q_3);
  q_0 = q_0 * norm;
  q_1 = q_1 * norm;
  q_2 = q_2 * norm;
  q_3 = q_3 * norm;
//FÕó¸³Öµ
F[0]=1;F[8]=1;F[16]=1;F[24]=1;F[32]=1;F[40]=1;F[48]=1;
F[1]=-gx*halfT;F[2]=-gz*halfT;F[3]=-gz*halfT;	F[4]=0; F[5]=0; F[6]=0;
F[7]=gx*halfT;F[9]=gz*halfT;F[10]=-gy*halfT;F[11]=0; F[12]=0; F[13]=0;
F[14]=gy*halfT;F[15]=-gz*halfT;F[17]=gx*halfT;F[18]=0; F[19]=0;F[20]=0;
F[21]=gz*halfT;F[22]=gy*halfT;F[23]=-gx*halfT;F[25]=0; F[26]=0; F[27]=0;
F[28]=0;F[29]=0;F[30]=0;F[31]=0;F[33]=0;F[34]=0;
F[35]=0;F[36]=0;F[37]=0;F[38]=0;F[39]=0;F[41]=0;
F[42]=0;F[43]=0;F[44]=0;F[45]=0;F[46]=0;F[47]=0;
 //¿¨¶ûÂüÂË²¨
 MatrixMultiply(F,7,7,P,7,7,A );	//A=F*P
 MatrixTranspose(F,7,7,Ft);	  //F×ªÖÃ  F'
 MatrixMultiply(A,7,7,Ft,7,7,B); // B=F*P*F'
 MatrixAdd( B,Q,P1,7,7 );
 MatrixTranspose(H,6,7,Ht);	  //F×ªÖÃ  F'
 MatrixMultiply(P1,7,7,Ht,7,6,E );   //E=P*H'
 MatrixMultiply(H,6,7,E,7,6,F1 ); //	 F1=H*P*H'	6*6
 MatrixAdd(F1,R,X,6,6 );           //X=F1+R	   6*6
 UD(X,6,U1,D1);	   //XµÄUD·Ö½â
 MatrixTranspose(U1,6,6,U1t);	 //U1µÄ×ªÖÃ
 MatrixMultiply(U1,6,6,D1,6,6,X1); //X1=U1*D1
 MatrixMultiply(X1,6,6,U1t,6,6,X2); //X2=U1*D1*U1t 
 MatrixInverse(X2,6,0);	 //XÄæ 
 MatrixMultiply(E,7,6,X2,6,6,K ); //ÔöÒæK   7*6

  vx = 2*(q1q3 - q0q2)*g;
  vy = 2*(q0q1 + q2q3)*g;
  vz = (q0q0 - q1q1 - q2q2 + q3q3)*g;
           
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  e1=ax-vx;e2=ay-vy;e3=az-vz;
  e4=mx-wx;e5=my-wy;e6=mz-wz;
 T[0]=e1;T[1]=e2;T[2]=e3;T[3]=e4;T[4]=e5;T[5]=e6;
 MatrixMultiply(K,7,6,T,6,1,Y );   //Y=K*(Z-Y)	7*1
 q_0= q_0+Y[0];
 q_1= q_1+Y[1];
 q_2= q_2+Y[2];
 q_3= q_3+Y[3];
 w1= w1+Y[4];
 w2= w2+Y[5];
 w3= w3+Y[6];

  
 MatrixMultiply(K,7,6,H,6,7,Z); //Z= K*H		7*7
 MatrixSub(I,Z,O,7,7 );	  //O=I-K*H
 
 MatrixMultiply(O,7,7,P1,7,7,P);
 
  // normalise quaternion
  norm = invSqrt(q_0*q_0 + q_1*q_1 + q_2*q_2 + q_3*q_3);
  q_0 = q_0 * norm;
  q_1 = q_1 * norm;
  q_2 = q_2 * norm;
  q_3 = q_3 * norm;
}


float LPF(float x, float pre_value, float CUTOFF,float dt)
{
    float RC, alpha, y;
    RC = 1.0f/(CUTOFF*2*3.1416f);
    alpha = dt/(RC+dt);
    y = pre_value + alpha * ( x - pre_value );
    return y;
}
// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0f;
	}
	if (v >= 1.0f)
	{
		return M_PI / 2;
	}
	if (v <= -1.0f)
	{
		return -M_PI / 2;
	}
	return asin(v);
}

