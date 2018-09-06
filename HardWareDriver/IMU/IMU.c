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
float safe_asin(float v);

volatile float exInt, eyInt, ezInt;  
volatile float integralFBx, integralFBy, integralFBz;
volatile float q0, q1, q2, q3, qa0, qa1, qa2, qa3;
volatile float integralFBhand, handdiff;
volatile double halfT ,elapsedT;
volatile uint32_t lastUpdate, now;
volatile float acc_vector = 0;  //  M/S^2
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw;

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
float offset_gx,offset_gy,offset_gz;
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
	
	// DRY  - PB 13
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
//	MPU6050_Calculate_Gyro_Offset(&offset_gx,&offset_gy,&offset_gz,800);
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
static int16_t IMU_unscaled[6];
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
	IMU_unscaled[i]=accgyroval[i];
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
			values[i] = ((float) accgyroval[i]) / 16.4f; 
		}
	}
	/* pass the address of values[6], after this function finishes, values[6],[7],[8] are filled with mag data
	*/
	HMC58X3_mgetValues(&values[6]);	//

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

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.



/***************************************************************************
IMU_data is used by all function
*******************************************************************************/
float acc_z_comp;
float acc_x,acc_y,acc_z;
static float IMU_data[9];
void IMU_getAttitude(float *RPY,float *rate_RPY)
{
		float gx,gy,gz;
    static float rateroll,ratepitch,rateyaw;
    float tong;
    float gyroXrate,gyroYrate;
    double angle_roll_acc,angle_pitch_acc;
    double kalAngleX, kalAngleY;
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
	
    acc_x=IMU_data[0];
    acc_y=IMU_data[1];
    acc_z=IMU_data[2];
    gx=IMU_unscaled[3]-offset_gx;
    gy=IMU_unscaled[4]-offset_gy;
    gz=IMU_unscaled[5]-offset_gz;

    rateroll=LPF(gx,rateroll,10,elapsedT);
    ratepitch=LPF(gy,ratepitch,10,elapsedT);
    rateyaw=LPF(gz,rateyaw,10,elapsedT);
	
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
    RPY[1]-= RPY[0]*sin(rateyaw * elapsedT * (2000/32767) * M_PI/180);
    RPY[0]+= RPY[1]*sin(rateyaw * elapsedT * (2000/32767) * M_PI/180);

    tong = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
    if(abs(acc_x) < tong) {
        angle_pitch_acc=-asin((float)acc_x/tong)*57.296f;
    }
    if(abs(acc_y) < tong) {
        angle_roll_acc=asin((float)acc_y/tong)*57.296f;
    }

    kalAngleX = Kalman_getAngle_roll(angle_roll_acc,gyroXrate, elapsedT);
    kalAngleY = Kalman_getAngle_pitch(angle_pitch_acc,gyroYrate, elapsedT);

    RPY[1] = RPY[1] * 0.975f +  kalAngleY * 0.025f;
    RPY[0] =  RPY[0] * 0.975f +   kalAngleX * 0.025f ;
//		acc_z_comp = acc_z_comp*0.9 + acc_z*0.1;
//		*rate_z += acc_z_comp*elapsedT;
}
// imu code brokking.

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
	IMU_Yaw = angles[2] = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI; // yaw
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
	lastUpdate = micros();
}

float LPF(float x, float pre_value, float CUTOFF,float dt)
{
    float RC, alpha, y;
    RC = 1.0f/(CUTOFF*2*3.1416f);
    alpha = dt/(RC+dt);
    y = pre_value + alpha * ( x - pre_value );
    return y;
}

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
