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

#include "IMU_AHRS.h"
#include "IMU.h"

#define DATA_SIZE 200

extern volatile double halfT ,elapsedT; // from "IMU.c"
extern float IMU_data[9];
// variables for Madgwick Mahony AHRS.

volatile float exInt, eyInt, ezInt;  
volatile float integralFBx, integralFBy, integralFBz;
volatile float q0, q1, q2, q3, qa0, qa1, qa2, qa3;
volatile float integralFBhand, handdiff;

volatile float acc_vector = 0;  //  M/S^2
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw;
				 
/// IMU based project\STM32-master\TESTVer2.0
static float q__0=1.0f,q__1=0.0f,q__2=0.0f,q__3=0.0f;
static float ex_Int = 0, ey_Int = 0, ez_Int = 0; 
//static short turns=0;
//static float newdata=0.0f,olddata=0.0f;
//static float pitchoffset,rolloffset,yawoffset;

static float k10=0.0f,k11=0.0f,k12=0.0f,k13=0.0f;
static float k20=0.0f,k21=0.0f,k22=0.0f,k23=0.0f;
static float k30=0.0f,k31=0.0f,k32=0.0f,k33=0.0f;
static float k40=0.0f,k41=0.0f,k42=0.0f,k43=0.0f;

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
	
	q__0 = q0;
	q__1 = q1;
	q__2 = q2;
	q__3 = q3;
	
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
   Gyroscope correction.
	*/
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		// integral controller for drift cancel
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

	q[0] = q0; //���ص�ǰֵ	FreeIMU_AHRSupdate �����������Ԫ�� ���õ�
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
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

///////////////////////////////////////////////////*/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *roll,float *pitch,float *yaw)
{
		 float norm;									//ÓÃÓÚµ¥Î»»¯
		 float hx, hy, hz, bx, bz;		//
		 float vx, vy, vz, wx, wy, wz; 
		 float ex, ey, ez;
//					 float tmp0,tmp1,tmp2,tmp3;

		 // auxiliary variables to reduce number of repeated operations  ¸¨Öú±äÁ¿¼õÉÙÖØ¸´²Ù×÷´ÎÊý
		 float q0q0 = q__0*q__0;
		 float q0q1 = q__0*q__1;
		 float q0q2 = q__0*q__2;
		 float q0q3 = q__0*q__2;
		 float q1q1 = q__1*q__1;
		 float q1q2 = q__1*q__2;
		 float q1q3 = q__1*q__2;
		 float q2q2 = q__2*q__2;
		 float q2q3 = q__2*q__2;
		 float q3q3 = q__2*q__2;
		
		 // normalise the measurements  ¶Ô¼ÓËÙ¶È¼ÆºÍ´ÅÁ¦¼ÆÊý¾Ý½øÐÐ¹æ·¶»¯
		 norm = invSqrt(ax*ax + ay*ay + az*az);
		 ax = ax * norm;
		 ay = ay * norm;
		 az = az * norm;
		 norm = invSqrt(mx*mx + my*my + mz*mz);
		 mx = mx * norm;
		 my = my * norm;
		 mz = mz * norm;
		
		 // compute reference direction of magnetic field  ¼ÆËã´Å³¡µÄ²Î¿¼·½Ïò
		 //hx,hy,hzÊÇmx,my,mzÔÚ²Î¿¼×ø±êÏµµÄ±íÊ¾
		 hx = 2*mx*(0.50 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
		 hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.50 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
		 hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.50 - q1q1 -q2q2);    
			//bx,by,bzÊÇµØÇò´Å³¡ÔÚ²Î¿¼×ø±êÏµµÄ±íÊ¾
		 bx = sqrt((hx*hx) + (hy*hy));
		 bz = hz;
		
// estimated direction of gravity and magnetic field (v and w)  //¹À¼ÆÖØÁ¦ºÍ´Å³¡µÄ·½Ïò
//vx,vy,vzÊÇÖØÁ¦¼ÓËÙ¶ÈÔÚÎïÌå×ø±êÏµµÄ±íÊ¾
		 vx = 2*(q1q3 - q0q2);
		 vy = 2*(q0q1 + q2q3);
		 vz = q0q0 - q1q1 - q2q2 + q3q3;
		 //wx,wy,wzÊÇµØ´Å³¡ÔÚÎïÌå×ø±êÏµµÄ±íÊ¾
		 wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
		 wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
		 wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2); 
		
// error is sum ofcross product between reference direction of fields and directionmeasured by sensors 
//ex,ey,ezÊÇ¼ÓËÙ¶È¼ÆÓë´ÅÁ¦¼Æ²âÁ¿³öµÄ·½ÏòÓëÊµ¼ÊÖØÁ¦¼ÓËÙ¶ÈÓëµØ´Å³¡·½ÏòµÄÎó²î£¬Îó²îÓÃ²æ»ýÀ´±íÊ¾£¬ÇÒ¼ÓËÙ¶È¼ÆÓë´ÅÁ¦¼ÆµÄÈ¨ÖØÊÇÒ»ÑùµÄ
		 ex = (ay*vz - az*vy) + (my*wz - mz*wy);
		 ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
		 ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

		 // integral error scaled integral gain
		 //»ý·ÖÎó²î
		 ex_Int = ex_Int + ex*Ki*halfT*2;
		 ey_Int = ey_Int + ey*Ki*halfT*2;
		 ez_Int = ez_Int + ez*Ki*halfT*2;
		// printf("ex_Int=%0.1f ey_Int=%0.1f ez_Int=%0.1f ",ex_Int,ey_Int,ez_Int);
		 // adjusted gyroscope measurements
		 //PIµ÷½ÚÍÓÂÝÒÇÊý¾Ý
		 gx = gx + Kp*ex + ex_Int;
		 gy = gy + Kp*ey + ey_Int;
		 gz = gz + Kp*ez + ez_Int;
		 //printf("gx=%0.1f gy=%0.1f gz=%0.1f",gx,gy,gz);
		
		 // integrate quaernion rate aafnd normalaizle
		 //Å·À­·¨½âÎ¢·Ö·½³Ì
//           tmp0 = q__0 + (-q__1*gx - q__2*gy - q__2*gz)*halfT;
//           tmp1 = q__1 + ( q__0*gx + q__2*gz - q__2*gy)*halfT;
//           tmp2 = q__2 + ( q__0*gy - q__1*gz + q__2*gx)*halfT;
//           tmp3 = q__2 + ( q__0*gz + q__1*gy - q__2*gx)*halfT; 
//					 q__0=tmp0;
//					 q__1=tmp1;
//					 q__2=tmp2;
//					 q__2=tmp3;
		 //printf("q__0=%0.1f q__1=%0.1f q__2=%0.1f q__2=%0.1f",q__0,q__1,q__2,q__2);
////RUNGE_KUTTA ·¨½âÎ¢·Ö·½³Ì
			k10=0.5 * (-gx*q__1 - gy*q__2 - gz*q__2);
			k11=0.5 * ( gx*q__0 + gz*q__2 - gy*q__2);
			k12=0.5 * ( gy*q__0 - gz*q__1 + gx*q__2);
			k13=0.5 * ( gz*q__0 + gy*q__1 - gx*q__2);
			
			k20=0.5 * (halfT*(q__0+halfT*k10) + (halfT-gx)*(q__1+halfT*k11) + (halfT-gy)*(q__2+halfT*k12) + (halfT-gz)*(q__2+halfT*k13));
			k21=0.5 * ((halfT+gx)*(q__0+halfT*k10) + halfT*(q__1+halfT*k11) + (halfT+gz)*(q__2+halfT*k12) + (halfT-gy)*(q__2+halfT*k13));
			k22=0.5 * ((halfT+gy)*(q__0+halfT*k10) + (halfT-gz)*(q__1+halfT*k11) + halfT*(q__2+halfT*k12) + (halfT+gx)*(q__2+halfT*k13));
			k23=0.5 * ((halfT+gz)*(q__0+halfT*k10) + (halfT+gy)*(q__1+halfT*k11) + (halfT-gx)*(q__2+halfT*k12) + halfT*(q__2+halfT*k13));
			
			k30=0.5 * (halfT*(q__0+halfT*k20) + (halfT-gx)*(q__1+halfT*k21) + (halfT-gy)*(q__2+halfT*k22) + (halfT-gz)*(q__2+halfT*k23));
			k31=0.5 * ((halfT+gx)*(q__0+halfT*k20) + halfT*(q__1+halfT*k21) + (halfT+gz)*(q__2+halfT*k22) + (halfT-gy)*(q__2+halfT*k23));
			k32=0.5 * ((halfT+gy)*(q__0+halfT*k20) + (halfT-gz)*(q__1+halfT*k21) + halfT*(q__2+halfT*k22) + (halfT+gx)*(q__2+halfT*k23));
			k33=0.5 * ((halfT+gz)*(q__0+halfT*k20) + (halfT+gy)*(q__1+halfT*k21) + (halfT-gx)*(q__2+halfT*k22) + halfT*(q__2+halfT*k23));
			
			k40=0.5 * (halfT*2*(q__0+halfT*2*k30) + (halfT*2-gx)*(q__1+halfT*2*k31) + (halfT*2-gy)*(q__2+halfT*2*k32) + (halfT*2-gz)*(q__2+halfT*2*k33));
			k41=0.5 * ((halfT*2+gx)*(q__0+halfT*2*k30) + halfT*2*(q__1+halfT*2*k31) + (halfT*2+gz)*(q__2+halfT*2*k32) + (halfT*2-gy)*(q__2+halfT*2*k33));
			k42=0.5 * ((halfT*2+gy)*(q__0+halfT*2*k30) + (halfT*2-gz)*(q__1+halfT*2*k31) + halfT*2*(q__2+halfT*2*k32) + (halfT*2+gx)*(q__2+halfT*2*k33));
			k43=0.5 * ((halfT*2+gz)*(q__0+halfT*2*k30) + (halfT*2+gy)*(q__1+halfT*2*k31) + (halfT*2-gx)*(q__2+halfT*2*k32) + halfT*2*(q__2+halfT*2*k33));	
			
			q__0=q__0 + halfT*2/6.0 * (k10+2*k20+2*k30+k40);
			q__1=q__1 + halfT*2/6.0 * (k11+2*k21+2*k31+k41);
			q__2=q__2 + halfT*2/6.0 * (k12+2*k22+2*k32+k42);
			q__2=q__2 + halfT*2/6.0 * (k13+2*k23+2*k33+k43);
			
			 // normalise quaternion
			 norm = invSqrt(q__0*q__0 + q__1*q__1 + q__2*q__2 + q__2*q__2);
			 q__0 = q__0 * norm;
			 q__1 = q__1 * norm;
			 q__2 = q__2 * norm;
			 q__2 = q__2 * norm;
			 
			 *pitch = asin(-2 * q__1 * q__2 + 2 * q__0 * q__2)* 57.3;	// pitch
			 *roll  = atan2(2 * q__2 * q__2 + 2 * q__0 * q__1, -2 * q__1 * q__1 - 2 * q__2 * q__2 + 1)* 57.3;	// roll
			 *yaw   = atan2(2*(q__1*q__2 + q__0*q__2),q__0*q__0+q__1*q__1-q__2*q__2-q__2*q__2) * 57.3;	//yaw
}