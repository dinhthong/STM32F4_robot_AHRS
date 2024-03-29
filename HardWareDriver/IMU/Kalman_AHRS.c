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

#include "Kalman_AHRS.h"
#include "IMU.h"
// variables for quaternion based Kalman Filter
volatile float kq_0, kq_1, kq_2, kq_3,w1,w2,w3; // È«¾ÖËÄÔªÊý

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

    kq_0=1.0;
    kq_1=0;
    kq_2=0;
    kq_3=0;
}

volatile float q_kalman_rpy[3];
void KalmanAHRS_getRollPitchYaw(float *angles, float IMU_data[])
{
	float q[4]; // ¡¡ËÄÔªÊý
	KalmanAHRS_getQ(q, IMU_data); // ¸üÐÂÈ«¾ÖËÄÔªÊý
#if 1
  angles[2] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw 
  angles[1] = -safe_asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	 // if(angles[2]<2)angles[2]+=360.0f;  //½« -+180¶È  ×ª³É0-360¶È
#else
	q_kalman_rpy[0] = angles[0] = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
										 1 - 2.0f * (q[1] * q[1] + q[2] * q[2]))) * 180 / M_PI;
	// we let safe_asin() handle the singularities near 90/-90 in pitch

	q_kalman_rpy[1] = angles[1] = -safe_asin(2.0f * (q[0] * q[2] - q[3] * q[1])) * 180 / M_PI;
	q_kalman_rpy[2] = angles[2] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) * 180 / M_PI;
#endif
}

void KalmanAHRS_getQ(float * q,volatile float IMU_values[9]) {
   Kalman_AHRSupdate(IMU_values[3] * M_PI/180, IMU_values[4] * M_PI/180, IMU_values[5] * M_PI/180,
   IMU_values[0], IMU_values[1], IMU_values[2], IMU_values[6], IMU_values[7], IMU_values[8]);
     
   q[0] = kq_0; //·µ»Øµ±Ç°Öµ
   q[1] = kq_1;
   q[2] = kq_2;
   q[3] = kq_3;
}

// Kalman
extern volatile double halfT ,elapsedT; // from "IMU.c"

void Kalman_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  float bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float g=9.79973;
  float Ha1,Ha2,Ha3,Ha4,Hb1,Hb2,Hb3,Hb4;
  float e1,e2,e3,e4,e5,e6;
//  float halfT;

// ÏÈ°ÑÕâÐ©ÓÃµÃµ½µÄÖµËãºÃ
  float q0q0 = kq_0*kq_0;
  float q0q1 = kq_0*kq_1;
  float q0q2 = kq_0*kq_2;
  float q0q3 = kq_0*kq_3;
  float q1q1 = kq_1*kq_1;
  float q1q2 = kq_1*kq_2;
  float q1q3 = kq_1*kq_3;
  float q2q2 = kq_2*kq_2;   
  float q2q3 = kq_2*kq_3;
  float q3q3 = kq_3*kq_3;      
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
	
    Ha1=(-kq_2)*g; Ha2=kq_3*g; Ha3=-kq_0*g; Ha4=kq_1*g;	 
    Hb1=bx*kq_0-bz*kq_2;
    Hb2=bx*kq_1+bz*kq_3;//
    Hb3=-bx*kq_2-bz*kq_0;
    Hb4=-bx*kq_3+bz*kq_1;
  
    H[0]= Ha1;H[1]= Ha2;H[2]= Ha3;H[3]= Ha4;
    H[7]= Ha4;H[8]=-Ha3;H[9]= Ha2;H[10]=-Ha1;
    H[14]=-Ha3;H[15]=-Ha4;H[16]= Ha1;H[17]= Ha2;
  
    H[21]= Hb1;H[22]= Hb2;H[23]= Hb3;H[24]= Hb4;      
    H[28]= Hb4;H[29]=-Hb3;H[30]= Hb2;H[31]=-Hb1;
    H[35]=-Hb3;H[36]=-Hb4;H[37]= Hb1;H[38]= Hb2;
    //×´Ì¬¸üÐÂ
    kq_0 = kq_0 + (-kq_1*gx - kq_2*gy - kq_3*gz)*halfT;
    kq_1 = kq_1 + (kq_0*gx + kq_2*gz - kq_3*gy)*halfT;
    kq_2 = kq_2 + (kq_0*gy - kq_1*gz + kq_3*gx)*halfT;
    kq_3 = kq_3 + (kq_0*gz + kq_1*gy - kq_2*gx)*halfT;  
      // ËÄÔªÊý¹éÒ»
    norm = invSqrt(kq_0*kq_0 + kq_1*kq_1 + kq_2*kq_2 + kq_3*kq_3);
    kq_0 = kq_0 * norm;
    kq_1 = kq_1 * norm;
    kq_2 = kq_2 * norm;
    kq_3 = kq_3 * norm;
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
    kq_0= kq_0+Y[0];
    kq_1= kq_1+Y[1];
    kq_2= kq_2+Y[2];
    kq_3= kq_3+Y[3];
    w1= w1+Y[4];
    w2= w2+Y[5];
    w3= w3+Y[6];
    MatrixMultiply(K,7,6,H,6,7,Z); //Z= K*H		7*7
    MatrixSub(I,Z,O,7,7 );	  //O=I-K*H
 
    MatrixMultiply(O,7,7,P1,7,7,P);
 
    // normalise quaternion
    norm = invSqrt(kq_0*kq_0 + kq_1*kq_1 + kq_2*kq_2 + kq_3*kq_3);
    kq_0 = kq_0 * norm;
    kq_1 = kq_1 * norm;
    kq_2 = kq_2 * norm;
    kq_3 = kq_3 * norm;
}
