/*
*/
#include "main.h"
void get_Baro(void);


void Initial_System_Timer(void)
{
	RCC->APB1ENR |= 0x0008;	
	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIM5->CR2 = 0x0000;
	TIM5->CNT = 0x0000;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->PSC = 84 - 1;	//1us
	TIM5->EGR = 0x0001;
	TIM5->CR1 |= 0x0001; // Enable
}

// LPF
float cut_off=10;
/*Baro */
double referencePressure;
long realPressure;
float fil_comple_alt, fil_lpf_alt, EstAlt;
float absoluteAltitude, ms5611_temperature;
float ms5611_altitude, ms5611_altitude_offset;
double dt;
float rpy_simple[3], rate_rpy[3]; //roll pitch yaw 
float rpy_ahrs[3];
float rpy_kalman[3];
float rate_alt;
uint16_t j;
uint32_t loop_var;
extern uint32_t lastUpdate;
double alt_setpoint = 0.0f;
int8_t calib_error = -1;
int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    delay_init(168);
    delay_ms(200);
    board_leds_config();
	    TIM_PWM_Configuration();
    exti_gpio_config();
	  usart_printf_config(115200);
    Initial_System_Timer();  
    IMU_init();
	      begin();
    referencePressure = readPressure(0);
	// for simple_imu
//	  offset_gx=-72.0f;
//	  offset_gy = 18.0f; 
//	  offset_gz = 0.0f;
		// for Mahony
    Initialize_Q();	
	// Kalman
	  Kalman_AHRS_init();
	
	lastUpdate = TIM5->CNT;
	  do {
	  MPU6050_Calculate_Gyro_Offset(&offset_gx,&offset_gy,&offset_gz,1000);
	  dt = 0.004;
		rpy_simple[0] = rpy_simple[1] = rpy_simple[2] = rate_rpy[0] = rate_rpy[1] = rate_rpy[2] = 0.0f;
		for (j=0; j <= 300; j++){
		//	get_Baro();
			get_Baro();
			// calculate Attitude also
			IMU_getAttitude(rpy_simple, rpy_ahrs, rate_rpy, rpy_kalman);
	//		IMU_getAttitude(rpy_simple, rate_rpy);
			delay_ms(4);
			
		}
		calib_error++;
	}
	    while (fabs(rate_rpy[0]) > 0.1f || fabs(rate_rpy[1]) > 0.1f || fabs(rate_rpy[2]) > 0.11f);
		
	
//	  offset_gyro[0] = offset_gyro[1] = offset_gyro[2] = 0.0f;

//	  offset_gyro[0] = offset_gx;
//	offset_gyro[1] = offset_gy;
//	offset_gyro[2] = offset_gz;



   // delay_ms(1000);
		printf(" Quadcopter Hello World! ");
	  // wait and check if Baro values are correct, because of LPF and baro code we need some iteration for data. 
//	  dt = 0.004;
//		for (j=0; j <= 300; j++){
//			get_Baro();
//			delay_ms(4);
//		}
    ms5611_altitude_offset = EstAlt;
    // imu time counter last Update
	//	MPU6050_Calculate_Gyro_Offset(&offset_gx,&offset_gy,&offset_gz,600);
//		lastUpdate = TIM5->CNT;
        while(1) {
            GPIO_ToggleBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
            IMU_getAttitude(rpy_simple, rpy_ahrs, rate_rpy, rpy_kalman);
            get_Baro();
					  ms5611_altitude = EstAlt - ms5611_altitude_offset;

//								rpy[0] = rpy[1] = rpy[2] = 0.0f;

//							  offset_gx=-72.0f;
//							  offset_gy = 18.0f;
//							  offset_gz = 0.0f;


            while ((micros() - loop_var )< 4000) {
		        };
			 
            dt=(micros()-loop_var)*0.000001;
            loop_var = micros();
    }
}

void get_Baro(void){
	  
		update_baro(&ms5611_temperature, &realPressure, &absoluteAltitude);
    fil_lpf_alt = LPF(absoluteAltitude, fil_lpf_alt, cut_off, dt);
    fil_comple_alt = fil_comple_alt*0.977f + fil_lpf_alt*0.023f;
	  EstAlt = fil_comple_alt*10;
}
