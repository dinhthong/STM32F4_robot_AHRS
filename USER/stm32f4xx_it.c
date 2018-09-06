#include "stm32f4xx_it.h"


void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}
/*
Input Capture Timer2 for RC channels readings.
In this case number of channels is predetermined by me.
7 channels, PPM.
More works will be made for universal cases.
*/
//__IO uint32_t IC2Value = 0;
//uint16_t RCvalue;
uint16_t channels_buffer[10],channels[10];
uint8_t channel_read_fail;
uint16_t PPMval;
uint8_t channels_cnt;
uint32_t lastRC_us;
void EXTI1_IRQHandler(void)
{
/* Make sure that interrupt flag is set */
	  if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
	  	  PPMval = TIM5->CNT - lastRC_us;
	  	  lastRC_us = TIM5->CNT;
	  		if (PPMval > 10000) {
						if (channels_cnt == 8 && channel_read_fail == 0) {
						    RC_update();
					  }
					  channels_cnt=0;
					  channel_read_fail=0;
				}
			  else {
					  if (PPMval > 1050 && PPMval < 1950) {
						    channels_buffer[channels_cnt++] = PPMval;
					  }
					  else {
						channel_read_fail = 1;
					  }
				}
    /* Clear interrupt flag */
		EXTI->PR = EXTI_Line1;
		}

}
void RC_update(void) {
		for (uint8_t i=0; i<8; i++) {
				channels[i] = channels_buffer[i];
	  } 
}
#define BUFFERSIZE 18
char RxBuffer[BUFFERSIZE];
__IO uint8_t RxIndex = 0x00;
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {

    }

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        //read from USART_DR will automatically clear the RNXE bit.
        RxBuffer[RxIndex++] = USART_ReceiveData(USART2);
        if (RxIndex >= BUFFERSIZE)
        {
            RxIndex=0;
            /* Receive Transaction data */
            rcv_USART_handler();
        }
    }
}

void send_USART_PC(void)
{
//printf("*%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,\r\n",
//	(float)tick_count*0.000001,(float)esc_1,(float)esc_2,(float)esc_3,(float)esc_4,angle_roll,
//		angle_pitch,angle_yaw,gyro_roll_input,gyro_pitch_input,gyro_yaw_input);
	
//	printf("*%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,\r\n",
//	(float)tick_count*0.000001,(float)CH_AILE,(float)CH_ELEV,(float)CH_GEAR,(float)CH_DR,
//		(float)CH_THRO,(float)CH_RUDD,angle_yaw,gyro_roll_input,gyro_pitch_input,gyro_yaw_input);
}

void rcv_USART_handler(void)
{
//    Kp_roll=(RxBuffer[0]-48)*100+(RxBuffer[1]-48)*10+(RxBuffer[2]-48)+(RxBuffer[4]-48)/10.0+(RxBuffer[5]-48)/100.0;
//    Ki_roll=(RxBuffer[6]-48)*100+(RxBuffer[7]-48)*10+(RxBuffer[8]-48)+(RxBuffer[10]-48)/10.0+(RxBuffer[11]-48)/100.0;
//    Kd_roll=(RxBuffer[12]-48)*100+(RxBuffer[13]-48)*10+(RxBuffer[14]-48)+(RxBuffer[16]-48)/10.0+(RxBuffer[17]-48)/100.0;
//	  Kp_pitch=Kp_roll;
//   	Ki_pitch=Ki_roll;
//   	Kd_pitch=Kd_roll;
}
uint32_t a5,c5;
float temp_sonar, alt_sonar, pre_alt_sonar;
void EXTI9_5_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)) {
            c5 = TIM5->CNT-a5; // determine the frequency but don't use
            a5 = TIM5->CNT;
        }
        else {
            temp_sonar=TIM5->CNT-a5;
            temp_sonar =temp_sonar/58;
					  // complementary filter
            temp_sonar = alt_sonar*0.8+temp_sonar*0.2;
					  //check if irregular data. and process it.
					  alt_sonar=temp_sonar;
            pre_alt_sonar=alt_sonar;
        }
    EXTI->PR = EXTI_Line5;
    }
}	