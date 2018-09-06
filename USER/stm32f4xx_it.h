#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H
void Initial_System_Timer(void);
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
	
	// my funcion
void RC_update(void);
extern uint16_t channels[10];
extern float alt_sonar;
void rcv_USART_handler(void);
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
