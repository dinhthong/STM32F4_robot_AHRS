#include "stm32f4xx.h"

//void USART_Configuration(unsigned int BaudRate);
void board_leds_config(void);
void TIMbase_Configuration(void);

void TIM_PWM_Configuration(void);

void TIM_IC_Config(void);
void exti_gpio_config(void);
void usart_printf_config(uint32_t BaudRate);

