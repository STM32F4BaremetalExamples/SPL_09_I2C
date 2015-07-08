#ifndef _RETARGET_STM32F4_H
#define _RETARGET_STM32F4_H
#include "stm32f4xx.h"                  // Device header

void USART1_init(int baudrate);

void USART1_sendChar(char ch);

char USART1_getChar(void);


void USART2_init(int baudrate);

void USART2_sendChar(char ch);

char USART2_getChar(void);

#endif

