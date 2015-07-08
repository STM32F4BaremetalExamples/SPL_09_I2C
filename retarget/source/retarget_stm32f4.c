#include "retarget_stm32f4.h"

void USART2_init(int baudrate){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/*Enable peripheral clock for GPIOA*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	/*Enable peripheral clock for UART2*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	/*GPIOA Configuration PA2 as TX PA3 as RX*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	/*Connect USART2 pins to AF2*/
	//TX=PA2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate=baudrate;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_Init(USART2,&USART_InitStructure);
	USART_Cmd(USART2,ENABLE);
}


void USART2_sendChar(char ch){
	while(!USART_GetFlagStatus(USART2,USART_FLAG_TXE));
	
	USART_SendData(USART2,ch);
}

char USART2_getChar(void){
	if(USART_GetFlagStatus(USART2,USART_FLAG_ORE)) USART_ClearFlag(USART2,USART_FLAG_ORE); 
	
	while(!USART_GetFlagStatus(USART2,USART_FLAG_RXNE));
	return USART_ReceiveData(USART2);
}

void USART1_init(int baudrate){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/*Enable peripheral clock for GPIOA*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	/*Enable peripheral clock for UART1*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	/*GPIOA Configuration PA9 as TX PA10 as RX*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	/*Connect USART2 pins to AF7*/
	//PA9=USART1_TX
	//PA10=USART1_RX
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate=baudrate;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1,ENABLE);
}


void USART1_sendChar(char ch){
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
	
	USART_SendData(USART1,ch);
}

char USART1_getChar(void){
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)) USART_ClearFlag(USART1,USART_FLAG_ORE); 
	
	while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));
	return USART_ReceiveData(USART1);
}

