#include "stm32f4xx.h"                  // Device header
#include "serial_stdio.h"
#include "retarget_stm32f4.h"
#include <string.h>
/*Led PB13, Button PC13*/

void delay_ms(int delay_time);
void led_init(void);
void button_init(void);

Serial_t USART2_Serial={USART2_getChar,USART2_sendChar};

char mybf[80];/*Input buffer*/
char wordBuffer[80];

int main(){
	int lineCounter=1;
	led_init();
	USART2_init(9600);
	serial_puts(USART2_Serial,"\nSystem ready\n");
	while(1){
		serial_printf(USART2_Serial,"%d$ ",lineCounter);
		serial_gets(USART2_Serial,mybf,80);
		serial_printf(USART2_Serial,"%s\n",mybf);
		if(sscanf(mybf,"%s",wordBuffer) > 0){
			serial_printf(USART2_Serial,"word: %s\n",wordBuffer);
			serial_printf(USART2_Serial,"characters: %d\n",strlen(wordBuffer));
		}
		lineCounter++;
	}
	return 0;
}

void delay_ms(int delay_time){
	for(int i=0; i<delay_time; i++);
}

void led_init(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef myGPIO;
	GPIO_StructInit(&myGPIO);
	myGPIO.GPIO_Mode=GPIO_Mode_OUT;
	myGPIO.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOA,&myGPIO);
}

void button_init(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef myGPIO;
	GPIO_StructInit(&myGPIO);
	myGPIO.GPIO_Mode=GPIO_Mode_IN;
	myGPIO.GPIO_Pin=GPIO_Pin_13;
	GPIO_Init(GPIOC,&myGPIO);
}
