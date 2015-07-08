#include "stm32f4xx.h"                  // Device header
#include "serial_stdio.h"
#include "retarget_stm32f4.h"
#include <string.h>
/*Led PB13, Button PC13*/

void delay_ms(int delay_time);
void led_init(void);
void button_init(void);

void I2C_init(void);
void I2C_transfer(unsigned char address, unsigned char * pData,unsigned char nData);
#define DAC_ADDRESS7	0x62
void DAC_output(unsigned short step);

Serial_t USART2_Serial={USART2_getChar,USART2_sendChar};

char mybf[80];/*Input buffer*/
char wordBuffer[80];

int main(){
	unsigned short dac_val;
	float volt=1.8;
	led_init();
	USART2_init(9600);
	I2C_init();
	serial_puts(USART2_Serial,"\nSystem ready\n");
	while(1){
		delay_ms(0xFFFF);
		dac_val=(unsigned short)(volt*(4095.0/3.3));
		DAC_output(dac_val);
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

void DAC_output(unsigned short step){
	unsigned char tx_data[]={0x40,0x0,0x0};
	tx_data[1]=((step>>4)&(0xFF));
	tx_data[2]=((step<<4)&(0xF0));
	I2C_transfer(DAC_ADDRESS7,tx_data,3);
}

void I2C_init(void){
		/*
	      Para I2C los pines a utilizar es pin B8 y B9 
				B8  I2C1_SCL
				B9  I2C1_SDA		 funcion alternativa F 04
	*/
	
	RCC->AHB1ENR|=(0x1<<1);//activando el puerto B
	GPIOB->MODER&=~((GPIO_MODER_MODER8)//Limpando el registro del pin 8
								|(GPIO_MODER_MODER9));//Limpando el registro del pin 8
	GPIOB->MODER|=((0x2<<16)| //funcion alternativa del registro 
								(0x2<<18));//funcion alternativa del registro 
	GPIOB->OTYPER|=((0x1<<8)//registro de Output open-drain 152 
								|(0x1<<9));//registro Output open-drain
	GPIOB->AFR[1]&=~((0xF<<4)// limpiando el  amfel 8
									|(0xF<<0));//limpiando el  amfel 9
	GPIOB->AFR[1]|=((0x4<<4)// afsel 8
									|(0x4<<0));//afsel  9	
	//terminacion de configuracion del los pines 
	//configuracion del i2c master 100khz 
	RCC->APB1ENR|=RCC_APB1ENR_I2C1EN;
	
	I2C1->CR1=0;
	I2C1->CR2=0;
	I2C1->CR2|=(16<<0);//[5:0] FREQ = 16 : 16MHz in APB1
	I2C1->CCR=0;
	I2C1->CCR|=(80);
	I2C1->TRISE=(16+1);
	I2C1->FLTR=0;
	I2C1->CR1|=I2C_CR1_PE;
}

void I2C_transfer(unsigned char address_7, unsigned char * pData,unsigned char nData){
	I2C1->CR1|=I2C_CR1_START;
	while(!((I2C1->SR1)&(I2C_SR1_SB)));//esperamos al envio del start
	I2C1->DR=(address_7<<1);// conversion a direccion de escitura de 8 bits
	while(!((I2C1->SR1)&(I2C_SR1_ADDR)));//esperamos al envio de direccion 
	if(I2C1->SR2){
	
	}	
	for(int i=0; i<nData;i++){
		I2C1->DR=pData[i];
		while(!((I2C1->SR1)&(I2C_SR1_TXE)));//esperamos el envio de dato
	}
	while(!((I2C1->SR1)&(I2C_SR1_BTF)));//esperamos el envio de dato
	I2C1->CR1|=I2C_CR1_STOP;
}
