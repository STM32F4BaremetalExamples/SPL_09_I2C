#include "stm32f4xx.h"                  // Device header
#include "serial_stdio.h"
#include "retarget_stm32f4.h"
#include <string.h>
/*Led PB13, Button PC13*/

void delay_ms(int delay_time);
void led_init(void);
void button_init(void);

void i2c_master_init(void);
void I2C_transfer(unsigned char address, unsigned char * pData,unsigned char nData);
#define DAC_ADDRESS7	0x62
void DAC_output(unsigned short step);

Serial_t USART2_Serial={USART2_getChar,USART2_sendChar};

char mybf[80];/*Input buffer*/
char wordBuffer[80];

int main(){
	unsigned short dac_val;
	float volt=1.6;
	led_init();
	USART2_init(9600);
	i2c_master_init();
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

//#define I2C_SLAVE
  
/* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
   input clock) must be a multiple of 10 MHz */
/* Uncomment the line below if you will use the I2C peripheral in Fast Mode */
//#define FAST_I2C_MODE
  
/* Uncomment the line below if you will use the I2C peripheral in 10-bit addressing
   mode */
//#define I2C_10BITS_ADDRESS
  
/* Define I2C Speed ----------------------------------------------------------*/
#ifdef FAST_I2C_MODE
 #define I2C_SPEED 340000
 #define I2C_DUTYCYCLE I2C_DutyCycle_16_9  

#else /* STANDARD_I2C_MODE*/
 #define I2C_SPEED 100000
 #define I2C_DUTYCYCLE  I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/
  
/* Define Slave Address  -----------------------------------------------------*/
#ifdef I2C_10BITS_ADDRESS
 #define SLAVE_ADDRESS (uint16_t)0x0330

#else /* I2C_7BITS_ADDRESS */
 #define SLAVE_ADDRESS 0x30
#endif /* I2C_10BITS_ADDRESS */
 
/* USER_TIMEOUT value for waiting loops. This timeout is just a guarantee that the
   application will not remain stuck if the I2C communication is corrupted. 
   You may modify this timeout value depending on CPU frequency and application
   conditions (interrupts routines, number of data to transfer, speed, CPU
   frequency...). */ 
#define USER_TIMEOUT                  ((uint32_t)0x64) /* Waiting 1s */  
  
/* I2Cx Communication boards Interface */
#define I2Cx_DMA                      DMA1
#define I2Cx_DMA_CHANNEL              DMA_Channel_1
#define I2Cx_DR_ADDRESS               ((uint32_t)0x40005410)
#define I2Cx_DMA_STREAM_TX            DMA1_Stream6
#define I2Cx_DMA_STREAM_RX            DMA1_Stream0
#define I2Cx_TX_DMA_TCFLAG            DMA_FLAG_TCIF6
#define I2Cx_TX_DMA_FEIFLAG           DMA_FLAG_FEIF6
#define I2Cx_TX_DMA_DMEIFLAG          DMA_FLAG_DMEIF6
#define I2Cx_TX_DMA_TEIFLAG           DMA_FLAG_TEIF6
#define I2Cx_TX_DMA_HTIFLAG           DMA_FLAG_HTIF6
#define I2Cx_RX_DMA_TCFLAG            DMA_FLAG_TCIF0
#define I2Cx_RX_DMA_FEIFLAG           DMA_FLAG_FEIF0
#define I2Cx_RX_DMA_DMEIFLAG          DMA_FLAG_DMEIF0
#define I2Cx_RX_DMA_TEIFLAG           DMA_FLAG_TEIF0
#define I2Cx_RX_DMA_HTIFLAG           DMA_FLAG_HTIF0
#define DMAx_CLK                      RCC_AHB1Periph_DMA1    

#define I2Cx                          I2C1
#define I2Cx_CLK                      RCC_APB1Periph_I2C1
#define I2Cx_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SDA_PIN                  GPIO_Pin_9                
#define I2Cx_SDA_GPIO_PORT            GPIOB                       
#define I2Cx_SDA_SOURCE               GPIO_PinSource9
#define I2Cx_SDA_AF                   GPIO_AF_I2C1
  
#define I2Cx_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SCL_PIN                  GPIO_Pin_8                
#define I2Cx_SCL_GPIO_PORT            GPIOB                    
#define I2Cx_SCL_SOURCE               GPIO_PinSource8
#define I2Cx_SCL_AF                   GPIO_AF_I2C1

void i2c_master_init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
    
	/* RCC Configuration */
	/*I2C Peripheral clock enable */
	RCC_APB1PeriphClockCmd(I2Cx_CLK, ENABLE);
	/*SDA GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2Cx_SDA_GPIO_CLK, ENABLE);
	/*SCL GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2Cx_SCL_GPIO_CLK, ENABLE);
	
	/* GPIO Configuration */
	/*Configure I2C SCL pin */
	GPIO_InitStructure.GPIO_Pin = I2Cx_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);

	/*Configure I2C SDA pin */
	GPIO_InitStructure.GPIO_Pin = I2Cx_SDA_PIN;
	GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);
	/* Connect PXx to I2C_SCL */
	GPIO_PinAFConfig(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_SOURCE, I2Cx_SCL_AF);
	/* Connect PXx to I2C_SDA */
	GPIO_PinAFConfig(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_SOURCE, I2Cx_SDA_AF);

	/* Configure I2C Filters */
	I2C_AnalogFilterCmd(I2Cx,ENABLE);
	I2C_DigitalFilterConfig(I2Cx,0x0F);
	
	I2C_InitTypeDef  I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
	I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx, &I2C_InitStructure);

	/* I2C ENABLE */
	I2C_Cmd(I2Cx, ENABLE);
}


void I2C_transfer(unsigned char address_7b, unsigned char * pData,unsigned char nData){
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB));//wait for start bit to be transmitted
	I2C_Send7bitAddress(I2C1,address_7b<<1,I2C_Direction_Transmitter);//send address, set write to device mode
	while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_ADDR));//wait for address to be transmitted
	if(I2C1->SR2){}
	for(int i=0; i<nData;i++){
		I2C_SendData(I2C1,pData[i]);
		while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE));//wait until tx buffer empty
	}
	while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_BTF));//wait until byte transfer finished
	I2C_GenerateSTOP(I2C1,ENABLE);
}
