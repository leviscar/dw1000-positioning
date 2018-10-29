#ifndef __I2C_H
#define __I2C_H
//IIC���в�������
#include "stm32f0xx_hal.h"
typedef uint8_t  u8;
#define PIN_CLK MPU_SCL_Pin
#define PIN_SDA MPU_SDA_Pin
#define IIC_GPIOx MPU_SCL_GPIO_Port

#define IIC_CLK_H() IIC_GPIOx->BSRR = PIN_CLK
#define IIC_CLK_L() IIC_GPIOx->BRR = PIN_CLK
#define IIC_SDA_H() IIC_GPIOx->BSRR = PIN_SDA
#define IIC_SDA_L() IIC_GPIOx->BRR = PIN_SDA
#define SDA_READ() (IIC_GPIOx->IDR & PIN_SDA)

void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

int I2C_ReadRegister_9250(unsigned char Address, unsigned char RegisterAddr, 
                                          unsigned short RegisterLen, unsigned char *RegisterValue);
int I2C_WriteRegister_9250(unsigned char Address, unsigned char RegisterAddr, 
                                           unsigned short RegisterLen, const unsigned char *RegisterValue);  


//----------------------------------------------

#endif
