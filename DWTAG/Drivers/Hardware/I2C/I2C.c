#include "I2C.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 

//////////////////////////////////////////////////////////////////////////////////
 
//��ʼ��IIC
//void IIC_Init(void)
//{					     
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE );	
//	   
//	GPIO_InitStructure.GPIO_Pin = PIN_CLK|PIN_SDA;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //���
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(IIC_GPIOx, &GPIO_InitStructure);
//	GPIO_SetBits(IIC_GPIOx,PIN_CLK|PIN_SDA); 	//PB13,PB14 �����
//}
//����IIC��ʼ�ź�
/*
static void I2C_delay(void)
{
    volatile int i = 14;	  //ԭΪi=7��ʹ��GD32оƬ���ӳ�������1����������ʱ��
    while (i)
        i--;
}

int I2C_Start(void)
{
    IIC_SDA_H();
    IIC_CLK_H();
    I2C_delay();
    if (!SDA_READ())
        return 0;
    IIC_SDA_L();
    I2C_delay();
    if (SDA_READ())
        return 0;
    IIC_SDA_L();
    I2C_delay();
    return 1;
}

void I2C_Stop(void)
{
    IIC_CLK_L();
    I2C_delay();
    IIC_SDA_L();
    I2C_delay();
    IIC_CLK_H();
    I2C_delay();
    IIC_SDA_H();
    I2C_delay();
}

 void I2C_Ack(void)
{
    IIC_CLK_L();
    I2C_delay();
    IIC_SDA_L();
    I2C_delay();
    IIC_CLK_H();
    I2C_delay();
    IIC_CLK_L();
    I2C_delay();
}

 void I2C_NoAck(void)
{
    IIC_CLK_L();
    I2C_delay();
    IIC_SDA_H();
    I2C_delay();
    IIC_CLK_H();
    I2C_delay();
    IIC_CLK_L();
    I2C_delay();
}

 int I2C_WaitAck(void)
{
    IIC_CLK_L();
    I2C_delay();
    IIC_SDA_H();
    I2C_delay();
    IIC_CLK_H();
    I2C_delay();
    if (SDA_READ()) {
        IIC_CLK_L();
        return 0;
    }
    IIC_CLK_L();
    return 1;
}

 void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        IIC_CLK_L();
        I2C_delay();
        if (byte & 0x80)
            IIC_SDA_H();
        else
            IIC_SDA_L();
        byte <<= 1;
        I2C_delay();
        IIC_CLK_H();
        I2C_delay();
    }
    IIC_CLK_L();
}

 uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    IIC_SDA_H();
    while (i--) {
        byte <<= 1;
        IIC_CLK_L();
        I2C_delay();
        IIC_CLK_H();
        I2C_delay();
        if (SDA_READ()) {
            byte |= 0x01;
        }
    }
    IIC_CLK_L();
    return byte;
}


int I2C_WriteRegister_9250(uint8_t addr, uint8_t reg, uint16_t len, const uint8_t * data)
{
    int i;
    if (!I2C_Start())
        return 1;
    I2C_SendByte(addr << 1 | 0);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return 1;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) {
        I2C_SendByte(data[i]);
        if (I2C_WaitAck()) {
            I2C_Stop();
            return 1;
        }
    }
    I2C_Stop();
    return 0;
}

//int I2C_WriteRegister_9250(uint8_t addr, uint8_t reg, uint8_t data)
//{
//    if (!I2C_Start())
//        return 1;
//    I2C_SendByte(addr << 1 | 1);
//    if (!I2C_WaitAck()) {
//        I2C_Stop();
//        return 1;
//    }
//    I2C_SendByte(reg);
//    I2C_WaitAck();
//    I2C_SendByte(data);
//    I2C_WaitAck();
//    I2C_Stop();
//    return 0;
//}

int I2C_ReadRegister_9250(uint8_t addr, uint8_t reg, uint16_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return 1;
    I2C_SendByte(addr << 1 | 0);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return 1;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | 1);
    I2C_WaitAck();
    while (len) {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return 0;
}

uint16_t i2cGetErrorCounter(void)
{
    // TODO maybe fix this, but since this is test code, doesn't matter.
    return 0;
}
*/


void IIC_Start(void)
{

	IIC_SDA_H();
	Delay_us(4);
	IIC_CLK_H();
	Delay_us(4);
 	IIC_SDA_L();//START:when CLK is high,DATA change form high to low 
	Delay_us(4);
	IIC_CLK_L();//ǯסI2C���ߣ�׼�����ͻ�������� 
	Delay_us(4);

}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{

	IIC_CLK_L();
	Delay_us(4);
	IIC_SDA_L();//STOP:when CLK is high DATA change form low to high
 	Delay_us(4);
	IIC_CLK_H(); 
	Delay_us(4);
	IIC_SDA_H();//����I2C���߽����ź�
	Delay_us(4);
	

}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	unsigned int ucErrTime=0;

	IIC_SDA_H();Delay_us(1);	   
	IIC_CLK_H();Delay_us(1);	 
	while(SDA_READ())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
//			printf("no ack\r\n");
			return 1;
		}
	}
	IIC_CLK_L();//ʱ�����0 
	Delay_us(4);
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_CLK_L();

	IIC_SDA_L();
	Delay_us(4);
	IIC_CLK_H();
	Delay_us(4);
	IIC_CLK_L();
	Delay_us(4);
	IIC_SDA_H();
	

	
}
//����NACKӦ��		    
void IIC_NAck(void)
{
	IIC_CLK_L();
	IIC_SDA_H();
	Delay_us(4);
	IIC_CLK_H();
	Delay_us(4);
	IIC_CLK_L();

	
}					 				     
  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
    
    IIC_CLK_L();//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
			if((txd&0x80)>>7)
				IIC_SDA_H();
			else
				IIC_SDA_L();
			txd<<=1; 	  
			Delay_us(4);   //��TEA5767��������ʱ���Ǳ����
			IIC_CLK_H();
			Delay_us(4); 
			IIC_CLK_L();	
			Delay_us(4);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;

    for(i=0;i<8;i++ )
	{
        IIC_CLK_L(); 
        Delay_us(4);
				IIC_CLK_H();
        receive<<=1;
        if(SDA_READ())receive++;   
		Delay_us(2); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}
//---------------------------------------------
int I2C_ReadRegister_9250(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	int ret,i;
	IIC_Start();
	IIC_Send_Byte(Address<<1);
	ret=IIC_Wait_Ack();//ret=1 failed
	if(ret)
	{
		printf("read no ack\r\n");
		IIC_Stop();
		return 1;
	}
	else
	{
		IIC_Send_Byte(RegisterAddr);
		IIC_Wait_Ack();
		IIC_Start();
		IIC_Send_Byte((Address<<1)|0x01);
		IIC_Wait_Ack();
		for(i=0;i<RegisterLen;i++)
		{
			if(i==(RegisterLen-1))
			{RegisterValue[i]=IIC_Read_Byte(0);}
			else
			{RegisterValue[i]=IIC_Read_Byte(1);}
		}
		IIC_Stop();
		return 0;
	}
	
}
int I2C_WriteRegister_9250(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
	int ret,i;
	IIC_Start();
	IIC_Send_Byte(Address<<1);
	ret=IIC_Wait_Ack();//ret=1 failed
	if(ret)
	{
		printf("write no ack\r\n");
		return 1;
	}
	else
	{
		IIC_Send_Byte(RegisterAddr);
		IIC_Wait_Ack();
		for(i=0;i<RegisterLen;i++)
		{
			IIC_Send_Byte(RegisterValue[i]);
			IIC_Wait_Ack();			
		}
		IIC_Stop();
		return 0;
	}
}
	

	


























