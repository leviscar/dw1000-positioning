#include "delay.h"


static uint8_t  fac_us=0;//us��ʱ������
static uint16_t fac_ms=0;//ms��ʱ������
// __IO uint16_t msec=0;
// __IO uint16_t usec=0;


//��ʼ���ӳٺ���
//��ʹ��ucos��ʱ��,�˺������ʼ��ucos��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void delay_init()	 
{

//	  if (SysTick_Config(SystemCoreClock / 8000))//The function initializes the System Timer and its interrupt, and starts the System Tick Timer. Counter is in free running mode to generate periodic interrupts.
//  { 
//    /* Capture error */ 
//    while (1);
//  }
	SysTick->CTRL&=~(SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);	//ѡ��ʱ��  HCLK
	fac_us=SystemCoreClock/8000000;
	fac_ms=SystemCoreClock/8000;
	 

}								    

//void Delay_ms(__IO uint32_t nTime)
//{ 
//  __IO uint16_t temp1=0;
//	uint16_t flag=0;
//	temp1=msec;
//	while(flag<(nTime))
//	{
//		if(msec>=temp1)
//		flag=(msec-temp1);
//		else
//		flag=(65535-temp1+msec);
//	};
//}
//void Delay_us(__IO uint32_t nTime)
//{ 
//   __IO uint16_t temp1;
//	temp1=usec;
//	while((usec>temp1)?(usec-temp1):(temp1-usec)<nTime);
//}


//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
void Delay_us(volatile uint32_t nus)
{		
	volatile uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
void Delay_ms(volatile uint16_t nms)
{	 		  	  
	volatile uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
} 

void get_msticks(unsigned long *timestamp)
{
	*timestamp=NULL;
}

void HAL_Delay(__IO uint32_t Delay)
{
	Delay_ms(Delay);
}
