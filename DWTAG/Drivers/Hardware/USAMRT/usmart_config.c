#include "usmart.h"
#include "usmart_str.h" 
#include "main.h"
#include "stm32f0xx_hal.h"
#include "Testing.h"
////////////////////////////�û�������///////////////////////////////////////////////
//������Ҫ�������õ��ĺ�����������ͷ�ļ�(�û��Լ����) 


												 
extern void test_fun(void(*ledset)(uint8_t),uint8_t sta);
extern void * test1(void);
//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[]=
{
#if USMART_USE_WRFUNS==1 	//���ʹ���˶�д����
	(void*)read_addr,"uint32_t read_addr(uint32_t addr)",
	(void*)write_addr,"void write_addr(uint32_t addr,uint32_t val)",	 
#endif
	(void*)NRF_Test,"void NRF_Test(uint8_t stat)",
	(void*)unlockflash,"void unlockflash(unsigned int passwd)",
	(void*)going,"void going(void)",
	(void*)Read_status,"void Read_status(void)",
	(void *)Start_dwrx,"void Start_dwrx(void)",
	(void*)ShowTimeStack,"void ShowTimeStack(void)",
	(void*)testI2C,"void testI2C(uint8 resaddr)",
#ifdef SetDelay
		(void*)Set_AtennaDelay,"void Set_AtennaDelay(uint16 delay)",

#endif

											 
};						  
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//�������ƹ�������ʼ��
//�õ������ܿغ���������
//�õ�����������
struct _m_usmart_dev usmart_dev=
{
	usmart_nametab,
	usmart_init,
	usmart_cmd_rec,
	usmart_exe,
	usmart_scan,
	sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),//��������
	0,	  	//��������
	0,	 	//����ID
	1,		//������ʾ����,0,10����;1,16����
	0,		//��������.bitx:,0,����;1,�ַ���	    
	0,	  	//ÿ�������ĳ����ݴ��,��ҪMAX_PARM��0��ʼ��
	0,		//�����Ĳ���,��ҪPARM_LEN��0��ʼ��
};   



















