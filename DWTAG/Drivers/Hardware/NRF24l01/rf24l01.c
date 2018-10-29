//////////////////////////////////////////////
//  * Ӳ������ ----------------------------
//  *         | P??-CE          :24L01-CE |
//  *         | P??-IRQ  :      24L01-IRQ |
//  *         | P??-CS  :        24L01-CS  |
//  *         | P??-SPI1-SCK  : 24L01-CLK |
//  *         | P??-SPI1-MISO : 24L01-DO  |
//  *         | P??-SPI1-MOSI : 24L01-DIO |
//  *          ----------------------------
//  * ��汾  ��ST3.0.0
//  *
////////////////////////////////////////////

#include  "rf24l01.h"

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x68,0x86,0x66,0x88,0x28}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x68,0x86,0x66,0x88,0x28}; //���͵�ַ
uint8_t SPI_RF_SendByte(uint8_t temp);
uint8_t nrfsta=0;//0:rece 1: trans
uint8_t TransCop=0;
/**************************************
* ������   : SPI_RF_Init
* ����     : RF24L01��ʼ��
* ������� : ��
* ����ֵ   : ��
**************************************/


/**************************************
* ������   : NRF24L01_Write_Reg
* ����     : ��?4L01�ļĴ���дֵ��һ���ֽڣ�
* ������� : uint8_t reg,uint8_t value
*            reg    Ҫд�ļĴ�����ַ  
*            value  ���Ĵ���д��ֵ
* ����ֵ   : һ���ֽڵ�״ֵ̬ 
**************************************/
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;

	SPI_RF_CS_LOW() ;	 //CSN=0;
	
	status=SPI_RF_SendByte(reg);
	SPI_RF_SendByte(value);
	
//	HAL_SPI_TransmitReceive(&hspi1,&reg,&status,1,500);
//	HAL_SPI_Transmit(&hspi1,&value,1,500);
//	HAL_SPIEx_FlushRxFifo(&hspi1);

	SPI_RF_CS_HIGH();   //CSN=1;
	return status;
}

/*************************************************/
/* �������ܣ���24L01�ļĴ���ֵ ��һ���ֽڣ�      */
/* ��ڲ�����reg  Ҫ���ļĴ�����ַ               */
/* ���ڲ�����value �����Ĵ�����ֵ                */
/*************************************************/
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
 	uint8_t value;

	SPI_RF_CS_LOW() ; //CSN=0; 
//	status=SPI_RF_SendByte(reg);
	value=SPI_RF_SendByte(0XFF);
  
//  HAL_SPI_Transmit(&hspi1,&reg,1,500);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
//	HAL_SPIEx_FlushRxFifo(&hspi1);
//  HAL_SPI_Receive(&hspi1,&value,1,500);
	
	SPI_RF_CS_HIGH();  //CSN=1;
	return value;
}

/*********************************************/
/* �������ܣ���24L01�ļĴ���ֵ������ֽڣ�   */
/* ��ڲ�����reg   �Ĵ�����ַ                */
/*           *pBuf �����Ĵ���ֵ�Ĵ������    */
/*           len   �����ֽڳ���              */
/* ���ڲ�����status ״ֵ̬                   */
/*********************************************/
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,i;
	SPI_RF_CS_LOW() ;//CSN=0


	status=SPI_RF_SendByte(reg);
	for(i=0;i<len;i++)pBuf[i]=SPI_RF_SendByte(0XFF);//��������
	
//	HAL_SPI_TransmitReceive(&hspi1,&reg,&status,1,500);
//	HAL_SPI_Receive(&hspi1,pBuf,len,500);
	SPI_RF_CS_HIGH(); //CSN=1
  return status;        //���ض�����״ֵ̬
}

/**********************************************/
/* �������ܣ���24L01�ļĴ���дֵ������ֽڣ�  */
/* ��ڲ�����reg  Ҫд�ļĴ�����ַ            */
/*           *pBuf ֵ�Ĵ������               */
/*           len   �����ֽڳ���               */
/**********************************************/
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,i;
	SPI_RF_CS_LOW() ;
	

	status=SPI_RF_SendByte(reg);
	for(i=0; i<len; i++)SPI_RF_SendByte(*pBuf++); //д������
	
//  HAL_SPI_TransmitReceive(&hspi1,&reg,&status,1,500);
//	HAL_SPI_Transmit(&hspi1,pBuf,len,500);
//	HAL_SPIEx_FlushRxFifo(&hspi1);

	SPI_RF_CS_HIGH(); 
  return status;          //���ض�����״ֵ̬
}

/********************************************/
/* �������ܣ����24L01�Ƿ����              */
/* ����ֵ��  0  ����                        */
/*           1  ������                      */
/********************************************/	
uint8_t NRF24L01_Check(void)
{
	uint8_t check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
	uint8_t check_out_buf[5]={0x00};

	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR, check_in_buf, 5);

	NRF24L01_Read_Buf(NRF_READ_REG+TX_ADDR, check_out_buf, 5);

	if((check_out_buf[0] == 0x11)&&\
	   (check_out_buf[1] == 0x22)&&\
	   (check_out_buf[2] == 0x33)&&\
	   (check_out_buf[3] == 0x44)&&\
	   (check_out_buf[4] == 0x55))return 0;
	else return 1;
}

/*********************************************/
/* �������ܣ�����24L01Ϊ����ģʽ             */
/*********************************************/
void NRF24L01_RX_Mode(void)
{
	nrfsta=0;
	SPI_RF_CE_LOW() ;	//CE���ͣ�ʹ��24L01����
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);//дRX���յ�ַ
	  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //����ͨ��0�Զ�Ӧ��    
  	//NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x00);    //�ر��Զ�Ӧ��    
		NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ͨ��0��������  	 
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //����RF����ͨ��Ƶ�� 		  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
		NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪�� 	     
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
	SPI_RF_CE_HIGH();	//CE�øߣ�ʹ�ܽ���
	Delay_us(130); 
}

/*********************************************/
/* �������ܣ�����24L01Ϊ����ģʽ             */
/*********************************************/
void NRF24L01_TX_Mode(void)
{
		nrfsta=1;
		SPI_RF_CE_LOW() ;	//CE���ͣ�ʹ��24L01����	    
  	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //�ر������Զ�Ӧ��    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //�ر�ͨ��0�Ľ��յ�ַ  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x01);//no�����Զ��ط����ʱ��:250us + 86us;����Զ��ط�����:1��
  	
		NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	  NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		SPI_RF_CE_HIGH();	//CE�øߣ�ʹ�ܷ���
		Delay_us(10);
}

/*********************************************/
/* �������ܣ�24L01��������                   */
/* ��ڲ�����rxbuf ������������              */
/* ����ֵ�� 0   �ɹ��յ�����                 */
/*          1   û���յ�����                 */
/*********************************************/
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;

	state=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //�����־
	if(state&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}

/**********************************************/
/* �������ܣ�����24L01Ϊ����ģʽ              */
/* ��ڲ�����txbuf  ������������              */
/* ����ֵ�� 0x10    �ﵽ����ط�����������ʧ��*/
/*          0x20    �ɹ��������              */
/*          0xff    ����ʧ��                  */
/**********************************************/
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
   
	SPI_RF_CE_LOW() ;	//CE���ͣ�ʹ��24L01����
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	SPI_RF_CE_HIGH();	//CE�øߣ�ʹ�ܷ���
//	while (RF_READINTPIN()!=0);//�ȴ�������� (note: this can be changed)
	
	
	while(TransCop!=1);
	TransCop=0;	
	state=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
	if(state&MAX_TX)//�ﵽ����ط�����
	{
		
		return MAX_TX; 
	}
	if(state&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ʧ��
}	


uint8_t SPI_RF_SendByte(uint8_t temp)
{
	while(((SPI1->SR)&SPI_FLAG_TXE)!=SPI_FLAG_TXE);
	
	*((__IO uint8_t *)&SPI1->DR) =temp;
	
	while(((SPI1->SR)&SPI_FLAG_RXNE)!=SPI_FLAG_RXNE);
	
	return (uint8_t)SPI1->DR;
	
}



/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/



