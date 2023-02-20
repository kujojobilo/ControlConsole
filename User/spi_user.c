/**
  ******************************************************************************

  * @author  ����
  * @date    2023/02/15
  * @brief   ��������SPI������뼯��
  ******************************************************************************
  */ 


#include "spi_user.h"
#include "stm8s_spi.h"

void SPI_Setup(){//���û�������
  SPI_DeInit();
  GPIO_Init(SPI_CLK_PORT,SPI_CLK_PIN,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(SPI_MOSI_PORT,SPI_MOSI_PIN,GPIO_MODE_OUT_PP_HIGH_FAST);//����SPI_SCK��SPI_MOSIΪ�����������
  GPIO_Init(SPI_MISO_PORT,SPI_MISO_PIN,GPIO_MODE_IN_PU_IT);//����SPI_MISOΪ�������ж�����
  SPI_Init(SPI_FIRSTBIT_MSB, //��λ��ǰ
              SPI_BAUDRATEPRESCALER_16, //��Ƶ��16��Ƶ����1Mhz
              SPI_MODE_MASTER, //��Ƭ��Ϊ���豸
              SPI_CLOCKPOLARITY_LOW, //SPIʱ������ʱΪ0
              SPI_CLOCKPHASE_1EDGE, //��һ��ʱ������Ч
              SPI_DATADIRECTION_2LINES_FULLDUPLEX,//˫�ߵ���ģʽ 
              SPI_NSS_HARD,//Ӳ�����ƴԻ�
              0x70);//CRCPR�Ĵ�����������CRCУ�飬����ΪĬ��ֵ0x70
  SPI_ITConfig(SPI_IT_RXNE, ENABLE);//ʹ�ܽ����ж�
  SPI_Cmd(ENABLE);
}


u8 SPI_SendByte(u8 data){//���͵��ֽ�
  
  while( !((SPI->SR & 0x02)>>1));
  SPI_SendData(data);
  while(!(SPI->SR &0x01));
  u8 byte = SPI_ReceiveData();
  return byte;
}


u8 SPI_RecieveByte(){//���յ��ֽ�
  while(!(SPI->SR &0x01));
  u8 data = SPI_ReceiveData();
  return data;
}

/*
void WIZ_CS(uint8_t val)
{
  if(val){
    GPIO_WriteHigh(W5500_CSN_PORT,W5500_CSN_PIN);
  }else{
    GPIO_WriteLow(W5500_CSN_PORT,W5500_CSN_PIN);
  }
}

*/



