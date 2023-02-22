/**
  ******************************************************************************

  * @author  王田
  * @date    2023/02/15
  * @brief   部分自用SPI代码抽离集中
  ******************************************************************************
  */ 


#include "spi_user.h"
#include "stm8s_spi.h"

void SPI_Setup(){//设置基本参数
  SPI_DeInit();
  GPIO_Init(SPI_CLK_PORT,SPI_CLK_PIN,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(SPI_MOSI_PORT,SPI_MOSI_PIN,GPIO_MODE_OUT_PP_HIGH_FAST);//设置SPI_SCK，SPI_MOSI为快速推挽输出
  GPIO_Init(SPI_MISO_PORT,SPI_MISO_PIN,GPIO_MODE_IN_PU_NO_IT);//设置SPI_MISO为上拉带中断输入
  SPI_Init(SPI_FIRSTBIT_MSB, //首位在前
              SPI_BAUDRATEPRESCALER_16, //主频的16分频，即1Mhz
              SPI_MODE_MASTER, //单片机为主设备
              SPI_CLOCKPOLARITY_LOW, //SPI时钟线闲时为1
              SPI_CLOCKPHASE_1EDGE, //第2个时钟沿有效
              SPI_DATADIRECTION_2LINES_FULLDUPLEX,//双线单向模式 
              SPI_NSS_SOFT,//软件控制丛机，NSS口为普通GPIO口
              0x07);//CRCPR寄存器，不开启CRC校验，设置为默认值0x00
  //SPI_ITConfig(SPI_IT_RXNE, DISABLE);//关闭中断
  SPI_Cmd(ENABLE);
}


u8 SPI_SendByte(u8 data){//发送单字节
  
  while( !((SPI->SR & 0x02)>>1));
  SPI_SendData(data);
  while(!(SPI->SR &0x01));
  u8 byte = SPI_ReceiveData();
  return byte;
}


u8 SPI_RecieveByte(){//接收单字节
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



