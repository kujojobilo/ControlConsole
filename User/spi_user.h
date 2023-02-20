/**
  ******************************************************************************

  * @author  王田
  * @date    2023/02/15
  * @brief   部分自用SPI代码抽离集中
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"


/* defines -------------------------------------------------------------------*/
#ifndef	SPI_USER_H
#define	SPI_USER_H
    
    
#define SPI_CLK_PORT GPIOC
#define SPI_CLK_PIN GPIO_PIN_5

#define SPI_MOSI_PORT GPIOC
#define SPI_MOSI_PIN GPIO_PIN_6

#define SPI_MISO_PORT GPIOC
#define SPI_MISO_PIN GPIO_PIN_7

//函数操作
void SPI_Setup();//设置基本参数
u8 SPI_SendByte(u8 data);//发送单字节
u8 SPI_RecieveByte();//接收单字节
//void WIZ_CS(uint8_t val);//控制w5500的连接


#endif