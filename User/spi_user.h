/**
  ******************************************************************************

  * @author  ����
  * @date    2023/02/15
  * @brief   ��������SPI������뼯��
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

//��������
void SPI_Setup();//���û�������
u8 SPI_SendByte(u8 data);//���͵��ֽ�
u8 SPI_RecieveByte();//���յ��ֽ�
//void WIZ_CS(uint8_t val);//����w5500������


#endif