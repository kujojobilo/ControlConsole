/**
  ******************************************************************************

  * @author  王田
  * @date    2023/02/08
  * @brief   I2C Simulator ( using GPIO )
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_gpio.h"

/* defines -------------------------------------------------------------------*/
#ifndef	I2C_SIM_H
#define	I2C_SIM_H
//端口引脚定义
#define SCL_PORT GPIOE
#define SCL_PIN GPIO_PIN_1
#define SDA_PORT GPIOE
#define SDA_PIN GPIO_PIN_2
//引脚高低电平操作定义
#define I2C_SCL_OUT        GPIO_Init(SCL_PORT,SCL_PIN,GPIO_MODE_OUT_OD_HIZ_FAST);
#define I2C_SCL_SET        GPIO_WriteHigh(SCL_PORT,SCL_PIN); 
#define I2C_SCL_CLR        GPIO_WriteLow(SCL_PORT,SCL_PIN); 
#define I2C_SCL_REVERSE    GPIO_WriteReverse(SCL_PORT,SCL_PIN); 

#define I2C_SDA_OUT        GPIO_Init(SDA_PORT,SDA_PIN,GPIO_MODE_OUT_OD_HIZ_FAST);
#define I2C_SDA_SET        GPIO_WriteHigh(SDA_PORT,SDA_PIN);
#define I2C_SDA_CLR        GPIO_WriteLow(SDA_PORT,SDA_PIN);
#define I2C_SDA_IN         GPIO_Init(SDA_PORT,SDA_PIN,GPIO_MODE_IN_FL_NO_IT); 


#endif