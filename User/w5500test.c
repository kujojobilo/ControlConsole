/**
  ******************************************************************************

  * @author  王田
  * @date    2023/02/15
  * @brief   Main program body
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_tim4.h"
#include "stm8s_it.h"
#include "stm8s_gpio.h"
#include "stm8s_clk.h"
#include "spi_user.h"
#include "w5500.h"


/* Private defines -----------------------------------------------------------*/
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t




/* Private function prototypes -----------------------------------------------*/

void watchDog();//配置看门狗CLK输出
void CLK_Init();//时钟切换初始化
void delay(int k);
void w5500_Reset();//重启w5500
void w5500_init();


u8 mac[6]={0x00,0x08,0xdc,0x01,0x02,0x03};//定义mac地址
u8 lip[4]={192,168,10,100};//定义IP
u8 subnet[4]={255,255,255,0};//定义子网掩码
u8 gateway[4]={192,168,10,1};//定义网关
u8 txsize[] = {128,128,128,128,128,128,128,128};//8个socket缓冲区大小设置
u8 rxsize[] = {128,128,128,128,128,128,128,128};
int index = 0;
u8 buf[64]={0};
/* Private functions ---------------------------------------------------------*/




void watchDog(){//配置看门狗CLK输出
  GPIO_Init(GPIOD,GPIO_PIN_7,GPIO_MODE_OUT_PP_LOW_SLOW);//配置PD7推挽低速输出
  TIM4_DeInit();
  TIM4_TimeBaseInit(TIM4_PRESCALER_128,1250);//频率为16/128=125khz，清除计数为1250，即每10ms触发中断
  TIM4_Cmd(ENABLE);
  TIM4_ITConfig(TIM4_IT_UPDATE,ENABLE);
  ITC_SetSoftwarePriority(ITC_IRQ_TIM4_OVF,ITC_PRIORITYLEVEL_3);//设定高中断等级
  CLK_ITConfig(CLK_PERIPHERAL_TIMER4,ENABLE);
  
}


void CLK_Init(){//时钟切换初始化
     //使能外部时钟HSE
  CLK_HSECmd(ENABLE);
  // CPU时钟分频1，CPU时钟 = 外部时钟 , 此处为16Mhz
  CLK->CKDIVR |= CLK_PRESCALER_CPUDIV1;       
  //切换模式为自动切换，时钟选取为外部时钟HSE，禁用中断，启用被选取时钟
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);
  CLK->SWCR &= ~(CLK_SWCR_SWEN|CLK_SWCR_SWIF);
}

void delay(int k){
  while(k--);
}

void w5500_Reset(){//重启w5500,RST低电平有效
  GPIO_WriteLow(W5500_RST_PORT,W5500_RST_PIN);
  delay(10);
  GPIO_WriteHigh(W5500_RST_PORT,W5500_RST_PIN);
  delay(100);
}

void w5500_init(){//对w5500做初始化设置
  w5500_GPIOInit();//相关GPIO初始化
  w5500_Reset();
  SPI_Setup();//SPI相关设置
  setSHAR(mac);//设置MAC地址
  setSUBR(subnet);//设置子网掩码
  setGAR(gateway);//设置网关
  setSIPR(lip); //设置IP
  sysinit(txsize,rxsize);//初始化8个socket的收发缓冲区
  setRTR(2000); //发送超时为2000
  setRCR(3);//超时重传3次
}



void assert_failed(uint8_t* file, uint32_t line){
	printf("error");
}//无意义，不加会报错



#pragma vector = 0x19//TIM4喂狗
__interrupt void TIM4_UPD_OVF_IRQHandler(){
  GPIO_WriteReverse(GPIOD,GPIO_PIN_7);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);//清除相关计数器
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
}
           


/* Main functions ---------------------------------------------------------*/
int main( void )
{
  delay(50);
  CLK_Init();
  delay(50);
  watchDog();
  //GPIO_WriteHigh(GPIOD,GPIO_PIN_7);
  rim();
  delay(50);
  w5500_init();
  delay(50);
  
  while(1);
}
/******************************************END OF FILE****/