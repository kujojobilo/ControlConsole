/**
  ******************************************************************************

  * @author  王田
  * @date    2023/02/08
  * @brief   Main program body
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_tim2.h"
#include "stm8s_tim3.h"
#include "stm8s_tim4.h"
#include "stm8s_it.h"
#include "stm8s_gpio.h"
#include "stm8s_clk.h"
#include "i2c_sim.h"


/* Private defines -----------------------------------------------------------*/
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

int status = 0;
/*
static bool I2C_send[8]={0,1,0,0,1,1,0,1};//发送读指令（CH456指令为0x4D);
bool I2C_rcv[10]={0};//接收容量，包括ACK1，接收字节，ACK2三部分，共10位
bool flag_rcv = 0;
int byte_count = 0;
*/
u8 data;


/* Private function prototypes -----------------------------------------------*/

void watchDog();//配置看门狗CLK输出
void CLK_Init();//时钟切换初始化
void I2C_Init_u();//I2C初始化
void I2C_Start();//I2C发送起始信号
void I2C_Stop();//I2C发送停止信号
void I2C_WriteByte(u8 data);//I2C写一个字节
u8 I2C_ReadByte();//I2C读一个字节
void I2C_Write1();//写配置，完成I2C驱动设置
void I2C_Write2();//设置I2C引脚
u8 I2C_Read();//I2C完成读操作
void delay(int k);

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

void I2C_Init_u(){//I2C初始化
  I2C_SDA_OUT;
  I2C_SDA_SET;
  I2C_SCL_OUT;
  I2C_SCL_SET;
  /*
  //设置定时器对I2C起到模拟时钟作用，此处初始化时不使能
  TIM2_DeInit();
  TIM2_TimeBaseInit(TIM2_PRESCALER_16,10);//频率为16/16=1Mhz，清除计数为10，即开启后10us触发中断
  TIM2_ITConfig(TIM2_IT_UPDATE,ENABLE);
  ITC_SetSoftwarePriority(ITC_IRQ_TIM2_OVF,ITC_PRIORITYLEVEL_2);//设定次级中断等级
  CLK_ITConfig(CLK_PERIPHERAL_TIMER2,ENABLE);
  */
  //设置中断进行定期I2C数据读取，扫描按键状态
  TIM3_DeInit();
  TIM3_TimeBaseInit(TIM3_PRESCALER_128,125000);//频率为16/128=125khz，清除计数为125000，即开启100ms触发中断
  TIM3_ITConfig(TIM3_IT_UPDATE,ENABLE);
  ITC_SetSoftwarePriority(ITC_IRQ_TIM3_OVF,ITC_PRIORITYLEVEL_2);//设定次级中断等级
  CLK_ITConfig(CLK_PERIPHERAL_TIMER3,ENABLE);
  TIM3_Cmd(ENABLE);
}



void I2C_Start(){//I2C发送起始信号
  I2C_SDA_OUT;
  I2C_SCL_OUT;
  I2C_SDA_SET;
  I2C_SCL_SET;
  delay(20);
  I2C_SDA_CLR;
  delay(20);
  I2C_SCL_CLR;
  return;
}

void I2C_Stop(){//I2C发送停止信号
  I2C_SDA_CLR;
  delay(20);
  I2C_SCL_SET;
  delay(20);
  I2C_SDA_SET;
  delay(20);
  return;
}


void I2C_WriteByte(u8 data){//I2C写一个字节
  for(int i = 0; i != 8; i++){
    if((data & 0x80) >> 7) 
        {I2C_SDA_SET;}
    else {I2C_SDA_CLR;}
    delay(20);
    I2C_SCL_SET;
    data <<= 1;
    delay(20);
    I2C_SCL_CLR;
  }
  I2C_SDA_SET;//应答码1
  delay(20);
  I2C_SCL_SET;
  delay(20);
  I2C_SCL_CLR;
  return;
}



u8 I2C_ReadByte(){//I2C读一个字节
  u8 data =0;
  for(int i = 0; i != 8; i++){
    delay(20);
    I2C_SCL_SET;
    delay(20);
    data <<= 1;
    if((GPIO_ReadInputData(SDA_PORT) & (u8)SDA_PIN) >> 2) 
    {data++;}
    I2C_SCL_CLR;
  }
  I2C_SDA_OUT;//应答码2
  I2C_SDA_SET;
  delay(20);
  I2C_SCL_SET;
  delay(20);
  I2C_SCL_CLR;
  return data;
}

void I2C_Write1(){
  I2C_Start();
  I2C_WriteByte(0x48);
  I2C_WriteByte(0x0A);
  I2C_Stop();
}

void I2C_Write2(){
  I2C_Start();
  u8 base = 0x60;
  I2C_WriteByte(base);
  I2C_WriteByte(0xFF);
  I2C_Stop();
}


u8 I2C_Read(){//I2C完成读操作
  u8 data;
  
  I2C_Start();
  I2C_WriteByte(0x4D);
  I2C_SDA_IN;
  data = I2C_ReadByte();
  I2C_Stop();
  return data;
}

void delay(int k){
   while(k--);
}




void assert_failed(uint8_t* file, uint32_t line){}//无意义，不加会报错

/*
#pragma vector = 0x0F//TIM2中断，模拟I2C时钟变化
__interrupt void TIM2_UPD_OVF_IRQHandler(){
  I2C_SCL_REVERSE;
  if( (GPIO_ReadOutputData(SCL_PORT) & (uint8_t)SCL_PIN) >> 1){//时钟为1时进行读写操作
    if(flag_rcv){//接收状态
      if(byte_count == 11){//发送停止信号，清除byte_count和flag_rcv
          delay(15);
          I2C_SDA_SET;
          I2C_SCL_SET;
          byte_count = 0;
          flag_rcv = 0;
          TIM2_ClearFlag(TIM2_FLAG_UPDATE);//清除相关计数器
          TIM2_ClearITPendingBit(TIM2_IT_UPDATE); 
          TIM2_Cmd(DISABLE);
          TIM3_Cmd(ENABLE);
          return;
        }
        I2C_rcv[byte_count] =(bool) ((GPIO_ReadOutputData(SDA_PORT) & (uint8_t)SDA_PIN) >> 2);
        byte_count++;
      }else{//发送状态
        if(I2C_send[byte_count]){I2C_SDA_SET;}//发送指令为1则发送1
        else                    {I2C_SDA_CLR;}//发送指令为0则发送0
        byte_count++;
          if(byte_count == 8){//发送8位指令后清零
            flag_rcv = 1;
            byte_count = 0;
            I2C_SDA_IN;
          }
     }
  }else{
    if(byte_count == 10){//接收完10位命令后，准备发送停止信号
      I2C_SDA_OUT;
      I2C_SDA_CLR;
      byte_count++;
    }
  }
  TIM2_ClearFlag(TIM2_FLAG_UPDATE);//清除相关计数器
  TIM2_ClearITPendingBit(TIM2_IT_UPDATE); 
}
*/

#pragma vector = 0x11//TIM3中断，进行I2C读取操作
__interrupt void TIM3_UPD_OVF_IRQHandler(){
  data = I2C_Read();
  //I2C_Write1();
  TIM3_ClearFlag(TIM3_FLAG_UPDATE);//清除相关计数器
  TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
  //TIM3_Cmd(DISABLE);
  //TIM2_Cmd(ENABLE);
}

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
  delay(50);
  I2C_Init_u();
 I2C_Write1();
// I2C_Write2();
  rim();
  while(1);
}
/******************************************END OF FILE****/