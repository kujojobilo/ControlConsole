/**
  ******************************************************************************

  * @author  ����
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
static bool I2C_send[8]={0,1,0,0,1,1,0,1};//���Ͷ�ָ�CH456ָ��Ϊ0x4D);
bool I2C_rcv[10]={0};//��������������ACK1�������ֽڣ�ACK2�����֣���10λ
bool flag_rcv = 0;
int byte_count = 0;
*/
u8 data;


/* Private function prototypes -----------------------------------------------*/

void watchDog();//���ÿ��Ź�CLK���
void CLK_Init();//ʱ���л���ʼ��
void I2C_Init_u();//I2C��ʼ��
void I2C_Start();//I2C������ʼ�ź�
void I2C_Stop();//I2C����ֹͣ�ź�
void I2C_WriteByte(u8 data);//I2Cдһ���ֽ�
u8 I2C_ReadByte();//I2C��һ���ֽ�
void I2C_Write1();//д���ã����I2C��������
void I2C_Write2();//����I2C����
u8 I2C_Read();//I2C��ɶ�����
void delay(int k);

/* Private functions ---------------------------------------------------------*/




void watchDog(){//���ÿ��Ź�CLK���
  GPIO_Init(GPIOD,GPIO_PIN_7,GPIO_MODE_OUT_PP_LOW_SLOW);//����PD7����������
  TIM4_DeInit();
  TIM4_TimeBaseInit(TIM4_PRESCALER_128,1250);//Ƶ��Ϊ16/128=125khz���������Ϊ1250����ÿ10ms�����ж�
  TIM4_Cmd(ENABLE);
  TIM4_ITConfig(TIM4_IT_UPDATE,ENABLE);
  ITC_SetSoftwarePriority(ITC_IRQ_TIM4_OVF,ITC_PRIORITYLEVEL_3);//�趨���жϵȼ�
  CLK_ITConfig(CLK_PERIPHERAL_TIMER4,ENABLE);
  
}


void CLK_Init(){//ʱ���л���ʼ��
     //ʹ���ⲿʱ��HSE
  CLK_HSECmd(ENABLE);
  // CPUʱ�ӷ�Ƶ1��CPUʱ�� = �ⲿʱ�� , �˴�Ϊ16Mhz
  CLK->CKDIVR |= CLK_PRESCALER_CPUDIV1;       
  //�л�ģʽΪ�Զ��л���ʱ��ѡȡΪ�ⲿʱ��HSE�������жϣ����ñ�ѡȡʱ��
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);
  CLK->SWCR &= ~(CLK_SWCR_SWEN|CLK_SWCR_SWIF);
}

void I2C_Init_u(){//I2C��ʼ��
  I2C_SDA_OUT;
  I2C_SDA_SET;
  I2C_SCL_OUT;
  I2C_SCL_SET;
  /*
  //���ö�ʱ����I2C��ģ��ʱ�����ã��˴���ʼ��ʱ��ʹ��
  TIM2_DeInit();
  TIM2_TimeBaseInit(TIM2_PRESCALER_16,10);//Ƶ��Ϊ16/16=1Mhz���������Ϊ10����������10us�����ж�
  TIM2_ITConfig(TIM2_IT_UPDATE,ENABLE);
  ITC_SetSoftwarePriority(ITC_IRQ_TIM2_OVF,ITC_PRIORITYLEVEL_2);//�趨�μ��жϵȼ�
  CLK_ITConfig(CLK_PERIPHERAL_TIMER2,ENABLE);
  */
  //�����жϽ��ж���I2C���ݶ�ȡ��ɨ�谴��״̬
  TIM3_DeInit();
  TIM3_TimeBaseInit(TIM3_PRESCALER_128,125000);//Ƶ��Ϊ16/128=125khz���������Ϊ125000��������100ms�����ж�
  TIM3_ITConfig(TIM3_IT_UPDATE,ENABLE);
  ITC_SetSoftwarePriority(ITC_IRQ_TIM3_OVF,ITC_PRIORITYLEVEL_2);//�趨�μ��жϵȼ�
  CLK_ITConfig(CLK_PERIPHERAL_TIMER3,ENABLE);
  TIM3_Cmd(ENABLE);
}



void I2C_Start(){//I2C������ʼ�ź�
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

void I2C_Stop(){//I2C����ֹͣ�ź�
  I2C_SDA_CLR;
  delay(20);
  I2C_SCL_SET;
  delay(20);
  I2C_SDA_SET;
  delay(20);
  return;
}


void I2C_WriteByte(u8 data){//I2Cдһ���ֽ�
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
  I2C_SDA_SET;//Ӧ����1
  delay(20);
  I2C_SCL_SET;
  delay(20);
  I2C_SCL_CLR;
  return;
}



u8 I2C_ReadByte(){//I2C��һ���ֽ�
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
  I2C_SDA_OUT;//Ӧ����2
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


u8 I2C_Read(){//I2C��ɶ�����
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




void assert_failed(uint8_t* file, uint32_t line){}//�����壬���ӻᱨ��

/*
#pragma vector = 0x0F//TIM2�жϣ�ģ��I2Cʱ�ӱ仯
__interrupt void TIM2_UPD_OVF_IRQHandler(){
  I2C_SCL_REVERSE;
  if( (GPIO_ReadOutputData(SCL_PORT) & (uint8_t)SCL_PIN) >> 1){//ʱ��Ϊ1ʱ���ж�д����
    if(flag_rcv){//����״̬
      if(byte_count == 11){//����ֹͣ�źţ����byte_count��flag_rcv
          delay(15);
          I2C_SDA_SET;
          I2C_SCL_SET;
          byte_count = 0;
          flag_rcv = 0;
          TIM2_ClearFlag(TIM2_FLAG_UPDATE);//�����ؼ�����
          TIM2_ClearITPendingBit(TIM2_IT_UPDATE); 
          TIM2_Cmd(DISABLE);
          TIM3_Cmd(ENABLE);
          return;
        }
        I2C_rcv[byte_count] =(bool) ((GPIO_ReadOutputData(SDA_PORT) & (uint8_t)SDA_PIN) >> 2);
        byte_count++;
      }else{//����״̬
        if(I2C_send[byte_count]){I2C_SDA_SET;}//����ָ��Ϊ1����1
        else                    {I2C_SDA_CLR;}//����ָ��Ϊ0����0
        byte_count++;
          if(byte_count == 8){//����8λָ�������
            flag_rcv = 1;
            byte_count = 0;
            I2C_SDA_IN;
          }
     }
  }else{
    if(byte_count == 10){//������10λ�����׼������ֹͣ�ź�
      I2C_SDA_OUT;
      I2C_SDA_CLR;
      byte_count++;
    }
  }
  TIM2_ClearFlag(TIM2_FLAG_UPDATE);//�����ؼ�����
  TIM2_ClearITPendingBit(TIM2_IT_UPDATE); 
}
*/

#pragma vector = 0x11//TIM3�жϣ�����I2C��ȡ����
__interrupt void TIM3_UPD_OVF_IRQHandler(){
  data = I2C_Read();
  //I2C_Write1();
  TIM3_ClearFlag(TIM3_FLAG_UPDATE);//�����ؼ�����
  TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
  //TIM3_Cmd(DISABLE);
  //TIM2_Cmd(ENABLE);
}

#pragma vector = 0x19//TIM4ι��
__interrupt void TIM4_UPD_OVF_IRQHandler(){
  GPIO_WriteReverse(GPIOD,GPIO_PIN_7);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);//�����ؼ�����
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