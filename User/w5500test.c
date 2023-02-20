/**
  ******************************************************************************

  * @author  ����
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

void watchDog();//���ÿ��Ź�CLK���
void CLK_Init();//ʱ���л���ʼ��
void delay(int k);
void w5500_Reset();//����w5500
void w5500_init();


u8 mac[6]={0x00,0x08,0xdc,0x01,0x02,0x03};//����mac��ַ
u8 lip[4]={192,168,10,100};//����IP
u8 subnet[4]={255,255,255,0};//������������
u8 gateway[4]={192,168,10,1};//��������
u8 txsize[] = {128,128,128,128,128,128,128,128};//8��socket��������С����
u8 rxsize[] = {128,128,128,128,128,128,128,128};
int index = 0;
u8 buf[64]={0};
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

void delay(int k){
  while(k--);
}

void w5500_Reset(){//����w5500,RST�͵�ƽ��Ч
  GPIO_WriteLow(W5500_RST_PORT,W5500_RST_PIN);
  delay(10);
  GPIO_WriteHigh(W5500_RST_PORT,W5500_RST_PIN);
  delay(100);
}

void w5500_init(){//��w5500����ʼ������
  w5500_GPIOInit();//���GPIO��ʼ��
  w5500_Reset();
  SPI_Setup();//SPI�������
  setSHAR(mac);//����MAC��ַ
  setSUBR(subnet);//������������
  setGAR(gateway);//��������
  setSIPR(lip); //����IP
  sysinit(txsize,rxsize);//��ʼ��8��socket���շ�������
  setRTR(2000); //���ͳ�ʱΪ2000
  setRCR(3);//��ʱ�ش�3��
}



void assert_failed(uint8_t* file, uint32_t line){
	printf("error");
}//�����壬���ӻᱨ��



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
  //GPIO_WriteHigh(GPIOD,GPIO_PIN_7);
  rim();
  delay(50);
  w5500_init();
  delay(50);
  
  while(1);
}
/******************************************END OF FILE****/