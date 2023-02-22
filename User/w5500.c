
/**
  ******************************************************************************

  * @author  王田
  * @date    2023/02/15
  * @brief   与w5500相关的代码
  ******************************************************************************
  */ 

#include "w5500.h"
#include "spi_user.h"


static uint8_t I_STATUS[MAX_SOCK_NUM];
static uint16_t SSIZE[MAX_SOCK_NUM]; /**< Max Tx buffer size by each channel */
static uint16_t RSIZE[MAX_SOCK_NUM]; /**< Max Rx buffer size by each channel */


/*
********************************以下为自行添加的内容
*/

void w5500_GPIOInit(){//初始化与w5500相关的GPIO引脚
  GPIO_Init(W5500_RST_PORT,W5500_RST_PIN,GPIO_MODE_OUT_PP_LOW_SLOW);//PB2为控制w5500重启的引脚
  GPIO_Init(W5500_INT_PORT,W5500_INT_PIN,GPIO_MODE_IN_PU_NO_IT);//PB1为w5500发送中断的引脚
  GPIO_Init(W5500_CSN_PORT,W5500_CSN_PIN,GPIO_MODE_OUT_PP_HIGH_SLOW);
}

void w5500_CSON(){
	GPIO_WriteHigh(W5500_CSN_PORT,W5500_CSN_PIN);
}
void w5500_CSOFF(){
	GPIO_WriteLow(W5500_CSN_PORT,W5500_CSN_PIN);
}



/*
********************************以下为官网例程的内容
*/
void IINCHIP_WRITE(uint16_t addr, uint8_t bsb,  uint8_t data){//MCU将单字节数据写入指定地址
  w5500_CSOFF();
  SPI_SendByte( (addr & 0xFF00)>>8);	// 地址段1
  SPI_SendByte( (u8)(addr & 0x00FF));	// 地址段2
  SPI_SendByte( bsb );  // 控制段
  SPI_SendByte(data); 
  w5500_CSON();
}
uint8_t IINCHIP_READ(uint16_t addr, uint8_t bsb){//MCU向W5500读取单字节数据
  w5500_CSOFF();
  SPI_SendByte( (addr & 0xFF00)>>8);	// 地址段1
  SPI_SendByte( (u8)(addr & 0x00FF));	// 地址段2
  SPI_SendByte( bsb );  // 控制段
  uint8_t data = SPI_SendByte(0x00);
  w5500_CSON();
  return data;
}
uint16_t wiz_write_buf(uint16_t addr,uint8_t bsb,uint8_t* buf,uint16_t len){//向W5500发送写指令
  w5500_CSOFF();
  uint16_t index = 0;//发送位数计数
  SPI_SendByte((addr & 0xFF00)>>8 );//地址段1
  SPI_SendByte( (u8)(addr & 0x00FF));//地址段2数据
  SPI_SendByte( bsb );//控制段
  for(index = 0; index < len; index++){
    SPI_SendByte( buf[index]);
  }
  w5500_CSON();
  return len;
}
uint16_t wiz_read_buf(uint16_t addr,uint8_t bsb, uint8_t* buf,uint16_t len){//向W5500发送读指令
  w5500_CSOFF();
  uint16_t index = 0;//发送位数计数
  SPI_SendByte((addr & 0xFF00)>>8 );//取地址
  SPI_SendByte( (u8)(addr & 0x00FF) );//地址段2
  SPI_SendByte( bsb );//控制段数据
  for(index = 0; index < len; index++){
    buf[index] = SPI_SendByte(0x00);
  }
  w5500_CSON();
  return len;
}
/**
@brief  This function set the transmit & receive buffer size as per the channels is used
Note for TMSR and RMSR bits are as follows\n
bit 1-0 : memory size of channel #0 \n
bit 3-2 : memory size of channel #1 \n
bit 5-4 : memory size of channel #2 \n
bit 7-6 : memory size of channel #3 \n
bit 9-8 : memory size of channel #4 \n
bit 11-10 : memory size of channel #5 \n
bit 12-12 : memory size of channel #6 \n
bit 15-14 : memory size of channel #7 \n
W5500的Tx, Rx的最大寄存器宽度是16K Bytes,\n
In the range of 16KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and receiption from non-allocated channel may cause some problems.\n
If the 16KBytes memory is already  assigned to centain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/
void sysinit(uint8_t * tx_size, uint8_t * rx_size){ // setting tx/rx buf size
  uint16_t i;
  uint16_t ssum,rsum;
  ssum = 0;
  rsum = 0;
  
  for( i = 0; i < MAX_SOCK_NUM; i++){// Set the size, masking and base address of Tx & Rx memory by each channel
  	SSIZE[i] = (uint16_t)(0);
	RSIZE[i] = (uint16_t)(0);
	if (ssum <= 16384){
	       switch( tx_size[i] )
	    {
	    case 1:
	      SSIZE[i] = (uint16_t)(1024);
	      break;
	    case 2:
	      SSIZE[i] = (uint16_t)(2048);
	      break;
	    case 4:
	      SSIZE[i] = (uint16_t)(4096);
	      break;
	    case 8:
	      SSIZE[i] = (uint16_t)(8192);
	      break;
	    case 16:
	      SSIZE[i] = (uint16_t)(16384);
	    break;
	    default :
	      RSIZE[i] = (uint16_t)(2048);
	      break;
	    }
	  }

	 if (rsum <= 16384){
	       switch( rx_size[i] )
	    {
	    case 1:
	      RSIZE[i] = (uint16_t)(1024);
	      break;
	    case 2:
	      RSIZE[i] = (uint16_t)(2048);
	      break;
	    case 4:
	      RSIZE[i] = (uint16_t)(4096);
	      break;
	    case 8:
	      RSIZE[i] = (uint16_t)(8192);
	      break;
	    case 16:
	      RSIZE[i] = (uint16_t)(16384);
	      break;
	    default :
	      RSIZE[i] = (uint16_t)(2048);
	      break;
	    }
	  }
	  ssum += SSIZE[i];
	  rsum += RSIZE[i];
	  IINCHIP_WRITE( Sn_TXMEM_SIZE , BSB_Socket_Write(i) , tx_size[i]);
          IINCHIP_WRITE( Sn_RXMEM_SIZE, BSB_Socket_Write(i) , rx_size[i]);
  }
  


}



uint8_t getISR(uint8_t s){
  return I_STATUS[s];
}
void putISR(uint8_t s, uint8_t val){
   I_STATUS[s] = val;
}
uint16_t getIINCHIP_RxMAX(uint8_t s){
  return RSIZE[s];
}
uint16_t getIINCHIP_TxMAX(uint8_t s){
  return SSIZE[s];
}
void setMR(uint8_t val){
  IINCHIP_WRITE(MR, BSB_Regular_Write ,val);
}
/**
@brief  This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission
will be there as per RTR (Retry Time-value Register)setting
*/
void setRTR(uint16_t timeout){ // set retry duration for data transmission, connection, closing ...
  IINCHIP_WRITE(RTR0, BSB_Regular_Write ,(uint8_t)((timeout & 0xff00) >> 8));
  IINCHIP_WRITE(RTR1, BSB_Regular_Write ,(uint8_t)(timeout & 0x00ff));
}
/**
@brief  This function set the number of Retransmission.

If there is no response from the peer or delay in response then recorded time
as per RTR & RCR register seeting then time out will occur.
*/
void setRCR(uint8_t retry){ // set retry count (above the value, assert timeout interrupt)
  IINCHIP_WRITE(WIZ_RCR, BSB_Regular_Write ,retry);
}
/**
@brief  This function set the interrupt mask Enable/Disable appropriate Interrupt. ('1' : interrupt enable)

If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
set in IR register.
*/
void clearIR(uint8_t mask){// clear interrupt
  IINCHIP_WRITE(IR, BSB_Regular_Write , ~mask | getIR() ); // must be setted 0x10.
}
/**
@brief  This function gets Interrupt register in common register.
 */
uint8_t getIR( void ){
  return IINCHIP_READ(IR, BSB_Regular_Read);
}



void setGAR(uint8_t * addr){ // set gateway address
  wiz_write_buf(GAR0, BSB_Regular_Write , addr, 4);
}
void setSUBR(uint8_t * addr){// set subnet mask address
  wiz_write_buf(SUBR0, BSB_Regular_Write , addr, 4);
}
void setSHAR(uint8_t * addr){ // set local MAC address
  wiz_write_buf(SHAR0, BSB_Regular_Write , addr, 6);  
}
void setSIPR(uint8_t * addr){ // set local IP address
   wiz_write_buf(SIPR0, BSB_Regular_Write , addr, 4);  
}
void getGAR(uint8_t * addr){
  wiz_read_buf(GAR0, BSB_Regular_Read , addr, 4);
}
void getSUBR(uint8_t * addr){
  wiz_read_buf(SUBR0, BSB_Regular_Read , addr, 4);
}
void getSHAR(uint8_t * addr){
  wiz_read_buf(SHAR0, BSB_Regular_Read , addr, 6);
}
void getSIPR(uint8_t * addr){
   wiz_read_buf(SIPR0, BSB_Regular_Read , addr, 4);
}
/**
@brief  This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(SOCKET s, uint16_t Sn_MSSR){// set maximum segment size
  IINCHIP_WRITE( Sn_MSSR0, BSB_Socket_Write(s) , (uint8_t)((Sn_MSSR & 0xff00) >> 8));
  IINCHIP_WRITE( Sn_MSSR1, BSB_Socket_Write(s) , (uint8_t)(Sn_MSSR & 0x00ff));
}
void setSn_TTL(SOCKET s, uint8_t ttl){
  IINCHIP_WRITE( Sn_TTL, BSB_Socket_Write(s)  , ttl);
}
/**
@brief  get socket interrupt status

These below functions are used to read the Interrupt & Soket Status register
*/
uint8_t getSn_IR(SOCKET s){ // get socket interrupt status
  return IINCHIP_READ(Sn_IR, BSB_Socket_Read(s) );
}
/**
@brief  get socket interrupt status

These below functions are used to read the Interrupt & Soket Status register
*/
uint8_t getSn_SR(SOCKET s){
  return IINCHIP_READ(Sn_IR, BSB_Socket_Read(s) );
}
/**
@brief  get socket TX free buf size

This gives free buffer size of transmit buffer. This is the data size that user can transmit.
User shuold check this value first and control the size of transmitting data

uint16_t getSn_TX_FSR(SOCKET s){// get socket TX free buf size
  uint16_t val=0,val1=0;
  do
  {
    val1 = IINCHIP_READ(Sn_TX_FSR0(s));
    val1 = (val1 << 8) + IINCHIP_READ(Sn_TX_FSR1(s));
      if (val1 != 0)
    {
        val = IINCHIP_READ(Sn_TX_FSR0(s));
        val = (val << 8) + IINCHIP_READ(Sn_TX_FSR1(s));
    }
  } while (val != val1);
   return val;
}
*/
/**
@brief   get socket RX recv buf size

This gives size of received data in receive buffer.

uint16_t getSn_RX_RSR(SOCKET s){ // get socket RX recv buf size
  uint16_t val=0,val1=0;
  do
  {
    val1 = IINCHIP_READ(Sn_RX_RSR0(s));
    val1 = (val1 << 8) + IINCHIP_READ(Sn_RX_RSR1(s));
    if(val1 != 0)
    {
        val = IINCHIP_READ(Sn_RX_RSR0(s));
        val = (val << 8) + IINCHIP_READ(Sn_RX_RSR1(s));
    }
  } while (val != val1);
   return val;
}
*/


/**
@brief   This function is being called by send() and sendto() function also.

This function read the Tx write pointer register and after copy the data in buffer update the Tx write pointer
register. User should read upper byte first and lower byte later to get proper value.

void send_data_processing(SOCKET s, uint8_t *wizdata, uint16_t len){
  uint16_t ptr =0;
  uint32_t addrbsb =0;
 
  ptr = IINCHIP_READ( Sn_TX_WR0(s) );
  ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ(Sn_TX_WR1(s));

  addrbsb = (uint32_t)(ptr<<8) + (s<<5) + 0x10;
  wiz_write_buf(addrbsb, wizdata, len);
  
  ptr += len;
  IINCHIP_WRITE( Sn_TX_WR0(s) ,(uint8_t)((ptr & 0xff00) >> 8));
  IINCHIP_WRITE( Sn_TX_WR1(s),(uint8_t)(ptr & 0x00ff));
}
*/
/**
@brief  This function is being called by recv() also.

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.

void recv_data_processing(SOCKET s, uint8_t *wizdata, uint16_t len){
  uint16_t ptr = 0;
  uint32_t addrbsb = 0;
  

  ptr = IINCHIP_READ( Sn_RX_RD0(s) );
  ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ( Sn_RX_RD1(s) );

  addrbsb = (uint32_t)(ptr<<8) + (s<<5) + 0x18;
  wiz_read_buf(addrbsb, wizdata, len);
  ptr += len;

  IINCHIP_WRITE( Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));
  IINCHIP_WRITE( Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));
}
*/

void setSn_IR(uint8_t s, uint8_t val)
{
    IINCHIP_WRITE(Sn_IR, BSB_Socket_Write(s) , val);
}
