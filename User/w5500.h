/**
  ******************************************************************************

  * @author  王田
  * @date    2023/02/15
  * @brief   与w5500相关的代码
  ******************************************************************************
  */ 



/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_gpio.h"


/* defines -------------------------------------------------------------------*/
#ifndef	W5500_H
#define	W5500_H

/*
********************************以下为自己添加的内容***************************
*/
//端口引脚定义
#define W5500_INT_PORT GPIOB
#define W5500_INT_PIN GPIO_PIN_1
#define W5500_RST_PORT GPIOB
#define W5500_RST_PIN GPIO_PIN_2
#define W5500_CSN_PORT GPIOE
#define W5500_CSN_PIN GPIO_PIN_5

#define BSB_Regular_Write 0x04 //通用寄存器的写模式
#define BSB_Regular_Read 0x00 // 通用寄存器的读模式
#define BSB_Socket_Write(ch) (0x0A+(ch<<5))            // Socket(ch) 寄存器写模式
#define BSB_Socket_Read(ch) (0x06+(ch<<5))            // Socket(ch) 寄存器读模式
#define BSB_Socket_Send_Write(ch) (0x14+(ch<<5))            //Socket(ch) 发送缓存写模式
#define BSB_Socket_Send_Read(ch) (0x10+(ch<<5))           // Socket(ch) 发送缓存读模式
#define BSB_Socket_Rcv_Write(ch) (0x1C+(ch<<5))              // Socket(ch) 接收缓存写模式
#define BSB_Socket_Rcv_Read(ch) (0x18+(ch<<5))             // Socket(ch) 接收缓存读模式

void w5500_GPIOInit();//与w5500相关的引脚初始化
void w5500_CSON();
void w5500_CSOFF();



/*
********************************以下为官网例程的内容****************************
*/

#define MR                          (0x0000)

/**
 @brief Gateway IP Register address
 */
#define GAR0                        (0x0001)
#define GAR1                        (0x0002)
#define GAR2                        (0x0003)
#define GAR3                        (0x0004)
/**
 @brief Subnet mask Register address
 */
#define SUBR0                       (0x0005)
#define SUBR1                       (0x0006)
#define SUBR2                       (0x0007)
#define SUBR3                       (0x0008)

/**
 @brief Source MAC Register address
 */
#define SHAR0                       (0x0009)
#define SHAR1                       (0x000A)
#define SHAR2                       (0x000B)
#define SHAR3                       (0x000C)
#define SHAR4                       (0x000D)
#define SHAR5                       (0x000E)
/**
 @brief Source IP Register address
 */
#define SIPR0                       (0x000F)
#define SIPR1                       (0x0010)
#define SIPR2                       (0x0011)
#define SIPR3                       (0x0012)
/**
 @brief set Interrupt low level timer register address
 */
#define INTLEVEL0                   (0x0013)
#define INTLEVEL1                   (0x0014)
/**
 @brief Interrupt Register
 */
#define IR                          (0x0015)
/**
 @brief Interrupt mask register
 */
#define IMR                         (0x0016)
/**
 @brief Socket Interrupt Register
 */
#define SIR                         (0x0017) 
/**
 @brief Socket Interrupt Mask Register
 */
#define SIMR                        (0x0018)
/**
 @brief Timeout register address( 1 is 100us )
 */
#define RTR0                        (0x0019)
#define RTR1                        (0x001A)
/**
 @brief Retry count reigster
 */
#define WIZ_RCR                         (0x001B)
/**
 @briefPPP LCP Request Timer register  in PPPoE mode
 */
#define PTIMER                      (0x001C)
/**
 @brief PPP LCP Magic number register  in PPPoE mode
 */
#define PMAGIC                      (0x001D)
/**
 @brief PPP Destination MAC Register address
 */
#define PDHAR0                      (0x001E)
#define PDHAR1                      (0x001F)
#define PDHAR2                      (0x0020)
#define PDHAR3                      (0x0021)
#define PDHAR4                      (0x0022)
#define PDHAR5                      (0x0023)
/**
 @brief PPP Session Identification Register
 */
#define PSID0                       (0x0024)
#define PSID1                       (0x0025)
/**
 @brief PPP Maximum Segment Size(MSS) register
 */
#define PMR0                        (0x0026)
#define PMR1                        (0x0027)
/**
 @brief Unreachable IP register address in UDP mode
 */
#define UIPR0                       (0x0028)
#define UIPR1                       (0x0029)
#define UIPR2                       (0x002A)
#define UIPR3                       (0x002B)
/**
 @brief Unreachable Port register address in UDP mode
 */
#define UPORT0                      (0x002C)
#define UPORT1                      (0x002D)
/**
 @brief PHY Configuration Register
 */
#define PHYCFGR                      (0x002E)
/**
 @brief chip version register address
 */
#define VERSIONR                    (0x0039)   

////////////////////////////////////////////////////////////////////////TODO
/**
 @brief socket Mode register
 */
#define Sn_MR                     (0x0000)

/**
 @brief channel Sn_CR register
 */
#define Sn_CR                       (0x0001)
/**
 @brief channel interrupt register
 */
#define Sn_IR                       (0x0002)
/**
 @brief channel status register
 */
#define Sn_SR                       (0x0003)
/**
 @brief source port register
 */
#define Sn_PORT0                    (0x0004)
#define Sn_PORT1                    (0x0005)
/**
 @brief Peer MAC register address
 */
#define Sn_DHAR0                    (0x0006)
#define Sn_DHAR1                    (0x0007)
#define Sn_DHAR2                 (0x0008)
#define Sn_DHAR3                   (0x0009)
#define Sn_DHAR4                    (0x000A)
#define Sn_DHAR5                    (0x000B)
/**
 @brief Peer IP register address
 */
#define Sn_DIPR0                    (0x000C)
#define Sn_DIPR1                    (0x000D)
#define Sn_DIPR2                    (0x000E)
#define Sn_DIPR3                    (0x000F)
/**
 @brief Peer port register address
 */
#define Sn_DPORT0                   (0x0010)
#define Sn_DPORT1                   (0x0011)
/**
 @brief Maximum Segment Size(Sn_MSSR0) register address
 */
#define Sn_MSSR0                    (0x0012)
#define Sn_MSSR1                    (0x0013)
/** 
 @brief IP Type of Service(TOS) Register 
 */
#define Sn_TOS                      (0x0015)
/**
 @brief IP Time to live(TTL) Register 
 */
#define Sn_TTL                      (0x0016)
/**
 @brief Receive memory size reigster
 */
#define Sn_RXMEM_SIZE               (0x001E)
/**
 @brief Transmit memory size reigster
 */
#define Sn_TXMEM_SIZE               (0x001F)
/**
 @brief Transmit free memory size register
 */
#define Sn_TX_FSR0                  (0x0020)
#define Sn_TX_FSR1                  (0x0021)
/**
 @brief Transmit memory read pointer register address
 */
#define Sn_TX_RD0                   (0x0022)
#define Sn_TX_RD1                  (0x0023)
/**
 @brief Transmit memory write pointer register address
 */
#define Sn_TX_WR0                   (0x0024)
#define Sn_TX_WR1                   (0x0025)
/**
 @brief Received data size register
 */
#define Sn_RX_RSR0                  (0x0026)
#define Sn_RX_RSR1                  (0x0027)
/**
 @brief Read point of Receive memory
 */
#define Sn_RX_RD0                   (0x0028)
#define Sn_RX_RD1                   (0x0029)
/**
 @brief Write point of Receive memory
 */
#define Sn_RX_WR0                   (0x002A)
#define Sn_RX_WR1                   (0x002B)
/**
 @brief socket interrupt mask register
 */
#define Sn_IMR                      (0x002C)
/**
 @brief frag field value in IP header register
 */
#define Sn_FRAG                     (0x002D)
/**
 @brief Keep Timer register
 */
#define Sn_KPALVTR                  (0x002F)

/* MODE register values */
#define MR_RST                       0x80 /**< reset */
#define MR_WOL                       0x20 /**< Wake on Lan */
#define MR_PB                        0x10 /**< ping block */
#define MR_PPPOE                     0x08 /**< enable pppoe */
#define MR_UDP_FARP                  0x02 /**< enbale FORCE ARP */


/* IR register values */
#define IR_CONFLICT                  0x80 /**< check ip confict */
#define IR_UNREACH                   0x40 /**< get the destination unreachable message in UDP sending */
#define IR_PPPoE                     0x20 /**< get the PPPoE close message */
#define IR_MAGIC                     0x10 /**< get the magic packet interrupt */

/* Sn_MR values */
#define Sn_MR_CLOSE                  0x00     /**< unused socket */
#define Sn_MR_TCP                    0x01     /**< TCP */
#define Sn_MR_UDP                    0x02     /**< UDP */
#define Sn_MR_IPRAW                  0x03      /**< IP LAYER RAW SOCK */
#define Sn_MR_MACRAW                 0x04      /**< MAC LAYER RAW SOCK */
#define Sn_MR_PPPOE                  0x05     /**< PPPoE */
#define Sn_MR_UCASTB                 0x10     /**< Unicast Block in UDP Multicating*/
#define Sn_MR_ND                     0x20     /**< No Delayed Ack(TCP) flag */
#define Sn_MR_MC                     0x20     /**< Multicast IGMP (UDP) flag */
#define Sn_MR_BCASTB                 0x40     /**< Broadcast blcok in UDP Multicating */
#define Sn_MR_MULTI                  0x80     /**< support UDP Multicating */

 /* Sn_MR values on MACRAW MODE */
#define Sn_MR_MIP6N                  0x10     /**< IPv6 packet Block */
#define Sn_MR_MMB                    0x20     /**< IPv4 Multicasting Block */
//#define Sn_MR_BCASTB                 0x40     /**< Broadcast blcok */
#define Sn_MR_MFEN                   0x80     /**< support MAC filter enable */


/* Sn_CR values */
#define Sn_CR_OPEN                   0x01     /**< initialize or open socket */
#define Sn_CR_LISTEN                 0x02     /**< wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT                0x04     /**< send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON                 0x08     /**< send closing reqeuset in tcp mode */
#define Sn_CR_CLOSE                  0x10     /**< close socket */
#define Sn_CR_SEND                   0x20     /**< update txbuf pointer, send data */
#define Sn_CR_SEND_MAC               0x21     /**< send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP              0x22     /**<  send keep alive message */
#define Sn_CR_RECV                   0x40     /**< update rxbuf pointer, recv data */

#ifdef __DEF_IINCHIP_PPP__
   #define Sn_CR_PCON                0x23      
   #define Sn_CR_PDISCON             0x24      
   #define Sn_CR_PCR                 0x25      
   #define Sn_CR_PCN                 0x26     
   #define Sn_CR_PCJ                 0x27     
#endif

/* Sn_IR values */
#ifdef __DEF_IINCHIP_PPP__
   #define Sn_IR_PRECV               0x80     
   #define Sn_IR_PFAIL               0x40     
   #define Sn_IR_PNEXT               0x20     
#endif

#define Sn_IR_SEND_OK                0x10     /**< complete sending */
#define Sn_IR_TIMEOUT                0x08     /**< assert timeout */
#define Sn_IR_RECV                   0x04     /**< receiving data */
#define Sn_IR_DISCON                 0x02     /**< closed socket */
#define Sn_IR_CON                    0x01     /**< established connection */

/* Sn_SR values */
#define SOCK_CLOSED                  0x00     /**< closed */
#define SOCK_INIT                    0x13     /**< init state */
#define SOCK_LISTEN                  0x14     /**< listen state */
#define SOCK_SYNSENT                 0x15     /**< connection state */
#define SOCK_SYNRECV                 0x16     /**< connection state */
#define SOCK_ESTABLISHED             0x17     /**< success to connect */
#define SOCK_FIN_WAIT                0x18     /**< closing state */
#define SOCK_CLOSING                 0x1A     /**< closing state */
#define SOCK_TIME_WAIT               0x1B     /**< closing state */
#define SOCK_CLOSE_WAIT              0x1C     /**< closing state */
#define SOCK_LAST_ACK                0x1D     /**< closing state */
#define SOCK_UDP                     0x22     /**< udp socket */
#define SOCK_IPRAW                   0x32     /**< ip raw mode socket */
#define SOCK_MACRAW                  0x42     /**< mac raw mode socket */
#define SOCK_PPPOE                   0x5F     /**< pppoe socket */

/* IP PROTOCOL */
#define IPPROTO_IP                   0        /**< Dummy for IP */
#define IPPROTO_ICMP                 1        /**< Control message protocol */
#define IPPROTO_IGMP                 2        /**< Internet group management protocol */
#define IPPROTO_GGP                  3        /**< Gateway^2 (deprecated) */
#define IPPROTO_TCP                  6        /**< TCP */
#define IPPROTO_PUP                  12       /**< PUP */
#define IPPROTO_UDP                  17       /**< UDP */
#define IPPROTO_IDP                  22       /**< XNS idp */
#define IPPROTO_ND                   77       /**< UNOFFICIAL net disk protocol */
#define IPPROTO_RAW                  255      /**< Raw IP packet */

#define	MAX_SOCK_NUM	8
typedef  uint8_t SOCKET;

void IINCHIP_WRITE( uint16_t addr, uint8_t bsb,  uint8_t data);
uint8_t IINCHIP_READ(uint16_t addr, uint8_t bsb);
uint16_t wiz_write_buf(uint16_t addr,uint8_t bsb,uint8_t* buf,uint16_t len);
uint16_t wiz_read_buf(uint16_t addr,uint8_t bsb, uint8_t* buf,uint16_t len);
void sysinit(uint8_t * tx_size, uint8_t * rx_size); // setting tx/rx buf size
uint8_t getISR(uint8_t s);
void putISR(uint8_t s, uint8_t val);
uint16_t getIINCHIP_RxMAX(uint8_t s);
uint16_t getIINCHIP_TxMAX(uint8_t s);
void setMR(uint8_t val);
void setRTR(uint16_t timeout); // set retry duration for data transmission, connection, closing ...
void setRCR(uint8_t retry); // set retry count (above the value, assert timeout interrupt)
void clearIR(uint8_t mask); // clear interrupt
uint8_t getIR( void );
void setSn_MSS(SOCKET s, uint16_t Sn_MSSR); // set maximum segment size
uint8_t getSn_IR(SOCKET s); // get socket interrupt status
uint8_t getSn_SR(SOCKET s); // get socket status
//uint16_t getSn_TX_FSR(SOCKET s); // get socket TX free buf size
//uint16_t getSn_RX_RSR(SOCKET s); // get socket RX recv buf size
uint8_t getSn_SR(SOCKET s);
void setSn_TTL(SOCKET s, uint8_t ttl);
//void send_data_processing(SOCKET s, uint8_t *wizdata, uint16_t len);
//void recv_data_processing(SOCKET s, uint8_t *wizdata, uint16_t len);

void setGAR(uint8_t * addr); // set gateway address
void setSUBR(uint8_t * addr); // set subnet mask address
void setSHAR(uint8_t * addr); // set local MAC address
void setSIPR(uint8_t * addr); // set local IP address
void getGAR(uint8_t * addr);
void getSUBR(uint8_t * addr);
void getSHAR(uint8_t * addr);
void getSIPR(uint8_t * addr);
void setSn_IR(uint8_t s, uint8_t val);




#endif