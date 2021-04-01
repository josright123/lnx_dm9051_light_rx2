 /* driver : configurations */
 
 /* [Architecture Basic] */
 /* (1) [DTS mode: yes] / [DTS mode: no] */
 /* (2) [Module mode: yes] / [Module mode: no] */
 /* (3) [INTERRUPT mode: yes] / [INTERRUPT mode: no] */
 /* (5) [Performance 1024_buf: yes] / [Performance 1024_buf: no] */
 
 /* [configuration definition] */
//(1) [DTS mode] Option 				//#define DTS_CONF_YES
//(2) [Module] Option 						//#define DM_CONF_MODULE
//(3) [INTERRUPT mode] Option //#define DM_CONF_INTERRUPT
//(4) [Supp] Option 							//#define SUPP_CONF_DMA_SYNC_YES (enhance for spi_sync)
//(5) [Performance] Options		//#define DM_CONF_1024_BUF_CASE_YES
//(6) [Custom MTK] Option 			//#define MTK_CONF_YES
//(1-1) [SPI configuration]
//#define DM_CONF_MAX_SPEED_HZ/DM_CONF_SPI_BUS_NUMBER/DM_CONF_SPI_CHIP_SELECT
//(3-1) [INTERRUPT configuration]
//#define DM_CONF_INTERRUPT_IRQ	26/DM_CONF_INTERRUPT_LOW_ACTIVE
//(4-1) [Supp Options] 
//#define ENABLE_PRINTLOG_DEBUG (basic print log)
//(5-1) [Performance Options] 
//#define DM_CONF_ADVENCE_MEM_YES/DM_CONF_1024_MTU_YES
//(6-1) [Custm Option] 
//#define MTK_CONF_SPI_DMA_YES
//#define QCOM_CONF_BOARD_YES
//#define DM_EXTREME_CPU_MODE
//#define DM_LIGHT_RX
                                                                                              
 /* ----------------------------- */
 /* {Easy Engineer customization} */
 /* ----------------------------- */
 // Please refer to "conf_rev.h"
 
 /* 1.{DTS Mode} */ 
 /* 2.{Module} */
 /* 3.{INTERRUPT Mode} */ 
 /* 5.[Performance] */
 
 /* {Basic} */
 
//#define DTS_CONF_YES
#define DM_CONF_MODULE
//#define DM_CONF_INTERRUPT

#define DM_EXTREME_CPU_MODE
#define DM_LIGHT_RX
//#define DM_CONF_1024_BUF_CASE_YES
//#define QCOM_CONF_BOARD_YES
//#define MTK_CONF_XY6762TEMP
 
 /* {Internal} */
 
//#define INTERNAL_ONEBYTE_SPI_SYNC  //temp
#ifdef DM_CONF_INTERRUPT
//#define DM_CONF_THREAD_IRQ
#endif

 /* 4-x.{Supp}:: */
 
#define ON_RELEASE

 /* 4.[Supp] */
 /* 6.[Custom MTK] */
 /* 7.[Manual] */
 
//#define SUPP_CONF_DMA_SYNC_YES
//(#define MTK_CONF_YES)
//#define DTS_MANUAL_TRIGGER

 /* 1-1.{SPI configuration}:: */
 
#ifndef DTS_CONF_YES
 /* [SPI configuration] opt1 */
#define DM_CONF_MAX_SPEED_HZ	7800000 //31200000 //15600000
//#define DM_CONF_SPI_BUS_NUMBER	0
//#define DM_CONF_SPI_CHIP_SELECT	1
 /* [SPI configuration] */
// #define DM_CONF_MAX_SPEED_HZ	7800000 //[1950000(1.953MHz),3900000(3.9MHz),7800000(7.8MHz)] //{13000000} //15600000 //{20000000} //31200000
 #define DM_CONF_SPI_BUS_NUMBER	0
 #define DM_CONF_SPI_CHIP_SELECT	1
#endif

 /* 2-1.{Module configuration}:: */
 
 /* 3-1.{Interrupt configuration}:: */
 #ifdef DM_CONF_INTERRUPT
 #ifndef DTS_CONF_YES
 /* 3.1.{Interrupt settings}:: */
  #define DM_CONF_INTERRUPT_IRQ	26
  #define DM_CONF_INTERRUPT_LOW_ACTIVE
 #endif //
 #endif //DM_CONF_INTERRUPT
 
 /* 5-1.{Performance}:: */
//[Performance fine tune options...]
 #define DM_CONF_1024_MTU_YES // 1024 limitation usage
//#define DM_CONF_1024_MTU_YES // 1024 limitation usage
 #define DM_CONF_ADVENCE_MEM_YES //Test effective memory buffer usage
 
 /* 6-1.{Custom}:: */
//[Custom oriented function...]
//#define MTK_CONF_SPI_DMA_YES

//[For irq trigger selection...]
#ifndef DTS_MANUAL_TRIGGER
#define DTS_AUTO_TRIGGER
#endif

#define  MORE_DM9051_MUTEX  // [Test to more mutex protecting]
#define  MORE_DM9051_MUTEX_EXT // [Be essential].[Must 'MORE_DM9051_MUTEX' defined so can effect.]

#define DM_CONF_SPI_DBG_INT_NUM     10  //25 //150
#define DM9_DBG_INT_ONOFF_COUNT     5   //10 //60 //50 //8

//#ifdef MTK_CONF_XY6762TEMP
//#define DM_CONF_SPI_TEST_BLKIN_SPLIT //[Tested for with 'MTK_CONF_XY6762TEMP']
//#define DM_CONF_SPI_TEST_BLKLEN    256   //4 //256
//#endif

#define MORE_DM9051_INT_BACK_TO_STANDBY 
#define ASL_RSRV_RX_HDR
#define ASL_RSRV_RX_HDR_LEN 44

//#define SCAN_LEN      3328  
//#define SCAN_LEN_MAX  (3328*3)
#define SCAN_LEN_HALF (512*13)

#define FREE_NO_DOUBLE_MEM_MAX  // so, to extra more than 13 KB is possible

#ifdef FREE_NO_DOUBLE_MEM_MAX // to extra more than 13 KB is possible
#undef SCAN_LEN_HALF
//#define SCAN_LEN_HALF (1024*13)
//#define SCAN_LEN_HALF (1024*32)         
//#define SCAN_LEN_HALF (1024*66)
#define SCAN_LEN_HALF (1024*98)
//#define JABBER_PACKET_SUPPORT
#endif //FREE_NO_DOUBLE_MEM_MAX

#define NUM_SCANX 16 //6 //10  //[8 recommand] //10 //60 //Define range [1~64], Max 64

// [Debug option]
#define ENABLE_PRINTLOG_DEBUG
// [Debug Display Select]
#define DM_DBG_ENDING_PACKET
#define	DM_DBG_ENDBUF_LEN			(1522+18)
// [Debug Display Switch]
#define	DISP_SKB_UNI_ERR	0

//#define DM_RELOAD_EEPROM //work for 'ifconfig hw ether' for reload eeprom after write eeprom.
