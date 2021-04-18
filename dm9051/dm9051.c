/*
 * drivers/net/dm9051.c
 * Copyright 2017 Davicom Semiconductor,Inc.
 *
 * 	This program is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU General Public License
 * 	as published by the Free Software Foundation; either version 2
 * 	of the License, or (at your option) any later version.
 *
 * 	This program is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 * (C) Copyright 2017-2021 DAVICOM Semiconductor,Inc. All Rights Reserved.
 *  V2.0 - 20190829, Support DTS usage, Support Interrupt.
 *  V2.1 - 20190910, Support Non-DTS usage and Interrupt. 
 *  (Still not tesed for Polling. MTK cpu support is to be added.) 
 *  V2.2 - 20190912, Add spi_setup, add POLLING support
 *  (Test ... save file test ...and test ...)
 *  (Test ... save file test ...and test2 ...)
 *  V2.2xc - 20190924, Support ASR 1024 buf limitation.
 *         - With the tested log which is result-good with few RXBErr and fewer macErr.  
 *  V2.2yc - 20190929, Support ASR 0 buf limitation.
 *         - Verify with Module Mode, NON-DTS, INT OK.
 *         - Verify with Module Mode, NON-DTS, Polling OK.
 *         - Verify with Module Mode, DTS, INT OK.
 *         - Verify with Module Mode, DTS, Polling OK.
 *         - Verify DM_CONF_MAX_SPEED_HZ	individually as 15600000 and 31200000 OK.
 *  V2.2zc - 20191015, Test.Good
 *  V2.2zcd.beta- 20191023, Add Supp Conf SPI dma yes
 *  V2.2zcd - 20191024, test Supp Conf SPI dma yes, OK
 *  V2.2zcd_R1 - 20191105, make 5 major macros in 'conf_ver.h'
 *  V2.2zcd_R2b - 20191108, put spi fer/msg in 'board_info.h'
 *  V2.2zcd_R2b2_savecpu2 - 20191122, save cpu in polling mode
 *  V2.2zcd_R2b2_savecpu3i - 20191126, Correcting interrupt mode ('int_get_attribute' & 'int_get_begin')
 *  V2.2zcd_R2b2_savecpu3i2 - 20191126, Str[]
 *  V2.2zcd_R2b2_savecpu5i - 20191128, Interrupt & sched
 *  V2.2zcd_R2b2_savecpu5i2p_xTsklet_thread - 20191211, Tasklet Not Suitable, But request_threaded_irq()
 *  V2.2zcd_R2b2_savecpu5i2p_xTsklet1 - 20191213, ASL_RSRV_RX_HDR_LEN                                                              
 *  V2.2zcd_R2b2_savecpu5i2p_xTsklet1p - 20191215 code-format
 *  V2.2zcd_R2b2_savecpu5i2p_xTsklet2p - 20191218 RXB explore
 *  V2.2zcd_R2b2_savecpu5i2p_xTsklet3p - 20191218 MTK MT6762       
 *  V2.2zcd_R2b2_savecpu5i3p_xTsklet3p - 20191218 RX CHECK & mtu   
 *  V2.2zcd_R2b2_savecpu5i3p_xTsklet5p - 20191220 Quick process 1516_issue, Release mode (ON_RELEASE) support,  
 *                                       Create "conf_rev.h" which is for customization purpose~
 *  V2.2zcd_tsklet_savecpu_5pt - 20191227 Make the SCAN_LEN (also SCAN_LEN_HALF) as 98 KB (conf_ver.h)
 *                                       To improve the RX procedure, Also well analysis everything in RX.
 *  V2.2zcd_tsklet_savecpu_5pt_Jabber - 20200121 Make Jabber support
 *  V2.2zcd_tsklet_savecpu_5pt_JabberP - 20200206 Add rx errors, rx frames statistics
 *  V2.2zcd_tsklet_savecpu_5pt_JabberP_PM - 20200331 PM is for suspend/resume function
 *  v2.2zcd_lnx_dm9051_dts_Ver2.2zcd_R2_b2_savecpu5i2p_Tasklet5p_JabberP_pm_NEW2.0_extreme - 20210204 Add 
 *                                       the DM_EXTREME_CPU_MODE definition for if poor performance
 *  v2.2zcd_light_rx - 20210222, new_tx.c for tx, new_rx.c for light rx
 *                     20210308, macro DM_LIGHT_RX, support
 *  v2.2_light_rx - 20210322, format on_line (https://codebeautify.org/c-formatter-beautifier) 
 *  v2.2_light_rx - 20210323, clean up and only valid lines stay
 *  v2.3_light_rx2 - 20210401, Make "new_spinb.hc" with 1nfo9 and trans_dm9 individually 
 *								for easily maintain spi_sync()
 *                 - 20210401, Make this branch only eliminate 'DM_CONF_TASKLET'
 *                   		#ifndef DM_CONF_INTERRUPT
 *                   		//#define DM_CONF_TASKLET
 *                   		#endif
 */

#include <linux/module.h>

#include <linux/kernel.h>

#include <linux/netdevice.h>

#include <linux/etherdevice.h>

#include <linux/interrupt.h>

#include <linux/skbuff.h>

#include <linux/spinlock.h>

#include <linux/cache.h>

#include <linux/crc32.h>

#include <linux/mii.h>

#include <linux/ethtool.h>

#include <linux/delay.h>

#include <linux/irq.h>

#include <linux/slab.h>

#include <linux/gpio.h>

#include <linux/of_gpio.h>

#include <asm/delay.h>

#include <asm/irq.h>

#include <asm/io.h>

#include <linux/spi/spi.h>

extern int spi_register_board_info(struct spi_board_info
  const * info, unsigned n); //had been. #define CONFIG_SPI, used in local static-make for dm9051.o
#include "dm9051.h"

#include "conf_ver.h"

#include "def_ktx.h"

#include "board_infos.h"

//[Driver information]
#define	DRV_VERSION_CODE	DMDRIVER_VERSION(2,3,0) //VER2.3.0= 0x20300
#define	DRV_PROJECT_NAME	"light_rx2"
#define	DRV_VERSION_DATE	"20210401"

char DRV_VERSION[50];
u8 DM9051_fifo_reset_flg = 0; //GLOBAL CONTROL FLAG.

void DRV_VERSION_CONSTRUCT(void)
{
#ifdef DM_CONF_INTERRUPT
	char *optype= "INT";
#else
	char *optype= "POLL";
#endif
	sprintf(DRV_VERSION, "DM9051.%s-Ver%d.%d.%s.%s", optype,
		(DRV_VERSION_CODE>>16 & 0xff),(DRV_VERSION_CODE>>8 & 0xff),
		DRV_PROJECT_NAME,
		DRV_VERSION_DATE);
}

//[#include "board_dev0.c"]
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
/* this no need register board information! */
#else
static struct spi_board_info dm9051_spi_board_devs[] __initdata = {
  [0] = {
    .modalias = "dm9051",
    .max_speed_hz = DRV_MAX_SPEED_HZ,
    .bus_num = DRV_SPI_BUS_NUMBER,
    .chip_select = DRV_SPI_CHIP_SELECT,
    .mode = SPI_MODE_0,
    #ifdef DM_CONF_POLLALL_INTFLAG
    .irq = DM_CONF_INTERRUPT_IRQ,
    #endif
  },
};
#endif

//[#include "ethtool_ops.c"]
static inline board_info_t * to_dm9051_board(struct net_device * dev) {
  return netdev_priv(dev);
}

static void dm9051_get_drvinfo(struct net_device * dev,
  struct ethtool_drvinfo * info) {
  strcpy(info -> driver, CARDNAME_9051);
  strcpy(info -> version, DRV_VERSION);
  strlcpy(info -> bus_info, dev_name(dev -> dev.parent), sizeof(info -> bus_info));
}

static void dm9000_set_msglevel(struct net_device * dev, u32 value) {
  board_info_t * dm = to_dm9051_board(dev);
  dm -> msg_enable = value;
}

static u32 dm9000_get_msglevel(struct net_device * dev) {
  board_info_t * dm = to_dm9051_board(dev);
  return dm -> msg_enable;
}

#if LNX_KERNEL_v58
static int dm9000_get_link_ksettings(struct net_device * dev,
  struct ethtool_link_ksettings * cmd) {
  struct board_info * dm = to_dm9051_board(dev);
  mii_ethtool_get_link_ksettings( & dm -> mii, cmd);
  return 0;
}
static int dm9000_set_link_ksettings(struct net_device * dev,
  const struct ethtool_link_ksettings * cmd) {
  struct board_info * dm = to_dm9051_board(dev);
  return mii_ethtool_set_link_ksettings( & dm -> mii, cmd);
}

#else
static int dm9000_get_settings(struct net_device * dev, struct ethtool_cmd * cmd) {
  board_info_t * dm = to_dm9051_board(dev);
  mii_ethtool_gset( & dm -> mii, cmd);
  return 0;
}

static int dm9000_set_settings(struct net_device * dev, struct ethtool_cmd * cmd) {
  board_info_t * dm = to_dm9051_board(dev);
  return mii_ethtool_sset( & dm -> mii, cmd);
}
#endif

static int dm9000_nway_reset(struct net_device * dev) {
  board_info_t * dm = to_dm9051_board(dev);
  return mii_nway_restart( & dm -> mii);
}

static u32 dm9000_get_link(struct net_device * dev) {
  board_info_t * dm = to_dm9051_board(dev);
  return (u32) dm -> linkBool;
}

#define DM_EEPROM_MAGIC (0x444D394B)

static int dm9000_get_eeprom_len(struct net_device * dev) {
  return 128;
}

static int dm9000_get_eeprom(struct net_device * dev,
  struct ethtool_eeprom * ee, u8 * data) {
  #if 1
  board_info_t * dm = to_dm9051_board(dev);
  int offset = ee -> offset;
  int len = ee -> len;
  int i;

  printk("[dm9051_get_eeprom]\n");

  // EEPROM access is aligned to two bytes 
  if ((len & 1) != 0 || (offset & 1) != 0)
    return -EINVAL;

  ee -> magic = DM_EEPROM_MAGIC;

  for (i = 0; i < len; i += 2)
    dm9051_read_eeprom(dm, (offset + i) / 2, data + i);
  #endif
  return 0;
}

static int dm9000_set_eeprom(struct net_device * dev,
  struct ethtool_eeprom * ee, u8 * data) {
  #if 1
  board_info_t * dm = to_dm9051_board(dev);
  int offset = ee -> offset;
  int len = ee -> len;
  int i;

  printk("[dm9051_set_eeprom]\n");

  // EEPROM access is aligned to two bytes 
  if ((len & 1) != 0 || (offset & 1) != 0)
    return -EINVAL;

  if (ee -> magic != DM_EEPROM_MAGIC)
    return -EINVAL;

  for (i = 0; i < len; i += 2)
    dm9051_write_eeprom(dm, (offset + i) / 2, data + i);
  #endif
  return 0;
}

//[#include "ethtool_ops1.c"]
static
const struct ethtool_ops dm9051_ethtool_ops = {
  .get_drvinfo = dm9051_get_drvinfo,
  #if LNX_KERNEL_v58
  .get_link_ksettings = dm9000_get_link_ksettings,
  .set_link_ksettings = dm9000_set_link_ksettings,
  #else
  .get_settings = dm9000_get_settings,
  .set_settings = dm9000_set_settings,
  #endif
  .get_msglevel = dm9000_get_msglevel,
  .set_msglevel = dm9000_set_msglevel,
  .nway_reset = dm9000_nway_reset,
  .get_link = dm9000_get_link,
  /*TBD	
  	.get_wol			= dm9000_get_wol,
  	.set_wol			= dm9000_set_wol,
  */
  .get_eeprom_len = dm9000_get_eeprom_len,
  .get_eeprom = dm9000_get_eeprom,
  .set_eeprom = dm9000_set_eeprom,
};

#if DM_RX_HEAVY_MODE //(for include impement files)
#define RXM_WrtPTR 0 // Write Read RX FIFO Buffer Writer Pointer
#define MD_ReadPTR 1 // Write Read RX FIFO Buffer Read Pointer
#include "unload.cc"
#include "unload1.cc"
#endif //DM_RX_HEAVY_MODE //(for include impement files)

#if !DM_RX_HEAVY_MODE //(for light_rx impement files)
#include "new_printnb.hc" 
#include "new_spinb.hc"
//("newload.c")
//("newload1.c")
// init
static void SCH_clear(char * headstr, board_info_t * db) //open
{
  //db->rx_count= //need in open only
  db -> Enter_hash =
	db -> sch_cause =
	db -> nSCH_INIT =
	db -> nSCH_LINK =
	db -> nSCH_XMIT =
	db -> nSCH_GoodRX =
	db -> nSCH_UniRX =
	db -> nSCH_INT = db -> nSCH_INT_B =
	db -> nSCH_INT_Glue =
	db -> nSCH_INT_NUm = db -> nSCH_INT_NUm_A =
	db -> nSCH_INFINI =
	db -> mac_process = 0;
  db -> nSCH_INT_Num_Disp = 100;
  db -> nSCH_XMIT_WAVE_PDL = NUM_SCH_XMIT_WAVE; //init.
}

void Operation_clear(board_info_t * db) // probe
{
  #if DM_RX_HEAVY_MODE
  db -> rwtrace1 =
    db -> RUNNING_rwregs[0] =
    db -> RUNNING_rwregs[1] = 0;
  #endif
  db -> bC.OvrFlw_counter =
    db -> bC.ERRO_counter =
    db -> bC.RXBErr_counter =
    db -> bC.LARGErr_counter =
    db -> bC.StatErr_counter =
    db -> bC.DO_FIFO_RST_counter = 0;
  db -> bC.DO_RP_Update_counter = 0;
  #if DM_RX_HEAVY_MODE
  db -> rx_rst_quan =
    db -> rx_tot_quan = 0;
  #endif
  db -> rx_scan_packets = 0;
  #ifdef DM_DBG_ENDING_PACKET
  db->endbuf_valid = 0;
  #endif
}

// init (AppSrc)
// board_info.c soure code
static void bcopen_rx_info_clear(struct rx_ctl_mach * pbc) { // fifo_reset ot rp_update
  pbc -> rxbyte_ref_accumu = REF_ACCMU_START;
  pbc -> rxbyte_ref_tx_counter = pbc -> rxbyte_has_tx_counter = pbc -> rxbyte_ref_tx_tmp = 0;
  pbc -> rxbyte_counter =
    pbc -> rxbyte_counter0 =

    pbc -> rxbyte_counter0_to_prt =
    #if 0
  pbc -> rx_brdcst_counter =
    pbc -> rx_multi_counter =
    #endif
  pbc -> rx_unicst_counter = 0;

  pbc -> isbyte = 0xff; // Special Pattern

  pbc -> nRRMAX =
    pbc -> sScanSizeMax = 0;
}

//#if 0
//#endif
#endif //!DM_RX_HEAVY_MODE //(for light_rx impement files)

#if !DM_RX_HEAVY_MODE //(for light_rx impement files)
//[APIs definitions] 
//[#include "new_spi_sync.c"]

#ifndef RPI_CONF_SPI_DMA_YES
#define dm9051_space_alloc std_space_request //#define dm9051_space_request std_space_request
#define dm9051_space_free std_space_free
#endif

#if DEF_SPIRW
#define dm9051_spirw_begin(d) dm9051_space_alloc(d)
#define dm9051_spirw_end(d) dm9051_space_free(d)
#else
#define dm9051_spirw_begin(d) // Never called undefine _DEF_SPIRW
#define dm9051_spirw_end(d) // Essentially called.
#endif

static int std_alloc(struct board_info * db) {
  #ifdef RPI_CONF_SPI_DMA_YES
  printk("[ *dm9051 DRV ] spi mode[= std] using 'enable_dma' is 0\n");
  printk("[ *dm9051 DRV ] spi mode[= dma] But using kmalloc, TxDatBuf[] or std_alloc TxDatBuf\n"); //ADD.JJ
  #else
  //.printk("[ *dm9051 DRV ] spi mode[= std] using kmalloc, TxDatBuf[] or std_alloc TxDatBuf\n"); //ADD.JJ
  #endif
  db -> spi_sypc_buf = kmalloc(SPI_SYNC_TRANSFER_BUF_LEN, GFP_ATOMIC);
  if (!db -> spi_sypc_buf)
    return -ENOMEM;
  return 0; // no-Err
}

static void std_free(struct board_info * db) {
  kfree(db -> spi_sypc_buf);
}

static int std_space_request(struct board_info * db) {
  callback_setup(0); // assign 0 to 'enable_dma'     
  /* Alloc non-DMA buffers */
  return std_alloc(db);
}
static void std_space_free(struct board_info * db) {
  /* Free non-DMA buffers */
  std_free(db);
}

// spi_user.c
void Fifo_Reset_Disp_RunningEqu_Ending(board_info_t * db) {
  /*char * s;
  if (db -> bC.rxbyte_counter0_to_prt >= 2) {
    if ((db -> _rwregs[0] != db -> _rwregs[1]) && (db -> bC.isbyte & ISR_PRS))
      s = "(---Accumulat times---)Rare-case";
    else if (db -> _rwregs[0] != db -> _rwregs[1])
      s = "(---Accumulat times---)Diff";
    else
      s = "(---Accumulat times---)Impossible";
    printk("dm9-IOR wrRd %04x/%04x (RO %d.%d%c) ISR %02x rxb= %02x %s (%2d ++)\n",
      db -> _rwregs[0], db -> _rwregs[1], db -> calc >> 8, db -> calc & 0xFF, '%', db -> bC.isbyte, db -> bC.dummy_pad, s,
      db -> bC.rxbyte_counter0_to_prt);
  }*/
}

static void
reset_rst(board_info_t * db) {
  iiow(db, DM9051_NCR, NCR_RST);
  mdelay(1);
}
static void
reset_bnd(board_info_t * db) {
  iiow(db, DM9051_MBNDRY, MBNDRY_BYTE);
  mdelay(1);
}

static void
dm9051_reset(board_info_t * db) {
  mdelay(2); // delay 2 ms any need before NCR_RST (20170510)
  reset_rst(db);
  reset_bnd(db);
  #if DM_RX_HEAVY_MODE
  db -> rwregs1 = 0x0c00;
  #endif
}

// ------------------------------------------------------------------------------
// state: 0 , fill '90 90 90 ...' e.g. dm9051_fi.fo_re.set(0, "fifo-clear0", db);
//		  1 , RST
//        2 , dump 'fifo-data...'
//		 11 , RST-Silent
// hstr:  String 'hstr' to print-out
//        NULL (no 'hstr' print)
// Tips: If (state==1 && hstr!=NULL)
//        Do increase the RST counter
// ------------------------------------------------------------------------------
static void dm9051_fifo_reset(u8 state, u8 * hstr, board_info_t * db) {
  u8 pk;
  if (state == 11) {
    if (hstr) {
      #if DM_RX_HEAVY_MODE
      db -> rx_rst_quan = 0;
      #endif
      ++db -> bC.DO_FIFO_RST_counter;
      Fifo_Reset_Disp_RunningEqu_Ending(db);
      #ifdef ON_RELEASE
      rel_printk2("dm9-%s Len %03d RST_c %d\n", hstr, db -> bC.RxLen, db -> bC.DO_FIFO_RST_counter);
      #else
      printk("dm9-%s Len %03d RST_c %d\n", hstr, db -> bC.RxLen, db -> bC.DO_FIFO_RST_counter); //printlog
      #endif
    }
    dm9051_reset(db);
    #if DEF_SPIRW
    iiow(db, DM9051_FCR, FCR_FLOW_ENABLE); /* Flow Control */
    iiow(db, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
    #ifdef DM_CONF_POLLALL_INTFLAG
    #if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE

    //#if defined DM_CONF_INTERRUPT_TEST_DTS_FALLING
    //	iiow(db, DM9051._INTCR, 0x01); //low active
    if (db -> irq_type & (IIRQ_TYPE_LEVEL_HIGH | IIRQ_TYPE_EDGE_RISING))
      iiow(db, DM9051_INTCR, 0x00); //high active(default)
    //iiow(db, DM9051._INTCR, 0x01);  //test
    //#elif defined DM_CONF_INTERRUPT_TEST_DTS_RISING
    //	iiow(db, DM9051._INTCR, 0x00); //high active(default)
    else
      iiow(db, DM9051_INTCR, 0x01); //low active
    //iiow(db, DM9051._INTCR, 0x00); //test
    //#endif

    #else
    #if(DRV_IRQF_TRIGGER == IRQF_TRIGGER_LOW)
    iiow(db, DM9051_INTCR, 0x01); //low active
    #else
    iiow(db, DM9051_INTCR, 0x00); //high active(default)
    #endif
    #endif
    //.iiow(db, DM9051_IMR, IMR_PAR | IMR_PTM | IMR_PRM); //...
    #else
    //.iiow(db, DM9051_IMR, IMR_PAR);
    #endif
    //..iiow(db, DM9051_IMR, IMR_PAR); //=int_reg_stop(db); 
    //iiow(db, DM9051_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);
    //iiow(db, DM9051_RCR, db->rcr_all);
    #endif
    bcopen_rx_info_clear( & db -> bC);
    DM9051_fifo_reset_flg = 1;
    return;
  }
  if (state == 1 || state == 2 || state == 3 || state == 5) {
    if (hstr) {
      #if DM_RX_HEAVY_MODE
      db -> rx_rst_quan = 0;
      #endif
      ++db -> bC.DO_FIFO_RST_counter;
      Fifo_Reset_Disp_RunningEqu_Ending(db);

      if (state == 1)
        #ifdef ON_RELEASE
      rel_printk2("dm9-%s Len %03d RST_c %d\n", hstr, db -> bC.RxLen, db -> bC.DO_FIFO_RST_counter);
      #else
      printk("dm9-%s Len %03d RST_c %d\n", hstr, db -> bC.RxLen, db -> bC.DO_FIFO_RST_counter); //"LenNotYeh", " %d", db->bC.DO_FIFO_RST_counter
      #endif
      else if (state == 3) {
        #ifdef ON_RELEASE
        rel_printk2("dm9-%s Len %03d RST_c %d (RXBErr %d)\n", hstr, db -> bC.RxLen,
          db -> bC.DO_FIFO_RST_counter, db -> bC.RXBErr_counter);
        #else
        if (db -> mac_process)
          printk("dm9-%s Len %03d RST_c %d (RXBErr %d)\n", hstr, db -> bC.RxLen,
            db -> bC.DO_FIFO_RST_counter, db -> bC.RXBErr_counter);
        else
          printlog("dm9-%s Len %03d RST_c %d (RXBErr %d)\n", hstr, db -> bC.RxLen,
            db -> bC.DO_FIFO_RST_counter, db -> bC.RXBErr_counter);
        #endif
      } else if (state == 2) // STATE 2
      {
        #ifdef ON_RELEASE
        rel_printk2("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db -> bC.RXBErr_counter);
        #else
        if (db -> mac_process)
          printk("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db -> bC.RXBErr_counter);
        else
          printlog("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db -> bC.RXBErr_counter); //"Len %03d ", db->bC.RxLen
        #endif
      } else // STATE 5
      {
        #ifdef ON_RELEASE
        rel_printk2("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db -> bC.RXBErr_counter);
        #else
        if (db -> mac_process)
          printk("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db -> bC.RXBErr_counter);
        else
          printlog("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db -> bC.RXBErr_counter);
        #endif
      }
    }
    dm9051_reset(db);
    #if DEF_SPIRW
    iiow(db, DM9051_FCR, FCR_FLOW_ENABLE); /* Flow Control */
    if (hstr)
      iiow(db, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
    else {
      pk = ior(db, DM9051_PPCR);
      iow(db, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
    }
    #ifdef DM_CONF_POLLALL_INTFLAG
    #if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
    //#if defined DM_CONF_INTERRUPT_TEST_DTS_FALLING
    //	iiow(db, DM9051._INTCR, 0x01); //low active
    if (db -> irq_type & (IIRQ_TYPE_LEVEL_HIGH | IIRQ_TYPE_EDGE_RISING))
      iiow(db, DM9051_INTCR, 0x00); //high active(default)
    //iiow(db, DM9051_INTCR, 0x01); //test
    //#elif defined DM_CONF_INTERRUPT_TEST_DTS_RISING
    //	iiow(db, DM9051._INTCR, 0x00); //high active(default)
    else
      iiow(db, DM9051_INTCR, 0x01); //low active
    //iiow(db, DM9051_INTCR, 0x00); //test
    //#endif	
    #else
    #if(DRV_IRQF_TRIGGER == IRQF_TRIGGER_LOW)
    iiow(db, DM9051_INTCR, 0x01); //low active
    #else
    iiow(db, DM9051_INTCR, 0x00); //high active(default)
    #endif
    #endif
    //.iiow(db, DM9051_IMR, IMR_PAR | IMR_PTM | IMR_PRM);
    #else
    //.iiow(db, DM9051_IMR, IMR_PAR);
    #endif
    //..iiow(db, DM9051_IMR, IMR_PAR); //=int_reg_stop(db); 
    //iiow(db, DM9051_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);
    //iiow(db, DM9051_RCR, db->rcr_all);
    #endif
    bcopen_rx_info_clear( & db -> bC);
    DM9051_fifo_reset_flg = 1;
    return;
  }
  //if (state==2){ 
  //while (1);
  //}
  return;
}

//  disp
static void dm9051_fifo_reset_statistic(board_info_t * db) {
  if (!(db -> bC.DO_FIFO_RST_counter % 10)) {
    rel_printk1("dm9-Mac_OvFlwEr.Rxb&LargEr RST_c %d\n", db -> bC.DO_FIFO_RST_counter);
    rel_printk1(" %d %d.%d %d\n",
      db -> bC.ERRO_counter, db -> bC.OvrFlw_counter, db -> bC.RXBErr_counter, db -> bC.LARGErr_counter);
    if (db -> bC.StatErr_counter)
      rel_printk1("dm9-RareFnd StatEr %d\n", db -> bC.StatErr_counter);
  }
}

#if 0
// when reset: return 1
int dm9051_fifo_ERRO(int ana_test, u8 rxbyte, board_info_t * db) {
  char hstr[72];
  if (db -> bC.rxbyte_counter == 5 || /*db->bC.rxbyte_counter0==(NUMRXBYTECOUNTER-1)*/ db -> bC.rxbyte_counter0 == NUMRXBYTECOUNTER) {
    db -> bC.RXBErr_counter++;
    if (ana_test < 3) {
      sprintf(hstr, "dmfifo_reset( 03 RxbErr ) rxb=%02X .%04x/%04x", rxbyte, db -> _rwregs[0], db -> _rwregs[1]);
      dm9051_fifo_reset(3, hstr, db);
      dm9051_fifo_reset_statistic(db);
      return 1;
    } else {
      if (db -> mac_process) {
        sprintf(hstr, "Or. Do_reset( 02 RxbErr, macError-clear ) rxb=%02X .%04x/%04x", rxbyte, db -> _rwregs[0], db -> _rwregs[1]); //printk("macError {Just treat as a normal-unicast.}\n"); //, Get recover-clear
        dm9051_fifo_reset(5, hstr, db);
      } else {
        sprintf(hstr, "Or. Do_reset( 02 RxbErr ) rxb=%02X .%04x/%04x", rxbyte, db -> _rwregs[0], db -> _rwregs[1]);
        dm9051_fifo_reset(2, hstr, db); //bcopen_rx_info_.clear(&db->bC); // as be done in 'dm9051._fifo_reset'
      }
      db -> mac_process = 0;
    }
  }
  return 0;
}
#endif

//[#include "new_tx.c"]
#if DM9051_CONF_TX
static void dm9051_tx_chk(struct net_device * dev, u8 * wrptr) {
  /*printk("dm9.tx_packets %lu ", dev -> stats.tx_packets);
  printk("tx(%02x %02x %02x %02x %02x %02x ", wrptr[0], wrptr[1], wrptr[2], wrptr[3], wrptr[4], wrptr[5]);
  printk("%02x %02x   %02x %02x %02x %02x ", wrptr[6], wrptr[7], wrptr[8], wrptr[9], wrptr[10], wrptr[11]);
  printk("%02x %02x\n", wrptr[12], wrptr[13]);*/
}

static u16 check_cntStop(board_info_t * db) {
  //u16 cs;
  //while (!spin_trylock( & db -> statelock_tx1_rx1)); //if(!)
  //cs = db -> bt.prob_cntStopped;
  //spin_unlock( & db -> statelock_tx1_rx1);
  //return cs;
  return (!skb_queue_empty( & db -> txq));
}

static void opening_wake_queue1(struct net_device * dev) //( u8 flag)
{
  //board_info_t * db = netdev_priv(dev);
  //while (!spin_trylock( & db -> statelock_tx1_rx1)); //if(!)
  //if (db -> bt.prob_cntStopped) {
  //  db -> bt.prob_cntStopped = 0;
  //  netif_wake_queue(dev);
  //}
  //spin_unlock( & db -> statelock_tx1_rx1);
  board_info_t * db = netdev_priv(dev);
  if (db -> bt.prob_cntStopped) {
    db -> bt.prob_cntStopped = 0;
    netif_wake_queue(dev);
  }
}

static void toend_stop_queue1(struct net_device * dev, u16 stop_cnt) {
  //board_info_t * db = netdev_priv(dev);
  //while (!spin_trylock( & db -> statelock_tx1_rx1)); //if(!)
  //switch (stop_cnt) {
  //case 1:
  //  db -> bt.prob_cntStopped++;
  //  break;
  //case NUM_QUEUE_TAIL:
  //default:
  //  db -> bt.prob_cntStopped = stop_cnt;
  //  break;
  //}
  //spin_unlock( & db -> statelock_tx1_rx1);
  //if (stop_cnt < NUM_QUEUE_TAIL)
  //  return;
  //if (stop_cnt == NUM_QUEUE_TAIL) {
  //  netif_stop_queue(dev);
  //  return;
  //}
  //.wrong path, but anyhow call stop for it
  //netif_stop_queue(dev);
  //printk("[.wrong path]: WARN, anyhow call stop for it .. ");
  //printk("(cntStop %d)\n", db -> bt.prob_cntStopped);
  //driver_dtxt_disp(db); // OPTIONAL CALLED
  //return;
  board_info_t * db = netdev_priv(dev);
  switch (stop_cnt) {
  case 1:
    db -> bt.prob_cntStopped++;
    break;
  case NUM_QUEUE_TAIL:
    db -> bt.prob_cntStopped = stop_cnt;
    break;
  }
  if (stop_cnt == NUM_QUEUE_TAIL)
    netif_stop_queue(dev);
}

static int
dm9051_continue_xmit_inRX(board_info_t * db) //dm9051_continue_poll_xmit
{
  struct net_device * dev = db -> ndev;
  int nTx = 0;

  db -> bt.local_cntTXREQ = 0;
  db -> bt.local_cntLOOP = 0;
  while (!skb_queue_empty( & db -> txq)) // JJ-20140225, When '!empty'
  {
    struct sk_buff * tx_skb;
    int nWait = 0;

    tx_skb = skb_dequeue( & db -> txq);
    if (tx_skb != NULL) {

      if (db -> nSCH_XMIT <= db -> nSCH_XMIT_WAVE_PDL) {
        char * p = (char * ) tx_skb -> data;
        if (!(p[0] & 1)) {
          if (db -> nSCH_XMIT == db -> nSCH_XMIT_WAVE_PDL) {
            if (db -> nSCH_XMIT_WAVE_PDL < NUM_TOTAL_ALL) {
              db -> nSCH_XMIT_WAVE_PDL += NUM_SCH_XMIT_WAVE;
              printk("dm9 %s ----- ALLOW display xmit counter to %d -----\n", "_XmitInRx", db -> nSCH_XMIT_WAVE_PDL);
            }
          }
        }
      }
      while ((ior(db, DM9051_TCR) & TCR_TXREQ) && (++nWait < 20))
		;
      if (nWait == 20)
        printk("[dm9] tx_step timeout B\n");

      if (db -> bt.local_cntTXREQ == 2) {
        db -> bt.local_cntTXREQ = 0;
      }

      nTx++;
	  
     dm9outblk(db, tx_skb -> data, tx_skb -> len);
//#ifdef QCOM_TX_DWORD_BOUNDARY //dm9051_outblk
//		dm9.lagcy_outblk_dwbnd(db, tx_skb -> data, tx_skb -> len);
//#else
//		dm9.lagcy_outblk(db, tx_skb -> data, tx_skb -> len)
//#endif	
	  
      iow(db, DM9051_TXPLL, tx_skb -> len);
      iow(db, DM9051_TXPLH, tx_skb -> len >> 8);
      #ifdef JABBER_PACKET_SUPPORT
      iow(db, DM9051_TCR, TCR_TXREQ | TCR_DIS_JABBER_TIMER);
      #else
      iow(db, DM9051_TCR, TCR_TXREQ);
      #endif
      dev -> stats.tx_bytes += tx_skb -> len;
      dev -> stats.tx_packets++;
      /* done tx */
      dm9051_tx_chk(dev, tx_skb -> data);
      dev_kfree_skb(tx_skb);
      db -> bt.local_cntTXREQ++;
      db -> bt.local_cntLOOP++;
      db -> bC.rxbyte_ref_tx_counter++;
    } //if
  } //while
  return nTx;
}
#endif

static int dm9051_tx_irx(board_info_t * db) {
  struct net_device * dev = db -> ndev;
  int nTx = 0;
  #if DM9051_CONF_TX

  if (check_cntStop(db))
  //if (db->bt.prob_cntStopped)  
  // This is more exactly right!!
  {
    #if LOOP_XMIT
    //mutex_lock(&db->addr_lock);
    nTx = dm9051_continue_xmit_inRX(db); //=dm9051_continue_poll_xmit
    opening_wake_queue1(dev);
    //mutex_unlock(&db->addr_lock);
    #endif //LOOP_XMIT
  }

  #endif //DM9051_CONF_TX
  return nTx;
}

//Ever supposed #include "new_rx.c", "one_rx_esp32_jj2.c"
#if DM_RX_HEAVY_MODE
#define dm9051_disp_hdr_s_new(b) DM9051_DISP_HDR_S(b)
#define dm9051_disp_hdr_e_new(b) DM9051_DISP_HDR_E(b)
#define rx_mutex_head(b) RX_MUTEX_HEAD(b)
#define rx_mutex_tail(b) RX_MUTEX_TAIL(b)
#else
#define dm9051_disp_hdr_s_new(b)
#define dm9051_disp_hdr_e_new(b)
#define rx_mutex_head(b)
#define rx_mutex_tail(b)
#endif

//rx.c
#if DM_RX_HEAVY_MODE
//static void dm9051_disp_hdr_s(board_info_t *db)
static void DM9051_DISP_HDR_S(board_info_t * db) {
  u16 calc = dm9051_rx_cap(db);
  db -> DERFER_rwregs[RXM_WrtPTR] = db -> rwregs[0];
  db -> DERFER_rwregs[MD_ReadPTR] = db -> rwregs[1];
  db -> DERFER_calc = calc;
}
//static void dm9051_disp_hdr_e(board_info_t *db, int rxlen)
static void DM9051_DISP_HDR_E(board_info_t * db) {
  u16 calc = dm9051_rx_cap(db);
  db -> DERFER_rwregs1[RXM_WrtPTR] = db -> rwregs[0];
  db -> DERFER_rwregs1[MD_ReadPTR] = db -> rwregs[1];
  db -> DERFER_calc1 = calc;
}

void RX_MUTEX_HEAD(board_info_t * db) {
  #ifdef DM_CONF_POLLALL_INTFLAG
  mutex_lock( & db -> addr_lock);
  //.iiow(db, DM9051._IMR, IMR._PAR); // Disable all interrupts 
  #elif DRV_POLL_1
  mutex_lock( & db -> addr_lock);
  #endif
}
void RX_MUTEX_TAIL(board_info_t * db) {
  #ifdef DM_CONF_POLLALL_INTFLAG
  //.iiow(db, DM9051._IMR, db->imr._all); // Re-enable interrupt mask 
  mutex_unlock( & db -> addr_lock);
  #elif DRV_POLL_1
  mutex_unlock( & db -> addr_lock);
  #endif
}
#endif

#if DM_RX_HEAVY_MODE == 0
static int dm9051_scanx(board_info_t * db, int nMax);
static void dm9051_dbg_scanx(board_info_t * db, int nRR);
static int dm9051_skbx(board_info_t * db, int nRR);
//static int dm9051_isr_ext2(board_info_t * db) {
//  int nR = dm9051_scanx(db, NUM_SCANX);
//  dm9051_dbg_scanx(db, nR);
//  return dm9051_skbx(db, nR);
//}

int rx_tx_isr0(board_info_t * db) {
  int nRTmp;
  int nTX, nRx;
  int nAllSum = 0;

  do {
    nTX = dm9051_tx_irx(db);
	
    //nRx = dm9051_isr_ext2(db); //dm9051_continue_poll_rx(db);
	//=
	  nRTmp = dm9051_scanx(db, NUM_SCANX);
	  dm9051_dbg_scanx(db, nRTmp);
	  nRx = dm9051_skbx(db, nRTmp);
  
    nAllSum += nTX;
    nAllSum += nRx;
  } while (nTX || nRx);

  return nAllSum;
}
#endif

#if DM_RX_HEAVY_MODE == 0
// [ xxxxxxxxxxxx ]
// [ scan and skb ]
// [ xxxxxxxxxxxx ]
#define DM_TYPE_ARPH 0x08
#define DM_TYPE_ARPL 0x06
#define DM_TYPE_IPH 0x08
#define DM_TYPE_IPL 0x00
static bool SB_skbing_packet_chk_data(board_info_t * db, u8 * phd, u8 * rdptr) {
  struct net_device * dev = db -> ndev;
  u8 flg_disp = 0;

  if ((phd[1] & 0x40) && (!(rdptr[0] & 1))) {
    flg_disp = 1;
    dev -> stats.rx_length_errors = 3;
    dev -> stats.rx_crc_errors = 6;
    dev -> stats.rx_fifo_errors = 9;
    dev -> stats.rx_missed_errors = 12;
    dev -> stats.rx_over_errors++;
    printk("\n[@dm9.multiErr (rxhdr %02x %02x %02x %02x)] mac %02x %02x %02x, %lu\n", phd[0], phd[1], phd[2], phd[3], rdptr[0], rdptr[1], rdptr[2],
      dev -> stats.rx_over_errors); //dev->stats
  } else if (rdptr[0] != dev -> dev_addr[0] || rdptr[1] != dev -> dev_addr[1] || rdptr[2] != dev -> dev_addr[2]) {
    if ((rdptr[4] == DM_TYPE_ARPH && rdptr[5] == DM_TYPE_ARPL) && (rdptr[12] != DM_TYPE_ARPH || rdptr[13] != DM_TYPE_ARPL)) // special in data-skip
    ; // error=fail //;;[current has rdptr[12]/rdptr[13]]
    if ((rdptr[4] == DM_TYPE_IPH && rdptr[5] == DM_TYPE_IPL) && (rdptr[12] != DM_TYPE_IPH || rdptr[13] != DM_TYPE_IPL)) // special in data-skip
    ; // error=fail //;;[current has rdptr[12]/rdptr[13]]
    else if (rdptr[0] & 1) //'skb->data[0]'
      return true; // ok

    flg_disp = 1;
    dev -> stats.rx_frame_errors++;
    //PRINTK("\n");
    printk("[@dm9.frame err (hdr %02x %02x %02x %02x)] [%02x %02x %02x %02x %02x %02x] %lu\n",
      phd[0], phd[1], phd[2], phd[3], rdptr[0], rdptr[1], rdptr[2], rdptr[3], rdptr[4], rdptr[5],
      dev -> stats.rx_frame_errors);

    //[01 00 9e 00] unicast and len is less 256 ;"Custom report 20210204/'lan_aging_error3.log'"
    #if DISP_SKB_UNI_ERR
    if (1){
	    u16 len;
	    len = (phd[3] << 8) + phd[2];
	    if (phd[0] == 0x01 && phd[1] == 0x00 && phd[3] == 0x00) {
	      printk("[@dm9.[warn] unknow uni-cast frame (hdr %02x %02x %02x %02x)] %02x %02x %02x %02x %02x %02x\n",
		phd[0], phd[1], phd[2], phd[3], rdptr[0], rdptr[1], rdptr[2], rdptr[3], rdptr[4], rdptr[5]);
	      //; fail //return true;
	    } else if (phd[0] == 0x01 && phd[1] == 0x00 && phd[3] != 0x00)
	      printk("[@dm9.[warn] unknow uni-cast long frame (hdr %02x %02x %02x %02x)] len %d\n",
		phd[0], phd[1], phd[2], phd[3], len);
    }
    //db->mac_process = 1 - db->mac_process;
    #endif
  }        

  if (flg_disp) {
    //printnb_packet(rdptr, prxhdr -> RxLen - 4);
    //printnb_packet( & rdptr[prxhdr -> RxLen - 4], 4);
    return false;
  }

  db -> bC.rx_unicst_counter++;
  db -> nSCH_UniRX++;
  return true;
}

void list_all_scan(board_info_t * db, int nR, int nRR) {
  int i;
  for (i = 0; i < nRR; i++) {
    if (i < nR)
      printk(" (scan[%d]) %4x, sizeof(rxhdr) + XX XX, bnd-len 0x%x (Y)\n", i, db -> mdra_regs[i], db -> len_rxhdr_pkt[i]);
    else
      printk(" (scan[%d]) %4x, sizeof(rxhdr) + XX XX, bnd-len 0x%x (Err)\n", i, db -> mdra_regs[i], db -> len_rxhdr_pkt[i]);
  }
  printk("  scan_end %4x\n", db -> mdra_reg_end);
  printk("  -------------\n");
}
void list_all_headlist(char * headstr, int nR) {
  int i;
  printnb("[dm9 %s scanx report:", headstr);
  for (i = 0; i < nR; i++)
    printnb(" Y");
  printnb(" Err");
  printnb("]\n");
}
int dump_all_headlist(int nR, int nRR) {
  int s, i;
  s = 0;
  if (nRR > 5) s = nRR - 5;
  printnb("[dm9 (dump pkt) skbx report:");
  for (i = s; i < nRR; i++) {
    if (i < nR)
      printnb(" Y");
    else
      printnb(" Err");
  }
  printnb("]\n");
  return s;
}
void dump_all_skb(board_info_t * db, int s, int nR, int nRR) {
  char * sbufp = db -> sbuf;
  int i, p = 1;
  printnb_init(1); // here, not essential, since done in probe

  //calc p
  for (i = 0; i < s; i++) {
    printk("   skb[%d]  %4x, sizeof(rxhdr) + %02x %02x, bnd-len 0x%x (NOT_dump)\n", i, db -> mdra_regs[i], sbufp[p + 2], sbufp[p + 3], db -> len_rxhdr_pkt[i]);
    p += db -> len_rxhdr_pkt[i];
  }

  for (i = s; i < nRR; i++) {
    //[detect-check]
    if (i > nR) printk(" {Warn too more}");
    //[normal]
    printk("   skb[%d]  %4x, sizeof(rxhdr) + %02x %02x\n", i, db -> mdra_regs[i], sbufp[p + 2], sbufp[p + 3]);
    if (i < nR) {
      printk("   bnd-len 0x%x (Y)\n", db -> len_rxhdr_pkt[i]);
      printk("   inblk len 4 + 0x%x(= %d)\n", db -> len_rxhdr_pkt[i] - RXHDR_SIZE, db -> len_rxhdr_pkt[i] - RXHDR_SIZE);
    } else {
      printk("   bnd-len 0x%x (Err)\n", db -> len_rxhdr_pkt[i]);
      printk("   inblk len 4 + 0x%x(= %d)\n", db -> RdForLen, db -> RdForLen);
    }
    printk(" [dm9 dumpLen %x]\n", db -> len_rxhdr_pkt[i]);
    printnb_rx_fifo( & sbufp[p], RXHDR_SIZE, &
      sbufp[p + RXHDR_SIZE], db -> len_rxhdr_pkt[i] - RXHDR_SIZE);
    //p += sizeof(rxhdr);
    p += db -> len_rxhdr_pkt[i];
  }
}

//[Read write-pointer of dm9051-rx-ram write-into] READ_WP_RXRAM
void read_wr_of_rx(board_info_t * db, u16 * ptrwr) {
  * ptrwr = ior(db, 0x24); //v.s. 'DM9051_MRRL'
  * ptrwr |= (u16) ior(db, 0x25) << 8; //v.s. 'DM9051_MRRH'
  db -> mdwr_reg_end = * ptrwr;
}
//--
//[Read Read-pointer of dm9051-rx-ram read-out] READ_RP_RXRAM
#define read_rp_of_rx read_mrrx1
#define write_rp_of_rx write_mrrx1
static void read_mrrx1(board_info_t * db, u16 * ptrmrr) {
  * ptrmrr = ior(db, DM9051_MRRL);
  * ptrmrr |= (u16) ior(db, DM9051_MRRH) << 8;
  db -> mdra_reg_end = * ptrmrr; // save every-time
}
//..
//[Write Read-pointer of dm9051-rx-ram read-out] WRITE_RP_RXRAM
static void write_mrrx1(board_info_t * db, u16 ptrmrw) {
  iow(db, DM9051_MRRL, ptrmrw & 0xff);
  iow(db, DM9051_MRRH, (ptrmrw >> 8) & 0xff);
}   

#ifdef DM_DBG_ENDING_PACKET
static void dm9051_temp_ending_packet_reg_s(board_info_t * db)
{
  db->end_reg_tp = db -> mdra_reg_end; //[temp storage]
}
static void dm9051_copy_ending_packet_reg_s(board_info_t * db)
{
  db->end_reg_s = db->end_reg_tp; //[move temp to register s]
}                             
static void dm9051_copy_ending_packet_reg_e(board_info_t * db)
{                                               
  db->end_reg_e = db -> mdra_reg_end; //[assign to register e]
}
static void dm9051_copy_ending_packet(board_info_t * db, int p, int RxLen)
{
  // Total copied: RXHDR_SIZE + RxLen
  // prepare a full-packet memory buffer, first. 
  if ((RXHDR_SIZE+RxLen)<=DM_DBG_ENDBUF_LEN){
	  char * sbuff = db -> sbuf;
	  memcpy(db->endbuf, &sbuff[p], RXHDR_SIZE+RxLen);
	  db->endbuf_valid = 1;
  }
}
#endif

static void dump_end_packet(board_info_t * db)
{	
#ifdef DM_DBG_ENDING_PACKET		
  if (db->endbuf_valid){
	  struct dm9051_rxhdr * prxhdr = (struct dm9051_rxhdr * ) db->endbuf;
	  int RxLen = prxhdr -> RxLen;                                                                 
	  printk(" [dm9 endbuf: reg_s %04x, reg_e %04x]\n", db->end_reg_s, db->end_reg_e);
	  printk(" [endblk len 4 + (0x%x= %d) = %x]\n", RxLen, RxLen, 4+RxLen);
	  printnb_rx_fifo( &db->endbuf[0], 4, &db->endbuf[4], RxLen);
  }
#endif
}

#define DM_MAINTAIN_TO_RDP_HANDLE 1 // for _dm9051_scanx, enhancer ...
#define DM_WRP_TO_RDP_HANDLE 1 // for _dm9051_scanx & _rx_work_carrier, Be MAINTAIN RXP, otherwise The fifo NCR_RESET. Both have it recovery by/with unplug/plug CAT5 cable. 
#define DM9051_DUMP_LEN_BF_RST 52 //32 //768 //512         

int scanx_recover_mode = 0;
int link_recover_mode = 0;

#define LP_EMPTY_STR  "."   //old
#define LP_EMPTY_LSTR  "  ...  ..."  //"  5f2  5f2  5f2","    .    .    .",

//return 1: Error
static int list_pointers(char * strhead, int nRRmax, u16 * addr_regs, u16 * data_val, char endc) {
  int ret = 0;
  int line_s, s, i;
  printnb_init(1); // 1 for print-log, 0 for no print-log 
  i = 0;
  while (1) { //while (i< nRRmax)
    printnb(strhead); //"[dm9 scan] "
    line_s = i;
    s = i;
    printnb("[");
    while (i < nRRmax && addr_regs[i] < 0x2000)
      printnb(" %4x", data_val[i++]);
    if (i == s) printnb(LP_EMPTY_LSTR);
    printnb("]");
    //[dbg]	
    if (i == nRRmax) break;
    s = i;
    printnb("[");
    while (i < nRRmax && addr_regs[i] < 0x3400)
      printnb(" %4x", data_val[i++]);
    if (i == s) printnb(LP_EMPTY_LSTR);
    printnb("]");
    //[dbg]	
    if (i == nRRmax) break;
    s = i;
    printnb("[");
    while (i < nRRmax && addr_regs[i] >= 0x3400 && addr_regs[i] <= 0x3fff)
      printnb(" %4x", data_val[i++]);
    if (i == s) printnb(LP_EMPTY_LSTR);
    printnb("]");
    //[dbg]	
    if (i == nRRmax) break;
    if (i == line_s) {
      printnb("[  Illegal read of wp %4x (range=c00 3fff)]", addr_regs[line_s]);
      ret = 1;
      break;
    }
    printnb("\n");
  }
  printnb("%c\n", endc); //param: '.'
  return ret;
}
static void dm9051_list_scanx(board_info_t * db) {
  list_pointers("[dm9 scan] ", db -> bC.nRRMAX, db -> mdra_regs, db -> mdra_regs, '.');
  list_pointers(" [dm9 LEN] ", db -> bC.nRRMAX, db -> mdra_regs, db -> len_rxhdr_pkt, ' ');
}
static void dm9051_manual_mark(board_info_t * db)
{                         
                  if (db -> bC.rxbyte_ref_tx_counter) { // cause by 
                    db -> bC.rxbyte_has_tx_counter++; // cause by 'db -> bC.rxbyte_ref_tx_counter' non-zero
                  }
                  if (db -> bC.rxbyte_has_tx_counter > (db -> bC.rxbyte_ref_accumu * NUMRXBYTEABNORMAL)) { //'10 x n'
                    if (db -> bC.rxbyte_has_tx_counter > (5 * NUMRXBYTEABNORMAL)) { //'10 x 5'
                      
                      //.db -> bC.isbyte = ISR_MBS | ISR_PRS;
                      //isr |= ISR_MBS | ISR_PRS;
                      
                      if (db -> bC.rxbyte_has_tx_counter < ((5 * NUMRXBYTEABNORMAL)+16)) {
                        if (db -> bC.rxbyte_enter) {
                          db -> bC.rxbyte_enter = 0;
                          printk("[dm9 [ISR %02x],ref conti-TX reg_s %4x rxb %02x [%2d] tmp [%2d] pkts] will tbd solution [Set isr %x to %x]\n",  db -> bC.isbyte, db -> mdra_reg_end, 
                            db->bC.dummy_pading, db -> bC.rxbyte_has_tx_counter, db -> bC.rxbyte_ref_tx_tmp, 
                            db -> bC.isbyte, 
                            ISR_MBS | ISR_PRS);
                          if (db -> bC.rxbyte_ref_tx_counter) {
                            printk("[dm9 [ISR %02x],ref conti-TX reg_s %4x rxb %02x [%2d] has already inc, since ref_tx_counter %d\n",  db -> bC.isbyte, db -> mdra_reg_end, 
                                db->bC.dummy_pading, db -> bC.rxbyte_has_tx_counter, db -> bC.rxbyte_ref_tx_counter);
                          } else { // because by 'db -> bC.rxbyte_ref_tx_counter' is zero
                            printk("[dm9 [ISR %02x],ref conti-TX reg_s %4x rxb %02x [%2d] inc to [%2d], because ref_tx_counter 0\n",  db -> bC.isbyte, db -> mdra_reg_end, 
                                db->bC.dummy_pading, db -> bC.rxbyte_has_tx_counter, db -> bC.rxbyte_has_tx_counter+1);
                            db -> bC.rxbyte_has_tx_counter++; 
                          }
                        }
                      }
                      
                      //[solution] show endBuf ...
                      if (db -> bC.rxbyte_has_tx_counter == ((5 * NUMRXBYTEABNORMAL)+16)){
                        dump_end_packet(db);
                      }
                      //[solution] isr ...
                      if (db -> bC.rxbyte_has_tx_counter >= ((5 * NUMRXBYTEABNORMAL)+16)){
                        printk("--- (dm9 [ISR %02x],ref conti-TX reg_s %4x rxb %02x [%2d] tmp [%2d]) now do solution [Set isr %x to %x] ---\n",  db -> bC.isbyte, db -> mdra_reg_end, 
                          db->bC.dummy_pading, db -> bC.rxbyte_has_tx_counter, db -> bC.rxbyte_ref_tx_tmp, db -> bC.isbyte, ISR_MBS | ISR_PRS);
                        db -> bC.isbyte = ISR_MBS | ISR_PRS;
						
						if (db -> bC.rxbyte_ref_tx_counter)
							;
						else // because by 'db -> bC.rxbyte_ref_tx_counter' is zero
							db -> bC.rxbyte_has_tx_counter++; 
                      }

                    }
                  }
                
                  if (db -> bC.rxbyte_has_tx_counter <= (5 * NUMRXBYTEABNORMAL)) {       
					if (db -> bC.rxbyte_has_tx_counter == (1 * NUMRXBYTEABNORMAL)) {
						printk("[dm9 [ISR %02x],ref dump-EndPkt reg_s %4x rxb %02x [%2d] inc to [%2d].\n", db -> bC.isbyte, db -> mdra_reg_end, 
							db->bC.dummy_pading, db -> bC.rxbyte_has_tx_counter, db -> bC.rxbyte_has_tx_counter+1);
						db -> bC.rxbyte_has_tx_counter++; 
						dump_end_packet(db);
						#if 1 //[Once more to display]//[After this dump]
						db -> bC.nRRMAX = 0; // set 0, after this, for once more to display scanX (rp & len).....
						#endif
					}           
               #if 1
                    if (db -> bC.rxbyte_has_tx_counter == (db -> bC.rxbyte_ref_accumu * NUMRXBYTEABNORMAL)) //[tmp]
                     printk("[dm9 [ISR %02x],ref conti-TX reg_s %4x rxb %02x [%2d] tmp [%2d] pkts] to accumulate %d\n",  db -> bC.isbyte, db -> mdra_reg_end,  
                      db->bC.dummy_pading, db -> bC.rxbyte_has_tx_counter, db -> bC.rxbyte_ref_tx_tmp, db -> bC.rxbyte_ref_accumu);
               #endif
               #if 1
                    else 
                    if (db -> bC.rxbyte_has_tx_counter > (db -> bC.rxbyte_ref_tx_tmp+1))  { //[arbitary]
                      db -> bC.rxbyte_ref_tx_tmp = db -> bC.rxbyte_has_tx_counter;
                      
                      if (db -> bC.rxbyte_has_tx_counter > (1 * NUMRXBYTEABNORMAL))
                        printk("[dm9 [ISR %02x],ref conti-TX reg_s %4x rxb %02x (%2d) tmp (%2d) pkts] to accumulate %d\n",  db -> bC.isbyte, db -> mdra_reg_end,
                          db->bC.dummy_pading, db -> bC.rxbyte_has_tx_counter, db -> bC.rxbyte_ref_tx_tmp, db -> bC.rxbyte_ref_accumu);
                    }
               #endif
                    
                    if (db -> bC.rxbyte_has_tx_counter >= (db -> bC.rxbyte_ref_accumu * NUMRXBYTEABNORMAL)) //'10 x n'
                      db -> bC.rxbyte_ref_accumu++;
                  }
                  db -> bC.rxbyte_ref_tx_counter = 0; // cause by (end)
}

//return 1, as is fifo reset ('scanx_recover_mode')
static int dm9051_isr_mark(board_info_t * db)
{     
  int i;
  u16 wp[2];
  u16 rp[4]; //rp[3]&rp[4]
        if (db -> bC.rxbyte_counter >= NUMRXBYTEABNORMAL) //'10' only
        {
          // TO DO MORE ...
          dump_end_packet(db); //new.rpi4
          // TO DO MORE ...
          printk("[dm9] [ISR %02x] abnormal RXBYTE for %d times\n", db -> bC.isbyte, db -> bC.rxbyte_counter);
          for (i = 0; i < NUMRXBYTEABNORMAL; i++) {
            if ((i<(NUMRXBYTEABNORMAL-2)) && (db -> bC.rxbyte_pad[i]==0)) // fewer in the log, if 'rxb' zero
              continue;
            if ((i<(NUMRXBYTEABNORMAL-2)) && (db -> bC.rxbyte_pad[i]==db -> bC.rxbyte_pad[0])) // fewer in the log, if 'rxb' non-zero also identical to _pad[0]
              continue;
            printk("[dm9] time %d [ISR %02x] abnormal rp RXBYTE:  %4x %02x\n",
              i+1, db -> bC.isbyte, db -> bC.rxbyte_regs[i], db -> bC.rxbyte_pad[i]);
          }
          db -> bC.rxbyte_counter = 0;
        }
        if (db -> bC.rxbyte_counter == (NUMRXBYTEABNORMAL - 1)) //'9'
        {
          // TO DO MORE ...
          printk("[dm9] [ISR %02x] abnormal RXBYTE for %d times\n", db -> bC.isbyte, db -> bC.rxbyte_counter);
          printnb_init(1);
          printnb("dm9rxb");
          for (i = 0; i < db -> bC.rxbyte_counter; i++) printnb(" %4x", db -> bC.rxbyte_regs[i]);
          printnb("\n");
          printnb("dm9rxb");
          for (i = 0; i < db -> bC.rxbyte_counter; i++) printnb(" %4x", db -> bC.rxbyte_pad[i]);
          printnb("\n");
        }
        if (db -> bC.rxbyte_counter == 0) {
          //  because uplug/plug CAT5 cable can work recovery OK.)
          int RdForLen = DM9051_DUMP_LEN_BF_RST; //DM9051_PKT_MAX;
          char * sbuff = db -> sbuf;
          u16 rxwr_localtmp; // will equal to 'db->mdwr_reg_end'
          scanx_recover_mode++;
          scanx_recover_mode = scanx_recover_mode % 3;
#if 1
          //[Test always 'scanx_recover_mode']
          scanx_recover_mode = 1;
#endif      
          if (scanx_recover_mode) {
            read_wr_of_rx(db, & rxwr_localtmp);
            printk("[dm9rxb dump_and_then_fifo_re-pointer].s ...\n");
            wp[0] = db -> mdwr_reg_end; //printk(" [dm9rxb]  inblk_rxwr-s-Linkup %4x (get rxwr_end)\n", db->mdwr_reg_end); //or 'rxwr_localtmp'
            wp[1] = 0;
          } else
            printk("[dm9rxb dump_and_then_fifo_reset].s ...\n");
          read_rp_of_rx(db, & db -> mdra_regs[0]);
          rp[0] = db -> mdra_reg_end; //printk(" [dm9rxb]  inblk_mrrx-s-Linkup %4x\n", db->mdra_reg_end);

          //#ifdef QCOM_BURST_MODE
          //.dm9051_inblk_noncpy(db, & sbuff[1], RdForLen);
          //#else
          //.dbg_inblk(db, & sbuff[1], RdForLen); //=rdptr-1 =&sbuff[p-1]
          //#endif
		{
			#ifndef QCOM_BURST_MODE
			char bf1 = sbuff[0]; //* prevPtr; //sbuff[p-1];
			#endif	  
			dm9inblk(db, & sbuff[1], RdForLen);
		  //#ifdef QCOM_RX_DWORD_BOUNDARY
		  //dm9.lagcy_inblk_ncpy_dwbnd(db, & sbuff[1], RdForLen);
		  //#else
		  //dm9.lagcy_inblk_ncpy(db, & sbuff[1], RdForLen);
		  //#endif
			#ifndef QCOM_BURST_MODE
			sbuff[0] = bf1;
			#endif
		}

          read_rp_of_rx(db, & db -> mdra_regs[0]);
          rp[1] = db -> mdra_reg_end; //printk(" [dm9rxb]  inblk_mrrx-e-Linkup %4x\n", db->mdra_reg_end);
          rp[2] = 0;

          //#if DM_WRP_TO_RDP_HANDLE
          if (scanx_recover_mode) {
            int wpflag = list_pointers("[dm9rxb wp]", 1, wp, wp, 'w'); //
            if (wpflag) printnb("[ checking mdwr_end %4x]\n", db -> mdwr_reg_end);
            list_pointers("[dm9rxb rp]", 2, rp, rp, 'r'); //
            write_rp_of_rx(db, db -> mdwr_reg_end); //or 'rxwr_localtmp'       
            read_rp_of_rx(db, & db -> mdra_regs[0]);
            rp[2] = db -> mdwr_reg_end;
            rp[3] = 0; //printk(" [dm9rxb]  inblk_rxwr-e-Linkup %4x (set/get rxwr_end to mrrx)\n", db->mdwr_reg_end); //or 'rxwr_localtmp'    
            list_pointers("[dm9rxb nrp]", 3, rp, rp, 'R'); //
            printk(" [dm9 rxb: reg_s %04x, reg_e %04x]\n", rp[0], db -> mdra_reg_end); //db->end_reg_s, db->end_reg_e
            printk(" [dm9 rxb: dump len %d]\n", DM9051_DUMP_LEN_BF_RST);
            printk(" [dm9 rxb %x]\n", sbuff[1]);
            printnb_rx_fifo( & sbuff[1], 4, //for-dbg-check
              &
              sbuff[5], DM9051_DUMP_LEN_BF_RST - 4); // [4] + [DM9051_DUMP_LEN_BF_RST  - 4]
            printk("[dm9rxb dump_and_then_fifo_re-pointer].e ...\n");
            ++db -> bC.DO_RP_Update_counter;
			#if 0
			dump_end_packet(db);
			#endif
			bcopen_rx_info_clear( & db -> bC); //UPDATE OK
			DM9051_fifo_reset_flg = 1; //UPDATE OK
            printk("[dm9] update done:");
            printk(" Update_c %d\n", db -> bC.DO_RP_Update_counter - NUMBOOTRPUPDATE); //as RST_c
            printk(" (RST_c %d)\n", db -> bC.DO_FIFO_RST_counter);
            printk("\n");
            //.pns = 1 + RdForLen; //[for as the rule, fill zero before return]
          }
          //#else                                    
          else {
            list_pointers("[dm9rxb rp]", 2, rp, rp, 'r'); //
            printk(" [dm9 rxb: reg_s %04x, reg_e %04x]\n", rp[0], db -> mdra_reg_end); //db->end_reg_s, db->end_reg_e
            printk(" [dm9 rxb: dump len %d]\n", DM9051_DUMP_LEN_BF_RST); //'RdForLen'
            printk(" [dm9 rxb %x]\n", sbuff[1]);
            printnb_rx_fifo( & sbuff[1], 4, //for-dbg-check
              &
              sbuff[5], DM9051_DUMP_LEN_BF_RST - 4); // [4] + [DM9051_DUMP_LEN_BF_RST  - 4]
            db -> bC.RXBErr_counter++;
            dm9051_fifo_reset(11, "aft_dump_reset", db);
            dm9051_fifo_reset_statistic(db);
            printk("[dm9rxb dump_and_then_fifo_reset].e ...\n");
            read_rp_of_rx(db, & db -> mdra_regs[0]);
			#if 0
			dump_end_packet(db);
			#endif
            printk("[dm9] reset-done: of RxbErr\n");
            printk(" RXBErr_c %d, RST_c %d\n", db -> bC.RXBErr_counter, db -> bC.DO_FIFO_RST_counter);
            //printk(" RxLenErr&MacOvrSft_Er %d, RST_c %d\n", db->bC.ERRO_counter, db->bC.DO_FIFO_RST_counter);
            printk(" (Update_c %d)\n", db -> bC.DO_RP_Update_counter - NUMBOOTRPUPDATE);
            printk("[dm9] mdra:\n");
            printk(" ncr-reset %4x\n", db -> mdra_reg_end);
            //.pns = 1 + RdForLen; //[for as the rule, fill zero before return]        
          }
          //#endif
          return 1;
          //[eventually, break;]
        }
        return 0;
}

static int dm9051_scanx(board_info_t * db, int nMax) {
  int nRR;
  int nR = 0, scanRR = 0; //[note: 'nR' is totally equal to 'scanRR']
  char * sbuff = db -> sbuf;
  int p = 1;
  int pns = 1; //next start
  struct dm9051_rxhdr * prxhdr; //struct spi_rxhdr spirxhdr;

  int RdForLen, RxLen;
  u8 rxbyte;
  #ifndef QCOM_BURST_MODE
  char bf1; //declaration
  #endif //QCOM_BURST_MODE

  db -> bC.isbyte = ior(db, DM9051_ISR); // Got ISR	
  db -> bC.isr_clear = 0;

  db -> sScanSize = 0;
  
  db -> bC.rxbyte_enter = 1;
  read_rp_of_rx(db, & db -> mdra_regs[nR]);
  while (scanRR < nMax) {          
   #ifdef DM_DBG_ENDING_PACKET
    dm9051_temp_ending_packet_reg_s(db);
   #endif
    rxbyte = ior(db, DM_SPI_MRCMDX); /* Dummy read */
    rxbyte = ior(db, DM_SPI_MRCMDX); /* Dummy read */
    db->bC.dummy_pading = rxbyte;
    if (rxbyte != DM9051_PKT_RDY) {
      
      #if DM_MAINTAIN_TO_RDP_HANDLE
      //[Handler RXBYte != 0x01]
      db -> bC.rxbyte_regs[db -> bC.rxbyte_counter] = db -> mdra_reg_end; //test
      db -> bC.rxbyte_pad[db -> bC.rxbyte_counter] = rxbyte;                                //test
      db -> bC.rxbyte_isr[db -> bC.rxbyte_counter] = db -> bC.isbyte; //test
      
      //db -> bC.isbyte = dm9051_remark(db, db -> bC.isbyte);
      if (!(db -> bC.isbyte & ISR_PRS)) {
        dm9051_manual_mark(db);
      }
      if (db -> bC.isbyte & ISR_PRS) { //ISR_PRS --> 0X01
        db -> bC.rxbyte_regs[db -> bC.rxbyte_counter] = db -> mdra_reg_end;
        db -> bC.rxbyte_pad[db -> bC.rxbyte_counter] = rxbyte;
        db -> bC.rxbyte_counter++;
        if (dm9051_isr_mark(db))
          pns = DM9051_DUMP_LEN_BF_RST + 1; // fifo reset ('scanx_recover_mode')
      }
      #endif //DM_MAINTAIN_TO_RDP_HANDLE
      
      if (!db -> bC.isr_clear)
        iow(db, DM9051_ISR, 0xff);
      break; //exhaust-empty
    }
    #if DM_MAINTAIN_TO_RDP_HANDLE
    db -> bC.rxbyte_ref_accumu = REF_ACCMU_START; //test
    db -> bC.rxbyte_ref_tx_counter = db -> bC.rxbyte_has_tx_counter = db -> bC.rxbyte_ref_tx_tmp = 0; //test
    db -> bC.rxbyte_counter = 0; // part of : TO DO MORE ... (of '_DM_MAINTAIN_TO_RDP_HANDLE')
    #endif

    if ((p + RXHDR_SIZE) > (SCAN_LEN_HALF + 1)) {
      printk("dm9 p+RXHDR_SIZE over-range\n");
      break; //protect
    }

    //db->bC.dummy_pad = rxbyte;
    db -> bC.dummy_pad_pointer = & sbuff[p];

    //dm9051_inblk_noncpy(db, &spirxhdr.spiwb, RXHDR_SIZE);
    #ifdef QCOM_BURST_MODE
    dm9inblk(db, & sbuff[p], RXHDR_SIZE); //dm9.lagcy_inblk_ncpy,dm9051_inblk_noncpy
    #else
    bf1 = sbuff[p - 1];
    dm9inblk(db, & sbuff[p - 1], RXHDR_SIZE); //dm9.lagcy_inblk_ncpy,dm9051_inblk_noncpy
    sbuff[p - 1] = bf1;
    #endif
	
    iow(db, DM9051_ISR, 0xff);
    db -> bC.isr_clear = 1;

    prxhdr = (struct dm9051_rxhdr * ) & sbuff[p];
    RxLen = prxhdr -> RxLen;
    p += RXHDR_SIZE;
    if ((p + RxLen) > (SCAN_LEN_HALF + 1)) {
      printk("dm9 warn reach, p+RxLen over SCAN_LEN_HALF+1 (%d > %d)\n", p + RxLen, SCAN_LEN_HALF + 1);
      printk("dm9-pkt-Wrong RxLen and range (RxLen %x= %d range %d)\n", RxLen, RxLen, DM9051_PKT_MAX);
      RdForLen = SCAN_LEN_HALF - p; //read-for-len, instead
      printk("dm9-read-for-len, instead %x = %d\n", RdForLen, RdForLen);
      goto inblk_dump;
    }
    if (RxLen > DM9051_PKT_MAX) {
      int s; //added
      printk("dm9-pkt-Wrong RxLen over-range (%x= %d > %x= %d)\n", RxLen, RxLen, DM9051_PKT_MAX, DM9051_PKT_MAX);
      //break;
      RdForLen = DM9051_PKT_MAX; //read-for-len, instead
      printk("dm9-read-for-len, instead %x = %d\n", RdForLen, RdForLen);
      inblk_dump:

        printnb_rx_fifo( & sbuff[p - RXHDR_SIZE], RXHDR_SIZE, //for-dbg-check
          &
          sbuff[p - RXHDR_SIZE], RXHDR_SIZE);

      //[inblk this and dump-list]
      db -> RdForLen = RdForLen;
	  
      //#ifdef QCOM_BURST_MODE
      //.dm9051_inblk_noncpy(db, & sbuff[p], RdForLen);
      //#else
      //.dbg_inblk(db, & sbuff[p - 1], RdForLen); //=rdptr-1 =&sbuff[p-1]
      //#endif
	#ifndef QCOM_BURST_MODE
	  bf1 = sbuff[p - 1]; //* prevPtr; //sbuff[p-1];
	#endif	  
		dm9inblk(db, & sbuff[p], RdForLen);
	  //#ifdef QCOM_RX_DWORD_BOUNDARY
	  //dm9.lagcy_inblk_ncpy_dwbnd(db, & sbuff[p], RdForLen);
	  //#else
	  //dm9.lagcy_inblk_ncpy(db, & sbuff[p], RdForLen);
	  //#endif
	#ifndef QCOM_BURST_MODE
	  sbuff[p - 1] = bf1;
	#endif

      printnb_rx_fifo( & sbuff[p], 24, //for-dbg-check
        &sbuff[p], 24);

      db -> len_rxhdr_pkt[nR] = RXHDR_SIZE + RxLen;

      //nR++;
      nRR = scanRR + 1;
      read_rp_of_rx(db, & db -> mdra_regs[nRR]);

      //[dump-here]
      do {
        char hstr[72];
        sprintf(hstr, "dmfifo_reset( 11 RxLenErr ) mac %02x %02x %02x %02x rxhdr %02x %02x %02x %02x (quick)",
          sbuff[p], sbuff[p + 1], sbuff[p + 2], sbuff[p + 3],
          sbuff[p - 4], sbuff[p - 3], sbuff[p - 2], sbuff[p - 1]);
        db -> bC.LARGErr_counter++; //db -> bC._ERRO_counter++;
        dm9051_fifo_reset(11, hstr, db);
        dm9051_fifo_reset_statistic(db);
      } while (0);

      /* debug */
      list_all_headlist("(RxLenerr)", nR); //printk("dm9 [RxLenerr] .....
      printk("dm9 mdra:\n");
      list_all_scan(db, nR, nRR /*nRR*/ );
      
      s = dump_all_headlist(nR, nRR /*nRR*/ );
      printk("dm9 mdra:\n");
      dump_all_skb(db, s, nR, nRR /*nRR*/ );           
      printk("  -------------\n");
      read_rp_of_rx(db, & db -> mdra_regs[0]);
      printk("dm9 reset-done: of LargeRxLen\n");
      printk(" RxLenErr&MacOvrSft_Er %d, RST_c %d\n", db -> bC.ERRO_counter, db -> bC.DO_FIFO_RST_counter);
      printk(" (Update_c %d)\n", db -> bC.DO_RP_Update_counter - NUMBOOTRPUPDATE);
      printk("dm9 mdra:\n");
      printk(" ncr-reset %4x\n", db -> mdra_reg_end);
      //[return 0]
      db -> sbuf[1] = 0; //for skip dm9051_skbx()
      return 0;
    }

    #ifdef QCOM_BURST_MODE
    dm9inblk_vir(db, & sbuff[p], RxLen); //dm9.inblk_virpacket, dm9051_inblk_virtual_packet//dm9051_inblk_noncpy(db, &sbuff[p], RxLen); /*rdptr*/
    #else
    bf1 = sbuff[p - 1];
    dm9inblk_vir(db, & sbuff[p - 1], RxLen); //dm9.inblk_virpacket, dm9051_inblk_virtual_packet//dm9051_inblk_noncpy(db, &sbuff[p-1], RxLen); /*rdptr-1*/
    sbuff[p - 1] = bf1;
    #endif
	
    iow(db, DM9051_ISR, 0xff);
    db -> bC.isr_clear = 1;

    p += RxLen;
    db -> len_rxhdr_pkt[nR] = RXHDR_SIZE + RxLen;

    db -> sScanSize += RXHDR_SIZE + RxLen;
    nR++;
    scanRR = nR;
    read_rp_of_rx(db, & db -> mdra_regs[nR]);
    pns = p;
    
   #ifdef DM_DBG_ENDING_PACKET                                       
    dm9051_copy_ending_packet_reg_s(db);
    dm9051_copy_ending_packet_reg_e(db);
    dm9051_copy_ending_packet(db, p - RXHDR_SIZE - RxLen, RxLen);
   #endif
  }
  sbuff[pns] = 0; // in case, to not 0x01.
  return scanRR;
}

static void dm9051_dbg_scanx(board_info_t * db, int nRR) {
  int kb_dot_digi = 0;
  if ((db -> sScanSize % 1024) >= 512)
    kb_dot_digi = 5;

  if (db -> bC.sScanSizeMax < db -> sScanSize) //Max calc only, no used in place others
    db -> bC.sScanSizeMax = db -> sScanSize;

  if (nRR && (db -> bC.nRRMAX < nRR)) {
    int update_c = 0;
    if (db -> bC.DO_RP_Update_counter) //addedd ever, valid-check
      update_c = db -> bC.DO_RP_Update_counter - NUMBOOTRPUPDATE;
    db -> bC.nRRMAX = nRR;
    printk("-------------------------------------------------------[nScanX %d]\n", nRR);
    dm9051_list_scanx(db);
    printk("[SCAN_END %4x]---------------------------------[SCAN_SIZE %d.%dKB] rst_c %d (update_c %d)\n", 
      db -> mdra_regs[nRR], db -> sScanSize / 1024, kb_dot_digi, 
      db -> bC.DO_FIFO_RST_counter,
      update_c);
    if (db -> bC.nRRMAX == NUM_SCANX)
      printk("--------------------------------------------------------[MAX_DONE]\n");
    #if 1 //[Once displaed is ok]//[After reset(or recovery)]
    db -> bC.nRRMAX = NUM_SCANX; // set MAX, after this, for not to display scanX (rp & len).....
    #endif
  }
}
static int dm9051_skbx(board_info_t * db, int nRR) {
  char * sbufp = db -> sbuf;
  int p = 1;
  int nR = 0;
  int skbRR = 0, RxLen;
  u8 * rdptr;
  struct sk_buff * skb;
  struct dm9051_rxhdr rxhdr;

  while (sbufp[p] == DM9051_PKT_RDY) {
    //db->bC.dummy_pad = sbufp[p];
    //db->bC.dummy_pad_pointer = &sbufp[p];
    memcpy((char * ) & rxhdr, & sbufp[p], RXHDR_SIZE);
    RxLen = rxhdr.RxLen;

    p += sizeof(rxhdr);
    if (!SB_skbing_packet_chk_data(db, & sbufp[p - sizeof(rxhdr)], & sbufp[p])) { //(u8 *) //(u8 *) 
      db -> RdForLen = (u16) RxLen;
      
      do {
        char hstr[72]; //dump_all_skb
        sprintf(hstr, "dmfifo_reset( 11 macErr ) mac %02x %02x %02x %02x rxhdr %02x %02x %02x %02x (quick)",
          sbufp[p], sbufp[p + 1], sbufp[p + 2], sbufp[p + 3],
          sbufp[p - 4], sbufp[p - 3], sbufp[p - 2], sbufp[p - 1]);
        db -> bC.ERRO_counter++;
        dm9051_fifo_reset(11, hstr, db);
        dm9051_fifo_reset_statistic(db);
      } while (0);
#if !DISP_SKB_UNI_ERR      
		db -> bC.nRRMAX = NUM_SCANX; // set MAX for not to display scanX (rp & len).....
#endif
#if DISP_SKB_UNI_ERR
		if (1){
			  /* debug */
			  int s;
			  list_all_headlist("(mac err)", nR); //printk("dm9 [mac err] .....
			  printk("dm9 mdra:\n");
			  list_all_scan(db, nR, nRR);
			  //dump	
			  s = dump_all_headlist(nR, nRR);
			  printk("dm9 mdra:\n");
			  dump_all_skb(db, s, nR, nRR);          
			  printk("  -------------\n");      
			//dump_end_packet(db)    ;
			//printk("  -------------\n");
		}
		printk("dm9 reset-done: of Skb-checkData\n");
#endif
      //new
      read_rp_of_rx(db, & db -> mdra_regs[0]);
      printk(" dm9 MacOvrSft_Er %d, RST_c %d\n", db -> bC.ERRO_counter, db -> bC.DO_FIFO_RST_counter);
      
#if DISP_SKB_UNI_ERR
      printk(" (Update_c %d)\n", db -> bC.DO_RP_Update_counter - NUMBOOTRPUPDATE);
      printk("dm9 mdra:\n");
      printk(" ncr-reset %4x\n", db -> mdra_reg_end);
#endif
      return skbRR; //nR;
    }

    #ifdef ASL_RSRV_RX_HDR
    if ((skb = dev_alloc_skb(RxLen + 4 + ASL_RSRV_RX_HDR_LEN)) == NULL)
      #else
    if ((skb = dev_alloc_skb(RxLen + 4)) == NULL) // SKB_size= RxLen + 4
      #endif
      {
        printk("dm9051 [!ALLOC skb size %d fail]\n", RxLen + 4);
        return skbRR; //nR;
      }
    skb_reserve(skb, 2);
    rdptr = (u8 * ) skb_put(skb, RxLen - 4); // [save 4] SKB_size - 4 - ASL_RSRV_RX_HDR_LEN - 4
    memcpy(rdptr, & sbufp[p], RxLen - 4); // [save 4] &sbufp[p]
    #if 1
    skb -> protocol = eth_type_trans(skb, db -> ndev); // [JJ found: skb->len -= 14]
    if (db -> ndev -> features & NETIF_F_RXCSUM) {
      //if ((((rxbyte & 0x1c) << 3) & rx, int nRRbyte) == 0)
      //	skb->ip_summed = CHECKSUM_UNNECESSARY;
      //else
      skb_checksum_none_assert(skb);
    }
    #endif
    if (in_interrupt())
      netif_rx(skb);
    else
      netif_rx_ni(skb);

    db -> ndev -> stats.rx_bytes += RxLen;
    db -> ndev -> stats.rx_packets++;

    p += RxLen;
    nR++;
    skbRR = nR;
  }
  return skbRR;
}
#endif

//[#include "new_sched1.c"]
void dm9051_spimsg_init(board_info_t * db) {
  //spi_message_init(&dm9.Tmsg);
  //spi_message_add_tail(&dm9.Tfer,&dm9.Tmsg);
  #if DEF_SPICORE_IMPL1
  #ifdef QCOM_BURST_MODE
  memset( & db -> spi_xfer2, 0, sizeof(struct spi_transfer) * 2); //[Add.] 
  spi_message_init( & db -> spi_msg2);
  spi_message_add_tail( & db -> spi_xfer2[0], & db -> spi_msg2);
  spi_message_add_tail( & db -> spi_xfer2[1], & db -> spi_msg2);
  #else
  memset( & db -> Tfer, 0, sizeof(struct spi_transfer)); //[Add.] 
  spi_message_init( & db -> Tmsg);
  spi_message_add_tail( & db -> Tfer, & db -> Tmsg);
  db -> fer = & db -> Tfer;
  db -> msg = & db -> Tmsg;
  #endif
  #endif
}

static int dm9051_sch_cnt1(u16 getVAL, u16 offset) // large equ +1 (as increasment)
{
  static u16 Sav1 = 0;
  if (!offset) // an option that always no need printed, so return 0. 
    return 0;
  if (getVAL >= (Sav1 + offset)) {
    Sav1 = getVAL;
    return 1;
  }
  return 0;
}

//Testing...JJ5_DTS
#define GLUE_LICENSE_PHYPOLL (3 + 2)
#define GLUE_LICENSE_INT (3 + 1)
#define GLUE_LICENSE_LE_EXPIRE (3 - 1)

static void
dm9051_INTPschedule_isr(board_info_t * db, int sch_cause) {
  //spin_lock(&db->statelock_tx_rx);//mutex_lock(&db->addr_lock);
  //spin_unlock(&db->statelock_tx_rx);//mutex_unlock(&db->addr_lock);
  if (dm9051_sch_cnt1(db -> nSCH_XMIT, 0)) //1500, 5500, 0
    printk("dm9-INFO TX %02d, sched R_SCH_XMIT %d (=%d) send skb_dequeue(txq).. \n", db -> nSCH_XMIT, R_SCH_XMIT, sch_cause); //, db->rx_count

  db -> sch_cause = sch_cause;

  #ifdef DM_CONF_POLLALL_INTFLAG
  if (sch_cause == R_SCH_INIT)
    return;
  if (sch_cause == R_SCH_INT_GLUE)
    return;
  //if (sch_cause== R_SCH._INFINI) 
  //	return;
  #endif

  switch (sch_cause) {
  case R_SCH_INIT:
    db -> nSCH_INIT++; // if (m<250) m++; 
    //	schedule_delayed_work(&db->rx._work, 0); //dm9051_continue_poll
    break;
  case R_SCH_INFINI:
    db -> nSCH_INFINI++;
    //	schedule_delayed_work(&db->rx._work, 0);  //dm9051_continue_poll
    break;
    //case R_SCH_LINK:
    //break;
  case R_SCH_PHYPOLL:
    break;
  case R_SCH_INT:
    db -> nSCH_INT++;
    //	schedule_delayed_work(&db->rx._work, 0); //dm9051_continue_poll 
    break;
  case R_SCH_INT_GLUE:
    db -> nSCH_INT_Glue++;
    #ifdef DM_CONF_POLLALL_INTFLAG
    DM9051_int_token++; //.DM9051_int_token++;
    #endif
    break;
  case R_SCH_XMIT:
    #ifdef DM_CONF_POLLALL_INTFLAG
    DM9051_int_token++;
    #endif
    //db->_nSCH_XMIT++;
    //.printk("(%d)dm9051_start_xmit, Need skb = skb_dequeue(&db->txq) to get tx-data\n", db->_nSCH_XMIT);
    break;
  }
  //spin_lock(&db->statelock_tx_rx);//mutex_lock(&db->addr_lock);
  switch (sch_cause) {
    #ifdef DM_CONF_INTERRUPT
    #ifndef DM_CONF_THREAD_IRQ
  case R_SCH_INT:
    schedule_delayed_work( & db -> rx_work, 0);
    break;
    #endif
    #else
  case R_SCH_INIT:
  case R_SCH_INFINI: //'POLLING'
  case R_SCH_INT_GLUE:
    //case R_SCH_LINK:
    //schedule_delayed_work(&db->rx_work, 0); //dm9051_continue_poll
    //break;
    schedule_delayed_work( & db -> rx_work, 0);
    break;
    #endif
  case R_SCH_PHYPOLL:
    #ifdef MORE_DM9051_MUTEX
    schedule_delayed_work( & db -> phypoll_work, 0);
    #else
    schedule_delayed_work( & db -> rx_work, 0);
    #endif
    break;

    #ifdef DM_CONF_INTERRUPT
  case R_SCH_XMIT:
    #ifdef MORE_DM9051_MUTEX
    schedule_delayed_work( & db -> xmit_work, 0);
    #else
    schedule_delayed_work( & db -> rx_work, 0); //dm9051_continue_poll
    //[OR] schedule_delayed_work(&db->tx_work, 0); //(dm9051_tx_work) This which need tryLOck() or Mutex() design.
    #endif
    break;
    #endif
    /* 0, Because @delay: number of jiffies to wait or 0 for immediate execution */
  }
  //spin_unlock(&db->statelock_tx_rx);//mutex_unlock(&db->addr_lock);
}

#ifdef DM_CONF_POLLALL_INTFLAG
#elif DRV_POLL_1
#if!defined DM_EXTREME_CPU_MODE && !defined DM_LIGHT_RX //20210308	
static void
dm9051_INTPschedule_weight(board_info_t * db, unsigned long delay) {
  static int sd_weight = 0;
  if (db -> DERFER_rwregs[MD_ReadPTR] != db -> DERFER_rwregs1[MD_ReadPTR]) {
    schedule_delayed_work( & db -> rx_work, 0);
    return;
  }

  //good.all.readout
  if (db -> DERFER_rwregs1[RXM_WrtPTR] == db -> DERFER_rwregs1[MD_ReadPTR]) { //THis is also 'db->DERFER_calc1>>8 == 0'
    //mdwa = 0;
    schedule_delayed_work( & db -> rx_work, delay);
    return;
  }

  sd_weight++;
  if (!(sd_weight % 3)) /* "slower" by more delay_work(delay) */
  /*if (!(sd_weight%5)) */
  /*if (!(sd_weight%6)) */
  {
    //normal
    if (sd_weight >= 6000) /*(sd_weight>=5000) in disp no adj*/
      sd_weight = 0;

    if (sd_weight == 0 && (db -> DERFER_calc1 >> 8) != 0) // fewer disp
      printk("-[dm9 SaveCPU for: MDWA 0x%x (RO %d.%d%c)]-\n", db -> DERFER_rwregs1[RXM_WrtPTR], db -> DERFER_calc1 >> 8, db -> DERFER_calc1 & 0xff, '%');

    schedule_delayed_work( & db -> rx_work, delay); // slower ,
    return;
  }
  schedule_delayed_work( & db -> rx_work, 0);
}
#endif //!DM_EXTREME_CPU_MODE && !DM_LIGHT_RX	
#endif

// SUB_DM9051.C
#if defined DM_CONF_PHYPOLL
void dm_schedule_phy(board_info_t * db) {
  //schedule_delayed_work(&db->phy._poll, HZ * 2); to be 3 seconds instead
  //schedule_delayed_work(&db->phy._poll, HZ * 3);
  schedule_delayed_work( & db -> phy_poll, HZ * 2);
}
#endif

void sched_work(board_info_t * db) {
  schedule_work( & db -> rxctrl_work);
}

/*[DM9051_Device_Ops.c]*/
void dm_sched_start_rx(board_info_t * db) // ==> OPEN_init_sched_delay_work
{
  if (db -> driver_state != DS_POLL) {
    db -> driver_state = DS_POLL;
    dm9051_INTPschedule_isr(db, R_SCH_INIT); //#ifndef DM_CONF_POLLALL_INTFLAG, #endif
  }
}

netdev_tx_t dm_sched_tx_via_sched_rx(struct sk_buff * skb, struct net_device * dev) {
  board_info_t * db = netdev_priv(dev);
  if (db -> nSCH_XMIT <= db -> nSCH_XMIT_WAVE_PDL) {
    char * p = (char * ) skb -> data;
    //[Mostly reach 'NUM_TOTAL_ALL']
    //'NUM_TOTAL_ALL' = 'NUM_SCH_XMIT_WAVE'*'NUM_TRIPS_OF_WAVE' //is originally defined as 5*5
    if (db -> nSCH_XMIT <= NUM_TOTAL_ALL)
      db -> nSCH_XMIT++; //finally,is 'NUM_TOTAL_ALL'+1, so also following print-out is ...

    if (db -> nSCH_XMIT <= db -> nSCH_XMIT_WAVE_PDL)
      printk("[dm9] .ndo_start_xmit [%02x] [%02x] [%02x], len %3d, %d times\n", p[0], p[1], p[2], skb -> len, db -> nSCH_XMIT); //when is 'NUM_TOTAL_ALL'+1, so also this print-out is disabled.
    else // [if (db->nSCH_XMIT <= (NUM_TOTAL_ALL+1))]
      printk("[dm9] .ndo_start_xmit [%02x] [%02x] [%02x], len %3d, %d times.....\n", p[0], p[1], p[2], skb -> len, db -> nSCH_XMIT); //when is 'NUM_TOTAL_ALL'+1, so also this print-out is the latest.
  }

  #if DM_CONF_APPSRC & DM9051_CONF_TX
  toend_stop_queue1(dev, 1); // for 'dm9051_tx'
  skb_queue_tail( & db -> txq, skb); // JJ: a skb add to the tail of the list '&db->txq'
  #endif
  #ifdef DM_CONF_POLLALL_INTFLAG
  dm9051_INTPschedule_isr(db, R_SCH_XMIT); // of 'dm9051_start_xmit', one_more_as_for_tx
  #endif
  return NETDEV_TX_OK;
}

//[for 'rxctrl_work']
void dm_sched_multi(struct net_device * dev) {
  board_info_t * db = netdev_priv(dev);
  sched_work(db);
}

static void
dm_hash_table_unlocked(struct net_device * dev) {
  board_info_t * db = netdev_priv(dev);
  #ifdef JABBER_PACKET_SUPPORT
  u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN | RCR_DIS_WATCHDOG_TIMER;
  #else
  u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
  #endif
  #if DEF_SPIRW
  struct netdev_hw_addr * ha;
  int i, oft;
  u32 hash_val;
  u16 hash_table[4];
  for (i = 0, oft = DM9051_PAR; i < 6; i++, oft++)
    iiow(db, oft, dev -> dev_addr[i]);

  /* Clear Hash Table */
  for (i = 0; i < 4; i++)
    hash_table[i] = 0x0;

  /* broadcast address */
  hash_table[3] = 0x8000;

  if (dev -> flags & IFF_PROMISC)
    rcr |= RCR_PRMSC;

  if (dev -> flags & IFF_ALLMULTI)
    rcr |= RCR_ALL;

  /* the multicast address in Hash Table : 64 bits */
  netdev_for_each_mc_addr(ha, dev) {
    hash_val = ether_crc_le(6, ha -> addr) & 0x3f;
    hash_table[hash_val / 16] |= (u16) 1 << (hash_val % 16);
  }

  /* Write the hash table */
  for (i = 0, oft = DM9051_MAR; i < 4; i++) {
    iiow(db, oft++, hash_table[i]);
    iiow(db, oft++, hash_table[i] >> 8);
  }

  iow(db, DM9051_RCR, rcr);
  #endif
  db -> rcr_all = rcr;
  /*
  //TEST
  	db->rcr_all |= RCR_PRMSC | IFF_ALLMULTI;
  	printk("Test db->rcr_all from %02x to %02x\n", rcr, db->rcr_all);
  */
}

static void
dm_hash_table(board_info_t * db) {
  struct net_device * dev = db -> ndev; //board_info_t *db = netdev_priv(dev);
  mutex_lock( & db -> addr_lock);
  dm_hash_table_unlocked(dev);
  mutex_unlock( & db -> addr_lock);
}

static void
rx_mutex_hash_table(board_info_t * db) {
  if (db -> Enter_hash) {
    dm_hash_table(db);
    db -> Enter_hash = 0;
  }
}

//  INT 
void int_reg_stop(board_info_t * db) {
  #if DEF_SPIRW
  iiow(db, DM9051_IMR, IMR_PAR); // Disable all interrupts 
  if (db -> nSCH_INT && (db -> nSCH_INT <= DM9_DBG_INT_ONOFF_COUNT))
    printk("[dm9IMR].[%02x].dis ------- nINT= %d\n",
      iior(db, DM9051_IMR), db -> nSCH_INT);
  #endif
}

void int_reg_start(board_info_t * db, char * headstr) {
  #if DEF_SPIRW
  iiow(db, DM9051_IMR, db -> imr_all); /*iow*/
  if (db -> nSCH_INT && (db -> nSCH_INT <= DM9_DBG_INT_ONOFF_COUNT))
    printk("%s.[%02x].ena ------- nINT= %d\n", headstr,
      iior(db, DM9051_IMR), db -> nSCH_INT); // Re-enable by interrupt mask register
  #endif
}

void IMR_DISABLE(board_info_t * db) {
  #ifdef DM_CONF_POLLALL_INTFLAG
  if (!DM9051_int_en) { // Note.ok. 
    //if (db->sch_cause==R_SCH_INT) {
    mutex_lock( & db -> addr_lock);
    int_reg_stop(db);
    mutex_unlock( & db -> addr_lock);
    //}
  }
  #endif
}

bool ISR_RE_STORE(board_info_t * db) {
  #ifdef DM_CONF_POLLALL_INTFLAG
  static unsigned short ctrl_rduce = 0;
  if (!DM9051_int_en) // Note that: Come-in with 'if (db->sch_cause==R_SCH_INT)' TRUE.
  {

    #if defined WR_ISR_ENDOF_RXWORK_ONLY //to-do-check-how-to...
    mutex_lock( & db -> addr_lock);
    db -> bC.isbyte = ior(db, DM9051_ISR); // Got ISR
    if (db -> bC.isbyte & 0x7e) {

      //if (db->bC.isbyte == 0x82) ; // [only 'PT']

      if (db -> bC.isbyte & 0x03) //(db->bC.isbyte & 0x01)
        ctrl_rduce++; // [with 'PT' or 'PR']
      else // somethings, BUT without PT or PR
        printk("[isr_reg] ISR= 0x%02x (somethings, BUT without PT or PR) Warn-Rare: overflow suspected\n", db -> bC.isbyte);

      iiow(db, DM9051_ISR, db -> bC.isbyte); // Clear ISR status
    } else {
      if (db -> bC.isbyte & 0x01)
        iiow(db, DM9051_ISR, db -> bC.isbyte); // Clear ISR status //printk("[int_reg].e WITH PR: Wr ISR= 0x%02x\n", db->bC.isbyte);
    }
    mutex_unlock( & db -> addr_lock);
    #endif

    return true;
  }
  #endif
  return false;
}

void IMR_ENABLE(board_info_t * db, int with_enable_irq) {
  mutex_lock( & db -> addr_lock);
  #ifdef DM_CONF_POLLALL_INTFLAG
    ...................VFJNKJNKBR................................................................................
    if (!DM9051_int_en) { // Note that: Come-in with 'if (db->sch_cause==R_SCH_INT)' TRUE.

      if (with_enable_irq) {

        if ((db -> nSCH_INT <= DM_CONF_SPI_DBG_INT_NUM)) // || (db->nSCH_INT == 24)
          printk("[%s][%d].enable_irq\n", "dm951_irq", db -> nSCH_INT); //from-"dm9051_rx_work"
        int_en(db -> ndev);
      }

      int_reg_start(db, "[dm9IMR]"); // "dm9IMR_irx_work", rxp no chg, if ncr-rst then rxp 0xc00 
    }
  if (DM9051_fifo_reset_flg) {
    #if DEF_SPIRW
    iiow(db, DM9051_RCR, db -> rcr_all); // if ncr-rst then rx enable
    #endif
    DM9051_fifo_reset_flg = 0;
  }
  #else
  if (DM9051_fifo_reset_flg) {
    int_reg_start(db, "[dmIMR_poll_rx_work]"); // exactly ncr-rst then rxp to 0xc00
    #if DEF_SPIRW
    iiow(db, DM9051_RCR, db -> rcr_all); // exactly ncr-rst then rx enable
    #endif
    DM9051_fifo_reset_flg = 0;
  }
  #endif
  mutex_unlock( & db -> addr_lock);
}

//[../new_load/control_sub.c]
static int dm9051_sch_cnt_chang(u16 nEnter) // large equ +1 (as increasment)
{
  static u16 nSAVE = 0xffff;
  if (nEnter != nSAVE) {
    nSAVE = nEnter;
    return 1;
  }
  return 0;
}

int rx_work_carrier(board_info_t * db) {
  struct net_device * dev = db -> ndev;
  unsigned nsr;
  int link;
  u16 wp[2];
  u16 rp[4]; //rp[3]&rp[4]
  static int
  try = 0;
  //ststic int ng_found = 0

  //[here!]
  //do {
  mutex_lock( & db -> addr_lock);
  #if DEF_SPIRW
  nsr = iior(db, DM9051_NSR);
  #endif
  link = !!(nsr & 0x40); //& NSR_LINKST

  if (!link &&
    try && !(
      try % 250) &&
    try <= 750)
    printk("[DM9051.carrier] nsr %02x, link= %d (try %d)\n", nsr, link,
      try);

  if (link) {
    if (db -> linkA < 3)
      db -> linkA++;
  } else {
    if (db -> linkA)
      db -> linkA--;
  }

  //db->linkBool= db->linkA ? 1 : 0;  //Rasp-save
  if (db -> linkA) {
    db -> linkBool = 1;
    try = 0; //ng_found= 0;
  } else {
    db -> linkBool = 0;
    try ++; //ng_found= 1;
  }

  if (db -> linkBool) //(netif_carrier_ok(dev))
  {
    if (dm9051_sch_cnt_chang(db -> nSCH_LINK))
      printk("[DM9051.carrier] Link Status is: %d nsr %02x [nSCH_LINK= %d. try %d]\n", link, nsr, db -> nSCH_LINK,
        try);
  } else {
    db -> nSCH_LINK++;
    if (db -> nSCH_LINK < 3)
      printk("[DM9051.carrier] Link Status is: %d\n", link); //"nsr %02x", nsr
  }

  if (netif_carrier_ok(dev) != db -> linkBool) { //Add	
    if (db -> linkBool)
      netif_carrier_on(dev); //db->nSCH_LINK= 0;
    else
      netif_carrier_off(dev);
    printk("[DM9051.phypoll] Link Status is: %d\n", link);

    //-------------------------
    //[+fifo reset] here! (tbd)
    //-------------------------
    if (db -> linkBool) { //(tbd)
      int RdForLen = DM9051_DUMP_LEN_BF_RST; //DM9051_PKT_MAX;
      char * sbuff = db -> sbuf;
      //#if DM_WRP_TO_RDP_HANDLE
      u16 rxwr_localtmp; // will equal to 'db->mdwr_reg_end'
      //#endif

      printk("\n");
      printk("----- @dm9.Connect as Cable-Plug(or Device Power-on).s -----\n");
      
      {
        int i;
          printnb_init(1); //test
          printnb("dm9_rxb_isr latest-read rxb count: %d latest-try-detecting isr [%02x], regs [%02x], rxb [%02x] (start)\n", 
            db -> bC.rxbyte_counter, db -> bC.rxbyte_isr[db -> bC.rxbyte_counter], db -> bC.rxbyte_regs[db -> bC.rxbyte_counter], db -> bC.rxbyte_pad[db -> bC.rxbyte_counter]);
          
          if (!db -> bC.rxbyte_counter) printnb("dm9_rxb_isr read rxb count: %d latest-try-detecting isr [%02x], regs [%02x], rxb [%02x]\n", 
            db -> bC.rxbyte_counter, db -> bC.rxbyte_isr[0], db -> bC.rxbyte_regs[0], db -> bC.rxbyte_pad[0]);
          else {
            printnb("dm9_rxb_isr hist list");
            for (i = 0; i < db -> bC.rxbyte_counter; i++) printnb(" %d(%02x)", i,db -> bC.rxbyte_isr[i]);
            printnb("\n");
          }
          if (!db -> bC.rxbyte_counter) printnb("dm9_rxb_isr read rxb count: %d latest-try-detecting isr [%02x], regs [%02x], rxb [%02x]\n", db -> bC.rxbyte_counter, db -> bC.rxbyte_isr[0], db -> bC.rxbyte_regs[0], db -> bC.rxbyte_pad[0]);
          else {
            printnb("dm9_rxb_reg hist list");
            for (i = 0; i < db -> bC.rxbyte_counter; i++) printnb(" %d(%02x)", i,db -> bC.rxbyte_regs[i]);
            printnb("\n");
          }
          if (!db -> bC.rxbyte_counter) printnb("dm9_rxb_isr read rxb count: %d latest-try-detecting isr [%02x], regs [%02x], rxb [%02x]\n", db -> bC.rxbyte_counter, db -> bC.rxbyte_isr[0], db -> bC.rxbyte_regs[0], db -> bC.rxbyte_pad[0]);
          else {
            printnb("dm9_rxb_dat hist list");
            for (i = 0; i < db -> bC.rxbyte_counter; i++) printnb(" %d(%02x)", i,db -> bC.rxbyte_pad[i]);
            printnb("\n");
          }
          
          if (db -> bC.rxbyte_counter) {
            for (i = 0; i < db -> bC.rxbyte_counter; i++) 
              printnb("dm9_rxb_isr read rxb counted: %d on-history-detecting isr (%02x), regs (%02x), rxb (%02x)\n", i, db -> bC.rxbyte_isr[i], db -> bC.rxbyte_regs[i], db -> bC.rxbyte_pad[i]);
              
            printnb("dm9_rxb_isr read rxb count: %d latest-try-detecting isr [%02x], regs [%02x], rxb [%02x]\n", 
              db -> bC.rxbyte_counter, db -> bC.rxbyte_isr[db ->bC.rxbyte_counter], db -> bC.rxbyte_regs[db ->bC.rxbyte_counter], db -> bC.rxbyte_pad[db ->bC.rxbyte_counter]);
          }
          printk("[dm9 code]: rx_work_carrier() dump with read rxb counted: %d\n", db -> bC.rxbyte_counter);
          dump_end_packet(db); //dump
        }

      //[select-recover-mode]
      link_recover_mode++;
      link_recover_mode = link_recover_mode % 3; //if ((link_recover _mode%3)==0) link_recover _mode = 0;
#if 1
      //[Test always 'link_recover_mode']
      link_recover_mode = 1;
#endif      
      //#if DM_WRP_TO_RDP_HANDLE
      if (link_recover_mode) {
        read_wr_of_rx(db, & rxwr_localtmp);
        wp[0] = db -> mdwr_reg_end; //printk(" [dm9 Linkup]  inblk_rxwr-s-Linkup %4x (get rxwr).s\n", db->mdwr_reg_end); //or 'rxwr_localtmp'
        wp[1] = 0;
      } else
        printk("[dm9 Linkup dump_and_then_fifo_reset].s ...\n");
      //#endif

      read_rp_of_rx(db, & db -> mdra_regs[0]); //.[printk(" [dm9 Linkup]  inblk_mrrx-s-Linkup %4x\n", db->mdra_reg_end)];	
      rp[0] = db -> mdra_reg_end; //printk(" [dm9 Linkup]  inblk_mrrx-s-Linkup %4x\n", db->mdra_reg_end);

	{
	#ifndef QCOM_BURST_MODE
	  char bf1 = sbuff[0]; //* prevPtr; //sbuff[p-1];
	#endif	  
      //.dm9051_inblk_noncpy(db, & sbuff[1], RdForLen);
	  //.dbg_inblk(db, & sbuff[1], RdForLen); //=rdptr-1 =&sbuff[p-1]
	  dm9inblk(db, & sbuff[1], RdForLen);
	  //#ifdef QCOM_RX_DWORD_BOUNDARY
	  //dm9.lagcy_inblk_ncpy_dwbnd(db, & sbuff[1], RdForLen);
	  //#else
	  //dm9.lagcy_inblk_ncpy(db, & sbuff[1], RdForLen);
	  //#endif
	#ifndef QCOM_BURST_MODE
	  sbuff[0] = bf1;
	#endif
	}

      read_rp_of_rx(db, & db -> mdra_regs[0]);
      rp[1] = db -> mdra_reg_end; //printk(" [dm9 Linkup]  inblk_mrrx-e-Linkup %4x\n", db->mdra_reg_end);
      rp[2] = 0;

      //#if DM_WRP_TO_RDP_HANDLE
      if (link_recover_mode) {
        int wpflag=list_pointers("dm9linkup wp", 1, wp, wp, 'w'); //list_pointers("dm9linkup wp"..);
        if (wpflag) printnb("[ checking mdwr_end %4x]\n", db -> mdwr_reg_end);
        list_pointers("dm9linkup rp", 2, rp, rp, 'r'); //list_pointers("dm9linkup rp"..);
        write_rp_of_rx(db, db -> mdwr_reg_end); //or 'rxwr_localtmp'   
        read_rp_of_rx(db, & db -> mdra_regs[0]);
        rp[2] = db -> mdwr_reg_end;
        rp[3] = 0; //printk(" [dm9 Linkup]  (rxwr to mrrx)-Linkup %4x (get/set rxwr).e\n", db->mdwr_reg_end); //or 'rxwr_localtmp'      
        list_pointers("dm9linkup nrp", 3, rp, rp, 'W'); //list_pointers("dm9linkup nrp"..);
        printk(" [dm9 test 0x%x]\n", DM9051_DUMP_LEN_BF_RST); //printk(" [dm9 0x04 + 0x%x]\n", len);
        printnb_rx_fifo( & sbuff[1], 4, & sbuff[5], DM9051_DUMP_LEN_BF_RST - 4); // [4] + [DM9051_DUMP_LEN_BF_RST  - 4]
        ++db -> bC.DO_RP_Update_counter;
#if 1
		printk("[dm9 code]: rx_work_carrier() dump with read rxb counted: %d\n", db -> bC.rxbyte_counter);
		dump_end_packet(db); //dump
#endif
    bcopen_rx_info_clear( & db -> bC); //UPDATE OK
    DM9051_fifo_reset_flg = 1; //UPDATE OK
        printk("[dm9] update done:");
        printk(" Update_c %d\n", db -> bC.DO_RP_Update_counter - NUMBOOTRPUPDATE); //as RST_c
        printk(" (RST_c %d)\n", db -> bC.DO_FIFO_RST_counter);
      }
      //#else                                      
      else {
        list_pointers("dm9linkup rp", 2, rp, rp, 'r'); //list_pointers("dm9linkup rp"..);
        printk(" [dm9 Tst_inblkLen 0x%x]\n", DM9051_DUMP_LEN_BF_RST);
        printnb_rx_fifo( & sbuff[1], 4, & sbuff[5], DM9051_DUMP_LEN_BF_RST - 4); // [4] + [DM9051_DUMP_LEN_BF_RST  - 4]
        dm9051_fifo_reset(11, "dump_then_reset", db);
        dm9051_fifo_reset_statistic(db);
        printk("[dm9 Linkup dump_and_then_fifo_reset].e ...\n");
        read_rp_of_rx(db, & db -> mdra_regs[0]);
#if 1
		printk("[dm9 code]: rx_work_carrier() dump with read rxb counted: %d\n", db -> bC.rxbyte_counter);
		dump_end_packet(db); //dump
#endif
        printk("[dm9] reset-done: of LInk-up\n");
        printk(" RxLenErr&MacOvrSft_Er %d, RST_c %d\n", db -> bC.ERRO_counter, db -> bC.DO_FIFO_RST_counter);
        printk(" (Update_c %d)\n", db -> bC.DO_RP_Update_counter - NUMBOOTRPUPDATE);
        printk("[dm9] mdra:\n");
        printk(" ncr-reset %4x\n", db -> mdra_reg_end);
      }
      //#endif
      printk("----- @dm9.Connect as Cable-Plug(or Device Power-on).e -----\n");
      printk("\n");
    }

    if (db -> linkBool) {
      int_reg_start(db, "[dmIMR_poll_Linkup]"); // IF exactly ncr-rst then rxp to 0xc00
      iiow(db, DM9051_RCR, db -> rcr_all); // IF exactly ncr-rst then rx enable

      //again.IF-re-check
      read_rp_of_rx(db, & db -> mdra_regs[0]);
      printk("[DM9051.phypoll] dm9 such-as-reset-done when Linkup:\n");
      printk("[DM9051.phypoll] dm9 mdra:\n");
      printk("[DM9051.phypoll]  imr-Linkup %4x\n", db -> mdra_reg_end);
      printk("[DM9051.phypoll]  rcr-Wr is %4x\n", db -> rcr_all);
    }
  }

  mutex_unlock( & db -> addr_lock);
  return link;
  //} while ((++try < 8) && !db->linkBool);
}

void rx_work_cyclekeep(board_info_t * db, int has_txrx) // link_sched_delay_work, INT_glue_sched_delay_work, and infini_sched_delay_work
{
  //struct net_device *dev = db->ndev;
  //if (!netif_carrier_ok(dev) && db->nSCH_LINK < 65500) 	//new-add
  //	dm9051_INTPschedule_isr(db, R_SCH_LINK);         	//new-add
  #ifdef DM_CONF_POLLALL_INTFLAG
  static u32 SSave_Num = 0;
  static u32 SSave_INT_B = 0;
  char * jmp_mark = "*";
  #endif

  #ifdef DM_CONF_POLLALL_INTFLAG
  if (DM9051_int_token) DM9051_int_token--;
  if (DM9051_int_token)
    dm9051_INTPschedule_isr(db, R_SCH_INT_GLUE); //again (opt-0)

  if (has_txrx)
    dm9051_INTPschedule_isr(db, R_SCH_INFINI); //again (opt-0)

  if (db -> nSCH_INT_NUm != db -> nSCH_INT_B) {
    if ((SSave_Num != db -> nSCH_INT_NUm) || (SSave_INT_B != db -> nSCH_INT_B)) {
      SSave_Num = db -> nSCH_INT_NUm;
      SSave_INT_B = db -> nSCH_INT_B;

      if (db -> nSCH_INT_NUm > (db -> nSCH_INT_B + 10)) {
        jmp_mark = "**";
        db -> nSCH_INT_NUm_A = db -> nSCH_INT = db -> nSCH_INT_B = db -> nSCH_INT_NUm;
        printk("[DM9_cyclekeep ALL-SYNC-EQUAL] INT.Num %5d(dis %5d), INT.Sch= %5d(en %d)%s\n",
          db -> nSCH_INT_NUm, db -> nSCH_INT_NUm_A, db -> nSCH_INT, db -> nSCH_INT_B, jmp_mark);
      }
    }
  }

  #elif DRV_POLL_1

  //dm9051_INTPschedule_isr(db, R_SCH_INFINI);
  //=
  // schedule_delayed_work(&db->rx_work, 0); //dm9051_rx_work

  #if defined DM_EXTREME_CPU_MODE || defined DM_LIGHT_RX //20210204	
  //(4.14.79-KT.POLL-2.2zcd.xTsklet.savecpu_5pt_JabberP.202002_nosave_20210204)
  //(lnx_dm9051_dts_Ver2.2zcd_R2_b2_savecpu5i2p_Tasklet5p_JabberP_pm_NEW2.0_extreme)
  schedule_delayed_work( & db -> rx_work, 0);
  #else //20210204	  
  #define DM_TIMER_EXPIRE1 1 //15
  #define DM_TIMER_EXPIRE2 0 //10
  #define DM_TIMER_EXPIRE3 0

  if (db -> DERFER_rwregs[RXM_WrtPTR] == db -> DERFER_rwregs1[RXM_WrtPTR])
    dm9051_INTPschedule_weight(db, DM_TIMER_EXPIRE1);
  else {
    //if ((db->DERFER_calc1>>8) < 50)
    //	schedule_delayed_work(&db->rx_work, DM_TIMER_EXPIRE2); // slow ,
    //else
    schedule_delayed_work( & db -> rx_work, DM_TIMER_EXPIRE3); // faster ,
  }
  #endif //20210204	

  #endif
}

/* ----- This is to let it do rx (ca also process xmit) ----- */
static void dm9051_mutex_dm9051(board_info_t * db) {
  int has_tx_rx = 0;
  static int dbg_first_in = 1;

  IMR_DISABLE(db);

  if (dbg_first_in) {
    dbg_first_in = 0;
  }

  rx_mutex_head(db);
  dm9051_disp_hdr_s_new(db);
  rx_mutex_tail(db);
  rx_work_carrier(db);
  #if 1
  if (netif_carrier_ok(db -> ndev)) {
    #endif
    do {
      rx_mutex_hash_table(db);

      //rx_tx_isr(db);=
      rx_mutex_head(db);  //mutex_lock( & db -> addr_lock);
      has_tx_rx = rx_tx_isr0(db); // e.g. has_tx_rx = 0;
      rx_mutex_tail(db); //mutex_unlock( & db -> addr_lock);

    } while (0);
    #if 1
  }
  #endif

  if (ISR_RE_STORE(db)) //if (IMR._ENABLE(db, 1))
    db -> nSCH_INT_B++;

  rx_mutex_head(db);
  dm9051_disp_hdr_e_new(db);
  rx_mutex_tail(db);

  rx_work_cyclekeep(db, has_tx_rx); //[CYCLE-KEEP]

  if (DM9051_fifo_reset_flg) {
    IMR_ENABLE(db, 1);
    //again
    read_rp_of_rx(db, & db -> mdra_regs[0]);
    //printk("dm9 IMR-ENABLE:\n");
    //printk("dm9 mdra:\n");
	#ifdef LINUX_VERSION_CODE_UD
	printk("dm9 KT%d.%d.%d is the number user-key-in\n", 
		(LINUX_VERSION_CODE>>16 & 0xff),(LINUX_VERSION_CODE>>8 & 0xff),(LINUX_VERSION_CODE & 0xff));
	#endif
    printk("dm9 KT%d.%d.%d-imr-reset %4x\n", 
		(LINUX_VERSION_CODE>>16 & 0xff),(LINUX_VERSION_CODE>>8 & 0xff),(LINUX_VERSION_CODE & 0xff),
		db -> mdra_reg_end);
  } else
    IMR_ENABLE(db, 1);
}

/* ----- This is to let it do xmit ----- */
static void dm9051_simple_mutex_dm9051(board_info_t * db) {
  rx_work_carrier(db);
  #if 1
  if (netif_carrier_ok(db -> ndev)) {
    #endif
    do {
      rx_mutex_hash_table(db);

      //rx_tx_isr(db);=
      rx_mutex_head(db);  //mutex_lock( & db -> addr_lock);
      rx_tx_isr0(db); // has_tx_rx = NOUSED.
      rx_mutex_tail(db); //mutex_unlock( & db -> addr_lock);

    } while (0);
    #if 1
  }
  #endif

  #ifdef DM_CONF_POLLALL_INTFLAG
  //[ASR gpio only (high trigger) raising trigger].s
  #ifdef MORE_DM9051_INT_BACK_TO_STANDBY
  //#ifdef DM_CONF_POLLALL_INTFLAG 
  if (DM9051_int_en) {
    //#endif
    mutex_lock( & db -> addr_lock);
    db -> bC.isbyte = ior(db, DM9051_ISR); // Got ISR

    if (db -> bC.isbyte & 0x01) {
      iiow(db, DM9051_ISR, db -> bC.isbyte); //~bhdbd~~ // Clear ISR status
      //;printk("--- dm9 check DM9051_INT_BACK_TO_STANDBY [%d]--- \n", db->nSCH_INT);
    }
    mutex_unlock( & db -> addr_lock);

    //#ifdef DM_CONF_POLLALL_INTFLAG 
  }
  //#endif
  #endif
  //[ASR gpio only (high trigger) raising trigger].e
  #endif
  //[added.]
  if (DM9051_fifo_reset_flg) {
    IMR_ENABLE(db, 1);
    //again
    read_rp_of_rx(db, & db -> mdra_regs[0]);
    //printk("dm9 IMR-ENABLE:\n");
    //printk("dm9 mdra:\n");
	#ifdef LINUX_VERSION_CODE_UD
	printk("dm9 KT%d.%d.%d is the number user-key-in\n", 
		(LINUX_VERSION_CODE>>16 & 0xff),(LINUX_VERSION_CODE>>8 & 0xff),(LINUX_VERSION_CODE & 0xff));
	#endif
    printk("dm9 KT%d.%d.%d-imr-reset %4x\n", 
		(LINUX_VERSION_CODE>>16 & 0xff),(LINUX_VERSION_CODE>>8 & 0xff),(LINUX_VERSION_CODE & 0xff),
		db -> mdra_reg_end);
  } else
    IMR_ENABLE(db, 1);
}

static void
dm_hash_table_work(struct work_struct * work) {
  board_info_t * db = container_of(work, board_info_t, rxctrl_work);
  db -> Enter_hash = 1;
  //dm_hash_table(db);
}

#ifdef DM_CONF_PHYPOLL

int db_phy = 0;
int nAll_run_gap = 0;

static void
dm_phy_poll(struct work_struct * w) {
  //#ifdef DM_CONF_PHYPOLL
  struct delayed_work * dw = to_delayed_work(w);
  board_info_t * db = container_of(dw, board_info_t, phy_poll);
  int a, b;

  //if.in.rx_work.procedure.s!
  #ifdef DM_CONF_POLLALL_INTFLAG
  #if defined MORE_DM9051_MUTEX && defined MORE_DM9051_MUTEX_EXT
  mutex_lock( & db -> spi_lock);
  if (!DM9051_int_en) {
    mutex_unlock( & db -> spi_lock);
    goto sched_phy;
  }
  mutex_unlock( & db -> spi_lock);
  #else
  if (!DM9051_int_en)
    goto sched_phy;
  #endif
  #else
  //if (!DM9051_int_en_OF_poll) goto sched_phy;
  #endif
  //if.in.rx_work.procedure.e!

  //debug.NOT.in_rx_work.s!
  a = (int) db -> nSCH_INT_NUm;
  b = (int) db -> nSCH_INT_B;
  if (a != (b + nAll_run_gap)) {
    nAll_run_gap = a - b; // record the diff.
  }
  db_phy++;
  //debug.NOT.in_rx_work.e!

  //dm_netdevice_carrier(db);
  dm9051_INTPschedule_isr(db, R_SCH_PHYPOLL); //extended-add

  #ifdef DM_CONF_POLLALL_INTFLAG
  sched_phy:
  #else
  //sched_phy:
  #endif
  if (netif_running(db -> ndev))
    dm_schedule_phy(db);
  //#endif
}
#endif //DM_CONF_PHYPOLL

#ifdef DM_CONF_THREAD_IRQ
static void dm9051_rx_work_proc(board_info_t * db) {
  #ifdef MORE_DM9051_MUTEX
  mutex_lock( & db -> spi_lock);
  #endif

  dm9051_mutex_dm9051(db);

  #ifdef MORE_DM9051_MUTEX
  mutex_unlock( & db -> spi_lock);
  #endif
}
#else
static void dm9051_rx_work(struct work_struct * work) { //TODO. (over-night ? result)
  struct delayed_work * dw = to_delayed_work(work);
  board_info_t * db = container_of(dw, board_info_t, rx_work);

  #ifdef MORE_DM9051_MUTEX
  mutex_lock( & db -> spi_lock);
  #endif

  dm9051_mutex_dm9051(db);

  #ifdef MORE_DM9051_MUTEX
  mutex_unlock( & db -> spi_lock);
  #endif
}
#endif

#ifdef MORE_DM9051_MUTEX
static void dm9051_phypoll_work(struct work_struct * work) {
  struct delayed_work * dw = to_delayed_work(work);
  board_info_t * db = container_of(dw, board_info_t, phypoll_work);
  mutex_lock( & db -> spi_lock);
  dm9051_simple_mutex_dm9051(db); //dm9051_mutex_dm9051(db);
  mutex_unlock( & db -> spi_lock);
}
static void dm9051_xmit_work(struct work_struct * work) {
  struct delayed_work * dw = to_delayed_work(work);
  board_info_t * db = container_of(dw, board_info_t, xmit_work);
  mutex_lock( & db -> spi_lock);
  dm9051_simple_mutex_dm9051(db); //dm9051_mutex_dm9051(db);
  mutex_unlock( & db -> spi_lock);
}
#endif //MORE_DM9051_MUTEX

void define_delay_work(board_info_t * db) {
  INIT_WORK( & db -> rxctrl_work, dm_hash_table_work);
  #ifdef DM_CONF_PHYPOLL
  INIT_DELAYED_WORK( & db -> phy_poll, dm_phy_poll);
  #endif

  #ifndef DM_CONF_THREAD_IRQ
  INIT_DELAYED_WORK( & db -> rx_work, dm9051_rx_work); //(dm9051_continue_poll); // old. 'dm9051_INTP_isr()' by "INIT_WORK"
  #endif

  #ifdef MORE_DM9051_MUTEX
  INIT_DELAYED_WORK( & db -> phypoll_work, dm9051_phypoll_work);
  INIT_DELAYED_WORK( & db -> xmit_work, dm9051_xmit_work);
  #endif
}

/*[when DM9051 stop]*/
//[../new_load/driver_ops.c]    
void sched_delay_work_cancel(board_info_t * db) {
  #ifdef DM_CONF_PHYPOLL
  cancel_delayed_work_sync( & db -> phy_poll);
  #endif

  //.flush_work(&db->rxctrl_work); /* stop any outstanding work */
  #ifndef DM_CONF_THREAD_IRQ
  cancel_delayed_work_sync( & db -> rx_work); //flush_work(&db->rx_work);
  #endif

  #ifdef MORE_DM9051_MUTEX
  cancel_delayed_work_sync( & db -> phypoll_work);
  cancel_delayed_work_sync( & db -> xmit_work);
  #endif
}

/* ops */
/* event: play a schedule starter in condition */
static netdev_tx_t
DM9051_START_XMIT(struct sk_buff * skb, struct net_device * dev) //void sta_xmit_sched_delay_work(board_info_t * db)
{
  return dm_sched_tx_via_sched_rx(skb, dev);
}

/* play with a schedule starter */
static void
dm9051_set_multicast_list_schedule(struct net_device * dev) {
  dm_sched_multi(dev);
}

#if 1
#define dm9051_spi_read_reg info9.iorb //dm9.iorb
#define dm9051_spi_write_reg info9.iowb //dm9.iowb

//[return 1 ok]
static int device_polling(board_info_t * db, u8 erre_bit, u8 expect) {
  int i;
  u8 tmp;
  for (i = 0; i < 1000; i++) {
    mdelay(1); //delay
    tmp = dm9051_spi_read_reg(db, DM9051_EPCR);
    if ((tmp & erre_bit) == expect) //ready
      break;
  }
  if (i == 1000) {
    printk("[dm9 read.write eeprom time out] on polling bit : 0x%02x (but want 0x%02x)\n", tmp, expect);
    return 0;
  }
  return 1; //OK
}

//[of spi_user.c(used by 'dm9051_ethtool_ops')]
static void dm9051_read_eeprom(board_info_t * db, int offset, u8 * to) {
  mutex_lock( & db -> addr_lock);
  dm9051_spi_write_reg(db, DM9051_EPAR, offset);
  dm9051_spi_write_reg(db, DM9051_EPCR, EPCR_ERPRR);

  //pr = 
  device_polling(db, EPCR_ERRE, 0x00); //while ( dm9051_spi_read_reg(db, DM9051_EPCR) & EPCR_ERRE) ;

  dm9051_spi_write_reg(db, DM9051_EPCR, 0x0);

  to[0] = dm9051_spi_read_reg(db, DM9051_EPDRL);
  to[1] = dm9051_spi_read_reg(db, DM9051_EPDRH);
  mutex_unlock( & db -> addr_lock);
}

/*
 * Write a word data to SROM
 */
static void dm9051_write_eeprom(board_info_t * db, int offset, u8 * data) {
  int pr;
  mutex_lock( & db -> addr_lock);
  dm9051_spi_write_reg(db, DM9051_EPAR, offset);
  dm9051_spi_write_reg(db, DM9051_EPDRH, data[1]);
  dm9051_spi_write_reg(db, DM9051_EPDRL, data[0]);
  dm9051_spi_write_reg(db, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);

  pr = device_polling(db, EPCR_ERRE, 0x00); //while ( dm9051_spi_read_reg(db, DM9051_EPCR) & EPCR_ERRE) ;

  dm9051_spi_write_reg(db, DM9051_EPCR, 0);
  if (pr) {
    printk("dm9 [write Word %d][polling OK] : %02x %02x\n", offset, data[0], data[1]);
  }
  mutex_unlock( & db -> addr_lock);

  //[my delay]
  mdelay(1); //delay
  mdelay(2); //delay
  mdelay(3); //delay
}
#endif

#ifdef DM_RELOAD_EEPROM
void
dm9051_reload_eeprom(board_info_t * db) {
  iow(db, DM9051_EPCR, 1 << 5); //EPCR_REEP= 1 << 5, EPCR_ERPRR/EPCR_WEP
  printk("dm951: reload EEPROM (Reloading)\n");
  mdelay(1); //delay (Driver needs to clear it up after the operation completes)
  iow(db, DM9051_EPCR, 0x0); /* Clear phyxcer write command */
}
#endif

void
dm9051_show_eeprom_mac(board_info_t * db) {
  int i;
  int offset = 0;
  u8 rmac[6];
  for (i = 0; i < 6; i += 2)
    dm9051_read_eeprom(db, (offset + i) / 2, & rmac[i]);
  printk("dm951: read eeprom MAC: %pM (%s)\n", rmac, "Reading");
}

void
dm9051_set_mac_ops(struct net_device * ndev, void * p) {
  board_info_t * db = netdev_priv(ndev);
  u8 * s = p;
  int offset = 0;
  //u8 rmac[6];
  int i;
  //[param check]
  printk("dm9 [write mac permanently]\n");
  printk("set param mac dm9051 %02x %02x %02x  %02x %02x %02x\n", s[0], s[1],
    s[2], s[3], s[4], s[5]);
  //[dm9]				    
  iow(db, DM9051_PAR + 0, s[0]);
  iow(db, DM9051_PAR + 1, s[1]);
  iow(db, DM9051_PAR + 2, s[2]);
  iow(db, DM9051_PAR + 3, s[3]);
  iow(db, DM9051_PAR + 4, s[4]);
  iow(db, DM9051_PAR + 5, s[5]);
  //for (i = 0; i < 6; i++)
  //ndev->dev_addr[i]= s[i];
  for (i = 0; i < 6; i++) {
    ndev -> dev_addr[i] = ior(db, DM9051_PAR + i);
  }
  //[mac reg]
  for (i = 0; i < 6; i++) {
    if (ndev -> dev_addr[i] != s[i]) {
      break;
    }
  }
  if (i != 6) {
    printk("dm9 set mac(but not as parameters) chip mac %02x %02x %02x  %02x %02x %02x [Can't write]\n", ndev -> dev_addr[0], ndev -> dev_addr[1],
      ndev -> dev_addr[2], ndev -> dev_addr[3], ndev -> dev_addr[4], ndev -> dev_addr[5]);
    return;
  }
  //[eeprom]
  printk("write eeprom mac dm9051 %02x %02x %02x  %02x %02x %02x\n", s[0], s[1], s[2], s[3], s[4], s[5]);
  for (i = 0; i < 6; i += 2)
    dm9051_write_eeprom(db, (offset + i) / 2, s + i);
  printk("[dm9 write and then read]\n");
  dm9051_show_eeprom_mac(db);
}

int
dm9051_set_mac_address(struct net_device * dev, void * p) {
  char * s = p;
  printk("dm9051_set_mac_address (%02x %02x)  %02x %02x %02x  %02x %02x %02x\n", s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]);

  dm9051_set_mac_ops(dev, s + 2);
  return eth_mac_addr(dev, p);
}

//----------------------------------------------------------------------------------------
//(customization code)
#ifdef QCOM_CONF_BOARD_YES
static irqreturn_t realtek_plug_irq_l(int irq, void * dev_id) {
  /*davicom add begin*/

  /*davicom add end*/
  return IRQ_HANDLED;
}

static irqreturn_t realtek_plug_irq_p(int irq, void * dev_id) {
  /*davicom add begin*/

  /*davicom add end*/
  return IRQ_HANDLED;
}
#endif

void Custom_Board_Init(struct spi_device * spi) {
  #ifdef QCOM_CONF_BOARD_YES
  struct pinctrl * phandle = NULL;
  unsigned int gpio_rst;
  unsigned int gpio_power;
  //////gary add interrupt for realtek end 
  phandle = devm_pinctrl_get(spi);
  if (IS_ERR_OR_NULL(phandle))
    printk("[ *dm9051 probe devm_pinctrl_get failed\n");
  else
    printk("[ *dm9051 probe devm_pinctrl_get sucess\n");
  struct pinctrl_state * turnon_reset = pinctrl_lookup_state(phandle, "dm9051_active");
  struct pinctrl_state * turnoff_reset = pinctrl_lookup_state(phandle, "dm9051_sleep");
  struct pinctrl_state * turnon_power = pinctrl_lookup_state(phandle, "lan_active");
  struct pinctrl_state * turnon_reset_8305 = pinctrl_lookup_state(phandle, "rtl8305_active");
  if (IS_ERR_OR_NULL(turnon_reset))
    printk("[ *dm9051 probe pinctrl_lookup_state reset failed\n");
  else
    printk("[ *dm9051 probe pinctrl_lookup_state reset sucess\n");

  if (IS_ERR_OR_NULL(turnon_power))
    printk("[ *dm9051 probe pinctrl_lookup_state power failed\n");
  else
    printk("[ *dm9051 probe pinctrl_lookup_state power sucess\n");
  pinctrl_select_state(phandle, turnon_power);
  //pinctrl_select_state(phandle,turnon_reset_8305);
  //msleep(100);
  // pinctrl_select_state(phandle,turnon_reset);
  msleep(100);
  // pinctrl_select_state(phandle,turnoff_reset);
  msleep(10);
  //yangguangfu add
  #endif
}

void SPI_SPI_Setup(struct board_info * db) //(struct spi_device *spi)
{
  #if DMA3_P1_MTKSETUP
  SPI_PARAM_Set(db);
  #endif

  #if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
  /* While DTS modle, Define spi max speed in the DTS file */
  #else
  #if LNX_KERNEL_v58
  db -> spidev -> max_speed_hz = DRV_MAX_SPEED_HZ;
  #else
  db -> spidev -> max_speed_hz = dm9051_spi_board_devs[0].max_speed_hz;
  #endif
  #endif
  #if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
  /* While DTS modle, Define spi max speed in the DTS file */
  #else
  db -> spidev -> mode = SPI_MODE_0;
  db -> spidev -> bits_per_word = 8;

  printk("%s Driver spi_setup()\n", CARDNAME_9051);
  if (spi_setup(db -> spidev)) {
    printk("[dm95_spi] spi_setup fail\n");
    return;
  }
  #endif
}

#if DEF_PRO
static int SubNetwork_SPI_Init(struct board_info * db, int enable) {
  //mutex_lock(&db->addr_lock);
  if (enable) {
    #if defined MTK_CONF_XY6762TEMP /* || defined QCOM_CONF_BOARD_YES */
    SPI_GPIO_SetupPwrOn(db);
    #endif
    #if 0 //DMA3_P1_MTKSETUP
    SPI_GPIO_Set(1); //mt_dm9051_pinctrl_init(db->spidev); //or, SPI_GPIO_Set(1);
    #endif
    SPI_SPI_Setup(db);
  }
  //mutex_unlock(&db->addr_lock);
  return 0;
}
#endif

//[EXTRA]
/* ----- This is essential for working buffer ----- */
static int dm9051_dbg_alloc(struct board_info * db) {
  #ifdef DM_CONF_SPI_TEST_BLKIN_SPLIT
  db -> blkin = kmalloc(DM_CONF_SPI_TEST_BLKLEN, GFP_ATOMIC);
  if (!db -> blkin)
    return -ENOMEM;
  #endif
  #ifdef DM_DBG_ENDING_PACKET
  db -> endbuf = kmalloc(DM_DBG_ENDBUF_LEN, GFP_ATOMIC); // 1522= 0x5f2= RXHDR_SIZE + 0x5ee
  if (!db -> endbuf) {
    #ifdef DM_CONF_SPI_TEST_BLKIN_SPLIT
    kfree(db -> blkin);
    #endif
    return -ENOMEM;
  }
  #endif

  #ifdef FREE_NO_DOUBLE_MEM_MAX
  db -> prebuf = kmalloc((SCAN_LEN_HALF + 1) * 1, GFP_DMA | GFP_ATOMIC);
  printk("[dm9 kmalloc(GFP_DMA | GFP_ATOMIC) start-addr 0x%x size %d]\n", (unsigned int) db -> prebuf, (SCAN_LEN_HALF + 1) * 1);
  #else
  db -> prebuf = kmalloc((SCAN_LEN_HALF + 1) * 2, GFP_DMA | GFP_ATOMIC); //or 'SCAN_LEN'
  #endif

  if (!db -> prebuf) {
    #ifdef DM_CONF_SPI_TEST_BLKIN_SPLIT
    kfree(db -> blkin);
    #endif
    #ifdef DM_DBG_ENDING_PACKET
    kfree(db -> endbuf);
    #endif
    return -ENOMEM;
  }
  
  //[[Test GFP_DMA].s]
	  db -> sbuf = kmalloc((SCAN_LEN_HALF + 1) * 1, GFP_ATOMIC);
	  if (db -> sbuf) {
		  printk("[dm9 kmalloc(GFP_ATOMIC) start-addr 0x%x size %d]\n", (unsigned int)db -> sbuf, (SCAN_LEN_HALF + 1) * 1);
		  kfree(db -> sbuf);
		  printk("[dm9 use kmalloc(GFP_DMA | GFP_ATOMIC) 0x%x] and [kfree() 0x%x]\n", (unsigned int) db -> prebuf, (unsigned int) db -> sbuf);
	}
  //[[Test GFP_DMA].e]
  
  //[db->scanmem= 0;]
  #ifdef FREE_NO_DOUBLE_MEM_MAX
  db -> sbuf = db -> prebuf;
  #else
  db -> sbuf = db -> prebuf + (SCAN_LEN_HALF + 1);
  #endif
  return 0;
}
/* ----- This is essential for working buffer ----- */
static void dm9051_dbg_free(struct board_info * db) {
  #ifdef DM_CONF_SPI_TEST_BLKIN_SPLIT
  kfree(db -> blkin);
  #endif
  #ifdef DM_DBG_ENDING_PACKET
  kfree(db -> endbuf);
  #endif
  kfree(db -> prebuf);
}

#if DEF_OPE
static void dm9051_init_dm9051(struct net_device * dev) {
  board_info_t * db = netdev_priv(dev);
  #if DEF_SPIRW
  int phy4;
  iiow(db, DM9051_GPCR, GPCR_GEP_CNTL); /* Let GPIO0 output */

  /* dm9051_reset(db); */

  /* DBG_20140407 */
  phy4 = dm9051_phy_read(dev, 0, MII_ADVERTISE);
  dm9051_phy_write(dev, 0, MII_ADVERTISE, phy4 | ADVERTISE_PAUSE_CAP); /* dm95 flow-control RX! */
  dm9051_phy_read(dev, 0, MII_ADVERTISE);

  /* Program operating register */
  iow(db, DM9051_TCR, 0); /* TX Polling clear */
  iiow(db, DM9051_BPTR, 0x3f); /* Less 3Kb, 200us */
  iiow(db, DM9051_SMCR, 0); /* Special Mode */
  /* clear TX status */
  iiow(db, DM9051_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);
  iow(db, DM9051_ISR, ISR_CLR_STATUS); /* Clear interrupt status */
  #endif
  /* Init Driver variable */
  db -> imr_all = IMR_PAR | IMR_PRM; /* "| IMR_PTM" */
  #ifdef JABBER_PACKET_SUPPORT
  db -> rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN | RCR_DIS_WATCHDOG_TIMER;
  #else
  db -> rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
  #endif

  /*
   * (Set address filter table) 
   * After.call.ndo_open
   * "kernel_call.ndo_set_multicast_list.later".
   */
  //(1)
  #if DM_CONF_APPSRC
  dm9051_fifo_reset(1, NULL, db); // 'NULL' for reset FIFO, and no increase the RST counter
  #endif
  int_reg_stop(db); //iiow(db, DM9051_IMR, IMR_PAR); //= int_reg_stop()
}

static void dm9051_open_code(struct net_device * dev, board_info_t * db) // v.s. dm9051_probe_code()
{
  /* Note: Reg 1F is not set by reset */
  iow(db, DM9051_GPR, 0); /* REG_1F bit0 activate phyxcer */
  mdelay(1); /* delay needs by DM9051 */
  /* Initialize DM9051 board */
  dm9051_reset(db);
  dm9051_init_dm9051(dev);
}
#endif

#if DEF_STO
static void dm9051_stop_code(struct net_device * dev, board_info_t * db) // v.s. dm9051_probe_code()
{
  mutex_lock( & db -> addr_lock);
  dm9051_phy_write(dev, 0, MII_BMCR, BMCR_RESET); /* PHY RESET */
  iow(db, DM9051_GPR, 0x01); /* Power-Down PHY */
  //int_reg_stop(db); //iow(db, DM9051_IMR, IMR_PAR);	/* Disable all interrupt */
  iow(db, DM9051_RCR, RCR_RX_DISABLE); /* Disable RX */
  mutex_unlock( & db -> addr_lock);
}
#endif

#if DEF_OPE
void read_intcr_print(board_info_t * db) {
  unsigned rdv = 0;
  unsigned char * int_pol;
  rdv = iior(db, DM9051_INTCR);
  int_pol = "------- active high -------";
  if (rdv & 0x01)
    int_pol = "------- active low -------";
  printk("ior[REG39H][%02x] (b0: %d)(%s)\n", rdv, rdv & 0x01, int_pol);
}
#endif

/**
 * Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device.
 */
static int
dm9051_open(struct net_device * dev) {
  #if DEF_OPE
  do {
    unsigned nsr;
    int link;
    board_info_t * db = netdev_priv(dev);
    SCH_clear("_open", db);

    #ifdef DM_CONF_POLLALL_INTFLAG
    int_get_attribute(db -> spidev, dev);
    #endif

    mutex_lock( & db -> addr_lock); //Note: must 

    printk("dm9.[dm9051_open_c].s\n");

    if (db -> chip_code_state == CCS_NUL)
      dm9051_open_code(dev, db);

    read_intcr_print(db);
    printk("dm9.[dm9051_open_c].e\n");

    #ifdef DM_CONF_POLLALL_INTFLAG
    /* (Must be after dm9051.open.code()) or later and before int_en() */
    int_get_begin(db -> spidev, dev); // (disable.irq)_insided
    #endif

    #if DM_CONF_APPSRC
    netif_carrier_off(dev); //new_add: (We add in begin for 'dm_schedule._phy' or 'dm_sched_start_rx' to detect and change to linkon.)
    #endif
    #if DM_CONF_APPSRC & DM9051_CONF_TX
    skb_queue_head_init( & db -> txq);
    netif_start_queue(dev);
    //[Init.] [Re-init compare to ptobe.] //db->tx_eq= 0; //db->tx_err= 0;
    #endif

    mutex_unlock( & db -> addr_lock);

    #if DM_CONF_APPSRC & DM9051_CONF_TX
    opening_wake_queue1(dev);
    #endif

    #ifdef DM_CONF_POLLALL_INTFLAG
    printk("[dm951_open].INT_EN.s -------\n");
    #else
    printk("[dm951_open].POLL.s -------\n");
    #endif

    #if DEF_SPIRW
    printk("[dm951_open].[before_int_reg_start (IMR %02x ) statistic nSCH_INT= %d] -------\n", /*ior*/ iior(db, DM9051_IMR), db -> nSCH_INT);
    #endif

    #ifdef DM_CONF_POLLALL_INTFLAG
    int_en(dev);
    int_reg_start(db, "[dm951_INT_open]"); //peek(imr_all)
    #else
    int_reg_start(db, "[dm951_poll_open]"); //pol_reg_start(db);
    #endif

    #if DEF_SPIRW
    iiow(db, DM9051_RCR, db -> rcr_all);
    #endif

	//... 20210418 [Read thru spi-sync before schedule start]
    nsr = iior(db, DM9051_NSR);
    link = !!(nsr & 0x40); //& NSR_LINKST
	
    #ifdef DM_CONF_POLLALL_INTFLAG
    printk("[dm951_open].INT_EN.e -------\n");
    printk("[dm951_open].INT_EN.e Link Status is: %d nsr %02x [nSCH_LINK= %d]\n", link, nsr, db -> nSCH_LINK);
    #else
    printk("[dm951_open].POLL.e -------\n");
    printk("[dm951_open].POLL.e Link Status is: %d nsr %02x [nSCH_LINK= %d]\n", link, nsr, db -> nSCH_LINK);
    #endif
    
    //... 20210418 [Schedule start while end-of-dm951_open]
    printk("[dm951_open].[END].[SCHEDULE START] -------\n");
    #if defined DM_CONF_PHYPOLL & DM_CONF_APPSRC
    //... 20210312 [Make for Not used] 20210312 Quacomm  MDM9626 Project
	//... Must use for while no traffic can maintain(sched poll) the phy's link state
    dm_schedule_phy(db);
    #endif
    #if DM_CONF_APPSRC
    dm_sched_start_rx(db); //Finally, start the delay work, to be the last calling, for you can not read/wrie dm9051 register since poling schedule work has began! 
    #endif
  } while (0);
  #endif
  return 0;
}

/**
 * dm951_net_stop - close network device
 * @dev: The device being closed.
 *
 * Called to close down a network device which has been active. Cancell any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state whilst it is not being used.
 */
static int dm9051_stop(struct net_device * dev) {
  printk("[dm951_if_stop].s ------- 02.e -------\n");
  #if DEF_STO
  do {
    board_info_t * db = netdev_priv(dev);
    #ifdef DM_CONF_POLLALL_INTFLAG
    int_dis(dev);
    mutex_lock( & db -> addr_lock);
    int_reg_stop(db);
    mutex_unlock( & db -> addr_lock);
    int_end(db -> spidev, db);
    #endif

    /* "kernel_call.ndo_set_multicast_list.first". */
    /* Then.call.ndo_stop                          */
    db -> driver_state = DS_IDLE;
    db -> chip_code_state = CCS_NUL;

    #if DM_CONF_APPSRC
    sched_delay_work_cancel(db);
    toend_stop_queue1(dev, NUM_QUEUE_TAIL);
    #endif
    //JJ-Count-on
    netif_carrier_off(dev);

    /* dm9051_shutdown(dev) */
    dm9051_stop_code(dev, db);
  } while (0);
  #endif
  return 0;
}

//{netdev_ops1.c}
static
const struct net_device_ops dm9051_netdev_ops = {
  /*
//..ndo_tx_timeout		= 	dm9000_timeout,
//..ndo_do_ioctl		= 	dm9051_ioctl,
//..ndo_set_features	= 	dm9000_set_features,
  #ifdef CONFIG_NET_POLL_CONTROLLER
  //.ndo_poll_controller= ...
  #endif
*/
  .ndo_open = dm9051_open,
  .ndo_stop = dm9051_stop,
  .ndo_start_xmit = DM9051_START_XMIT,
  .ndo_set_rx_mode = dm9051_set_multicast_list_schedule,
  #if!LNX_KERNEL_v58
  .ndo_change_mtu = eth_change_mtu,
  #endif
  .ndo_validate_addr = eth_validate_addr,
  .ndo_set_mac_address = dm9051_set_mac_address, //eth_mac_addr,	
};

//[new_load/driver.c]
/*
 * Search DM9051 board, allocate space and register it
 */
static int
dm9051_probe_ndev(struct spi_device * spi) {
  board_info_t * db = dev_get_drvdata( & spi -> dev);
  struct net_device * ndev = db -> ndev;
  ndev -> if_port = IF_PORT_100BASET;
  ndev -> netdev_ops = & dm9051_netdev_ops;
  ndev -> ethtool_ops = & dm9051_ethtool_ops;
  return 0;
}

// eth.c
void disp_mtu(struct net_device * dev) {
  printk("dm951: mtu %d\n", dev -> mtu);
}

void conf_mii(struct net_device * dev, struct board_info * db) {
  db -> mii.dev = dev;
  db -> mii.phy_id_mask = 1; //db->mii.phy_id_mask  = 0x1f;
  db -> mii.reg_num_mask = 0xf; //db->mii.reg_num_mask = 0x1f;
  db -> mii.phy_id = 1;
  db -> mii.mdio_read = dm9051_phy_read_lock;
  db -> mii.mdio_write = dm9051_phy_write_lock;
}

void control_objects_init(board_info_t * db) {
  struct net_device * ndev = db -> ndev;
  #ifdef MORE_DM9051_MUTEX
  mutex_init( & db -> spi_lock);
  #endif
  mutex_init( & db -> addr_lock);
  spin_lock_init( & db -> statelock_tx1_rx1); // used in 'dm9051' 'start' 'xmit'
  #if DM_CONF_APPSRC
  define_delay_work(db);
  #endif
  #if DM_CONF_APPSRC
  toend_stop_queue1(ndev, NUM_QUEUE_TAIL); //ending_stop_queue1(ndev);	
  #endif
  skb_queue_head_init( & db -> txq); //[Init.]
}

//spi_user.c
#if DEF_PRO
unsigned dm9051_chipid(board_info_t * db) {
  unsigned chipid = 0;
  #if DEF_SPIRW
  chipid = ior(db, DM9051_PIDL); //printk("ior %02x [DM9051_PIDL= %02x]\n", DM9051_PIDL, chipid);
  chipid |= (unsigned) ior(db, DM9051_PIDH) << 8; //printk("+ ior %02x [DM9051_PIDH <<8] = %04x\n", DM9051_PIDH, chipid);
  if (chipid == (DM9051_ID >> 16))
    return chipid;

  chipid = ior(db, DM9051_PIDL); //printk("ior %02x [DM9051_PIDL] = %02x\n", DM9051_PIDL, chipid);
  chipid |= (unsigned) ior(db, DM9051_PIDH) << 8; //printk("+ ior %02x [DM9051_PIDH <<8] = %04x\n", DM9051_PIDH, chipid);
  if (chipid == (DM9051_ID >> 16))
    return chipid;

  chipid = ior(db, DM9051_PIDL); //printk("ior %02x [DM9051_PIDL= %02x]\n", DM9051_PIDL, chipid);
  chipid |= (unsigned) ior(db, DM9051_PIDH) << 8; //printk("+ ior %02x [DM9051_PIDH <<8] = %04x\n", DM9051_PIDH, chipid);
  if (chipid == (DM9051_ID >> 16))
    return chipid;
  #endif
  return chipid;
}

int dm9051_set_mac(board_info_t * db, struct net_device * ndev) {
  #if DEF_SPIRW
  int i;
  for (i = 0; i < 6; i++)
    ndev -> dev_addr[i] = ior(db, DM9051_PAR + i);
  #endif
  printk("dm951: chip MAC: %pM (%s)\n", ndev -> dev_addr, "DBG-0");

  if (!is_valid_ether_addr(ndev -> dev_addr)) {
    #if DEF_SPIRW
    iow(db, DM9051_PAR + 0, 0x00);
    iow(db, DM9051_PAR + 1, 0x60);
    iow(db, DM9051_PAR + 2, 0x6e);
    iow(db, DM9051_PAR + 3, 0x90);
    iow(db, DM9051_PAR + 4, 0x51);
    iow(db, DM9051_PAR + 5, 0xee);
    for (i = 0; i < 6; i++)
      ndev -> dev_addr[i] = ior(db, DM9051_PAR + i);
    #endif
    printk("dm951: chip MAC: %pM (%s)\n", ndev -> dev_addr, "DBG-0.1");
    return 0; //[mac_src= "fixed2chip.0";] //["Free-Style";]
  }
  return 1;
}
#endif

static int
dm9051_probe(struct spi_device * spi) {
  const unsigned char * mac_src;

  do {
    board_info_t * db;
    struct net_device * ndev;
    unsigned chipid;
    int ret = 0;

    printnb_init(1); // 1 for print-log, 0 for no print-log 
    ndev = alloc_etherdev(sizeof(struct board_info));
    if (!ndev) {
      dev_err( & spi -> dev, "failed to alloc ethernet device\n");
      return -ENOMEM;
    }

    ndev -> mtu = 1500; // My-eth-conf
    /* setup board info structure */
    db = netdev_priv(ndev); //= to_dm9051_board(ndev)
    memset(db, 0, sizeof(board_info_t)); //[Add.] 
    db -> ndev = ndev;
    db -> spidev = spi;

    #if 1
    /* setup spi pointer field */
    SET_NETDEV_DEV(ndev, & spi -> dev);
    /*
     * No need: db->dev = &pdev->dev;            
     */
    /* setup &spi->dev pointer field */
    dev_set_drvdata( & spi -> dev, db);
    #endif

    dm9051_probe_ndev(spi); /* (ndev) */
    Custom_Board_Init(spi);
    printk("SubNetwork_SPI_Init\n");
    SubNetwork_SPI_Init(db, 1); //contain with spi->bits_per_word = 8;
    #if 1
    /* ----- This is essential for working buffer ----- */
    if (dm9051_dbg_alloc(db)) {
      ret = -ENOMEM;
      goto err_first_prepare;
    }
    #endif
    if (dm9051_spirw_begin(db)) {
      ret = -ENOMEM;
      goto err_prepare;
    }
    dm9051_spimsg_init(db);
    disp_mtu(ndev);
    conf_mii(ndev, db);
    control_objects_init(db);

    chipid = dm9051_chipid(db);
    if (chipid != (DM9051_ID >> 16)) {
      printk("Read [DM9051_PID] = %04x\n", chipid);
      printk("Read [DM9051_PID] error!\n");
      ret = -ENOMEM; //[temp as -ENOMEM]
      goto err_id;
    }
    mac_src = "eeprom2chip.0";

    #ifdef DM_RELOAD_EEPROM
    if (1) {
      int i;
      for (i = 0; i < 6; i++) {
        printk("\n");
        dm9051_reload_eeprom(db);
        dm9051_show_eeprom_mac(db);
      }
    }
    #endif
	
    if (!dm9051_set_mac(db, ndev))
      mac_src = "fixed2chip.0";

    ret = register_netdev(ndev);
    if (ret) {
      dev_err( & spi -> dev, "failed to register network device\n");
      printk("[  dm9051  ] dm9051_probe {failed to register network device}\n");
      goto err_netdev;
    }
    db -> driver_state = DS_NUL;
    db -> chip_code_state = CCS_NUL;
    printk("dm951 %s: at MAC: %pM, summary (%s)\n", //isNO_IRQ %d 
      ndev -> name, ndev -> dev_addr, mac_src); //ndev->irq,
    printk("dm951 %s: bus_num %d, spi_cs %d\n",
      ndev -> name, spi -> master -> bus_num,
      spi -> chip_select);
    printk("[dm95_spi] spi_setup db->spidev->bits_per_word= %d\n", db -> spidev -> bits_per_word);
    printk("[dm95_spi] spi_setup db->spidev->mode= %d\n", db -> spidev -> mode);
    printk("[dm95_spi] spi_setup db->spidev->db->spidev->max_speed_hz= %d\n", db -> spidev -> max_speed_hz);
    Operation_clear(db); //[In probe, this should be essential.]
    SCH_clear("_probe", db); //[In probe, this should be not essential.]
    return 0;
    err_netdev:
      err_id:
      dm9051_spirw_end(db);
    err_prepare:
      dm9051_dbg_free(db);
    err_first_prepare:
      free_netdev(ndev);
    return ret;
  } while (0);
  return 0;
}

static int
dm9051_drv_remove(struct spi_device * spi) // vs. dm9051_probe
{
  do {
    board_info_t * db = dev_get_drvdata( & spi -> dev);

    dm9051_spirw_end(db);
    #if 1
    /* ----- This is essential for working buffer ----- */
    dm9051_dbg_free(db);
	#endif
    //int_end(db->spidev, db);
    unregister_netdev(db -> ndev);
    free_netdev(db -> ndev);
  } while (0);
  return 0;
}
#endif //!DM_RX_HEAVY_MODE //(for light_rx impement files)

#if DMA3_P4_KT
/*3p*/
static int dm9051_drv_suspend(struct spi_device * spi, pm_message_t state) {
  board_info_t * db = dev_get_drvdata( & spi -> dev);
  struct net_device * ndev = db -> ndev;
  if (ndev) {
    //.	db->in_suspend = 1;
    if (!netif_running(ndev))
      return 0;

    netif_device_detach(ndev);
    dm9051_stop(ndev);
  }
  return 0;
}

static int dm9051_drv_resume(struct spi_device * spi) {
  board_info_t * db = dev_get_drvdata( & spi -> dev);
  struct net_device * ndev = db -> ndev;
  if (ndev) {
    if (netif_running(ndev)) {
      dm9051_open(ndev);
      netif_device_attach(ndev);
    }
    //.	db->in_suspend = 0;
  }
  return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
//[User must config KConfig to PM_SLEEP for the power-down function!!]
#if DMA3_P4N_KT
static int dm9051_drv_suspend(struct device * dev) //(struct spi_device *spi)  //...
{
  board_info_t * db = dev_get_drvdata(dev); //(&spi->dev);
  struct net_device * ndev = db -> ndev;
  if (ndev) {
    if (!netif_running(ndev))
      return 0;
    netif_device_detach(ndev);
    dm9051_stop(ndev);
  }
  return 0;
}
static int dm9051_drv_resume(struct device * dev) //(struct spi_device *spi)   //...
{
  board_info_t * db = dev_get_drvdata(dev); //(&spi->dev);
  struct net_device * ndev = db -> ndev;
  if (ndev) {
    if (netif_running(ndev)) {
      dm9051_open(ndev);
      netif_device_attach(ndev);
    }
  }
  return 0;
}
#endif
#endif

#if DMA3_P4N_KT
static SIMPLE_DEV_PM_OPS(dm9051_drv_pm_ops, dm9051_drv_suspend, dm9051_drv_resume);
#endif

static struct of_device_id dm9051_match_table[] = { //"davicom,dm9051" (dts_yes_driver_table.h)
  {
    .compatible = "davicom,dm9051",
  },
  {}
};

struct spi_device_id dm9051_spi_id_table = {
  "dm9051",
  0
}; //(dts_yes_driver_table.h)=DRVNAME_9051="dm9051"

static struct spi_driver dm9051_driver = {
  .driver = {
    .name = DRVNAME_9051, //="dm9051"
    .owner = THIS_MODULE,
    #if DMA3_P4N_KT
    .pm = & dm9051_drv_pm_ops,
    #endif
    .of_match_table = dm9051_match_table,
    .bus = & spi_bus_type,
  },
  .probe = dm9051_probe,
  .remove = dm9051_drv_remove,
  .id_table = & dm9051_spi_id_table,
  #if DMA3_P4_KT
  /*3p*/
  .suspend = dm9051_drv_suspend,
  /*3p*/
  .resume = dm9051_drv_resume,
  #endif
  #if DMA3_P4N_KT
  //.suspend = dm9051_drv_suspend,
  //.resume = dm9051_drv_resume,
  #endif
};

// 1----------------------------------------------------------------------------------------------------------

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
/* this no need register board information ! */
#else //DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
#if DMA3_P6_DRVSTATIC
/* Joseph 20151030 */
extern int spi_register_board_info(struct spi_board_info
  const * info, unsigned n);
#else //DMA3_P6_DRVSTATIC
/* Joseph: find/delete/new */
static unsigned verbose = 3;
module_param(verbose, uint, 0);
MODULE_PARM_DESC(verbose,
  "0 silent, >0 show gpios, >1 show devices, >2 show devices before (default=3)");

static struct spi_device * spi_device;

static void dm9051_device_spi_delete(struct spi_master * master, unsigned cs) {
  struct device * dev;
  char str[32];

  snprintf(str, sizeof(str), "%s.%u", dev_name( & master -> dev), cs);

  dev = bus_find_device_by_name( & spi_bus_type, NULL, str);
  if (dev) {
    if (verbose)
      pr_info(DRVNAME_9051 ": Deleting %s\n", str);
    device_del(dev);
  }
}
static int dm9051_spi_register_board_info(struct spi_board_info * spi, unsigned n) {
  /* Joseph_20151030: 'n' is always 1, ARRAY_SIZE(table) is 1 table-item in this design  */
  struct spi_master * master;

  master = spi_busnum_to_master(spi -> bus_num);
  if (!master) {
    pr_err(DRVNAME_9051 ":  spi_busnum_to_master(%d) returned NULL\n",
      spi -> bus_num);
    return -EINVAL;
  }
  /* make sure it's available */
  dm9051_device_spi_delete(master, spi -> chip_select);
  spi_device = spi_new_device(master, spi);
  put_device( & master -> dev);
  if (!spi_device) {
    pr_err(DRVNAME_9051 ":    spi_new_device() returned NULL\n");
    return -EPERM;
  }
  return 0;
}

static void dm9051_spi_unregister_board(void) {
  //----------------------
  //[#ifdef MODULE #endif]
  //----------------------
  if (spi_device) {
    device_del( & spi_device -> dev);
    kfree(spi_device);
  }
}
#endif //DMA3_P6_DRVSTATIC
#endif //DM_DM_CONF_RARE_PROJECTS_DTS_USAGE

void conf_spi_board(void) {
  /* ------------------------------------------------------------------------------------------- */
  /* #if DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE                                              */
  /*   dm9051_spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs)); */
  /* #else                                                                                       */
  /* #endif                                                                                      */
  /* ------------------------------------------------------------------------------------------- */

  #if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
  /* this no need register board information ! */
  #else
  #if DMA3_P6_DRVSTATIC
  spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs));
  #else
  dm9051_spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs));
  #endif
  #endif
}

void unconf_spi_board(void) {
  /* ---------------------------------------------- */
  /* #if DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE */
  /*     dm9051_spi_unregister_board();             */
  /* #else                                          */
  /* #endif                                         */
  /* ---------------------------------------------- */

  #if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
  /* this no need register board information ! */
  #else
  #if DMA3_P6_DRVSTATIC
  #else
  dm9051_spi_unregister_board();
  #endif
  #endif
}

static int __init
dm9051_init(void) {
  printk("\n");
  //printk("[dm951_insmod].s -------- 00.s --------\n");
  printk("[dm951_s_insmod].s ");
  printk("  --");
  printk("  --");
  printk("  --");
  printk("--00.e--");
  #if 0
  conf_spi_print(-);
  //;;NO 'db' here! / ;;NO 'spi' here!
  //;can not call to get chipid= dm9051_chipid(db);..
  #endif
  printk("  --");
  printk("  --");
  printk("  --\n");
  printk("%s, DRV %s\n", CARDNAME_9051, MSTR_DTS_VERSION);
  printk("%s, DRV %s\n", CARDNAME_9051, MSTR_MOD_VERSION);
  printk("%s, DRV %s\n", CARDNAME_9051, MSTR_INT_VERSION);
  printk("%s, SPI %s\n", CARDNAME_9051, RD_MODEL_VERSION);
  printk("%s, SPI %s\n", CARDNAME_9051, WR_MODEL_VERSION);

  DRV_VERSION_CONSTRUCT();
  #ifdef LINUX_VERSION_CODE_UD
  printk("%s Driver loaded, KV= %d.%d.%d is the number user-key-in\n", CARDNAME_9051,
	(LINUX_VERSION_CODE>>16 & 0xff),(LINUX_VERSION_CODE>>8 & 0xff),(LINUX_VERSION_CODE & 0xff));
  #endif
  printk("%s Driver loaded, KV= %d.%d.%d, %s\n", 
	CARDNAME_9051,
	(LINUX_VERSION_CODE>>16 & 0xff),(LINUX_VERSION_CODE>>8 & 0xff),(LINUX_VERSION_CODE & 0xff),
	DRV_VERSION);

  #ifdef MTK_CONF_YES
  printk("%s, SPI %s\n", CARDNAME_9051, MSTR_MTKDMA_VERSION);
  #endif
  printk("%s, SPI %s\n", CARDNAME_9051, MSTR_DMA_SYNC_VERSION);
  printk("%s, DRV %s\n", CARDNAME_9051, MSTR_EXTREME_VERSION);
  printk("%s, DRV %s\n", CARDNAME_9051, MSTR_LIGHT_RX_VERSION);
  #ifdef DM_LIGHT_RX
  printk("%s, DRV Scanx Number: %d\n", CARDNAME_9051, NUM_SCANX);
  #endif

  #if 0
  dm9051_hw_reset();
  #endif
  conf_spi_board();
  printk("[dm951_s_insmod].e\n");
  return spi_register_driver( & dm9051_driver);
}

static void dm9051_cleanup(void) {
  printk("dm9051_exit.s\n");
  unconf_spi_board();
  printk("[dm951_e_rmmod].s ------- 00.s -------\n");
  spi_unregister_driver( & dm9051_driver);
  printk("[dm951_e_rmmod].e ------- 00.e -------\n");
}

module_init(dm9051_init);
module_exit(dm9051_cleanup);

MODULE_AUTHOR("Joseph CHANG <joseph_chang@davicom.com.tw>");
MODULE_DESCRIPTION("Davicom DM9051 network driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:dm9051");
