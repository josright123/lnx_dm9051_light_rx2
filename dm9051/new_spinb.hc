//[spi]
#if 1
static u8 std_spi_read_reg(board_info_t * db, unsigned reg);
static void std_spi_write_reg(board_info_t * db, unsigned reg, unsigned val);
static void std_read_rx_buf_ncpy(board_info_t * db, u8 * buff, unsigned len);
static void std_read_rx_buf_ncpy_dword_boundary(board_info_t * db, u8 * buff, unsigned len);
static int std_write_tx_buf(board_info_t * db, u8 * buff, unsigned len);
static int std_write_tx_buf_dword_boundary(board_info_t * db, u8 * buff, unsigned len);
static void std_read_rx_buf_virtual(board_info_t * db, u8 * buff, unsigned len);

#define ior								info9.iorb //dm9.iorb
#define iior							info9.iorb //dm9.iorb
#define iow								info9.iowb //dm9.iowb
#define iiow							info9.iowb //dm9.iowb

//#define dm9051_inblk_noncpy			dm9.inblk_noncpy
//#define dm9051_outblk					dm9.outblk
//#define dm9051_inblk_virtual_packet	dm9.inblk_virpacket //'dm9051_inblk_virtual_packet' = '= std_read_rx_buf_virtual()'

typedef struct cb_info_t { //["new_spi_sync.c", DEF_SPICORE]
  u8( * iorb)(board_info_t * db, unsigned reg);
  void( * iowb)(board_info_t * db, unsigned reg, unsigned val);
  void( * inblk_virpacket)(board_info_t * db, u8 * buff, unsigned len);
//  void( * inblk_noncpy)(board_info_t * db, u8 * buff, unsigned len); // reserve 1 byte in the head.
//  int( * outblk)(board_info_t * db, u8 * buff, unsigned len);
  //void (*inblk_defcpy)(board_info_t *db, u8 *buff, unsigned len, bool need_read); // 1st byte is the rx data.
  
//  void( * burst_inblk_ncpy)(board_info_t * db, u8 * buff, unsigned len);
//  void( * burst_inblk_ncpy_dwbnd)(board_info_t * db, u8 * buff, unsigned len);
  void( * lagcy_inblk_ncpy)(board_info_t * db, u8 * buff, unsigned len);
  void( * lagcy_inblk_ncpy_dwbnd)(board_info_t * db, u8 * buff, unsigned len);
  
  int( * lagcy_outblk)(board_info_t * db, u8 * buff, unsigned len);
  int( * lagcy_outblk_dwbnd)(board_info_t * db, u8 * buff, unsigned len);
} cb_info;

const cb_info info9 = {
	.iorb = std_spi_read_reg,
	.iowb = std_spi_write_reg,
	.inblk_virpacket = std_read_rx_buf_virtual,
	.lagcy_inblk_ncpy = std_read_rx_buf_ncpy,
	.lagcy_inblk_ncpy_dwbnd = std_read_rx_buf_ncpy_dword_boundary,
	.lagcy_outblk = std_write_tx_buf,
	.lagcy_outblk_dwbnd = std_write_tx_buf_dword_boundary,
};

#define dm9inblk_vir	trans_dm9.inblk_vir	//for -dynamic
#define dm9inblk		trans_dm9.inblk		//
#define dm9outblk		trans_dm9.outblk	//

typedef struct cb_trans_t { 
  void( * inblk_vir)(board_info_t * db, u8 * buff, unsigned len); //for -dynamic
  void( * inblk)(board_info_t * db, u8 * buff, unsigned len);
  int( * outblk)(board_info_t * db, u8 * buff, unsigned len);
} cb_trans;

static cb_trans trans_dm9;

void callback_setup(int dma_bff) {
  #ifdef RPI_CONF_SPI_DMA_YES
  //enable_dma = dma_bff;
  //if (enable_dma) {
  //        dm9.iorb= dma_spi_read_reg;
  //        dm9.iowb= dma_spi_write_reg;
  //        dm9.inblk_defcpy= dma_read_rx_buf;  // 1st byte is the rx data.
  //        dm9.inblk_noncpy= dmaRX; // reserve 1 byte in the head. // dma_ with_ ncpy_
  //        dm9.outblk= dmaTX;
  //} else {
  //        dm9.iorb= std_spi_read_reg;
  //        dm9.iowb= std_spi_write_reg;
  //        dm9.inblk_defcpy= std_read_rx_buf;  // 1st byte is the rx data.
  //       dm9.inblk_noncpy= stdRX;
  //       dm9.outblk= stdTX;
  //}
  #else
	  
	trans_dm9.inblk_vir = info9.inblk_virpacket;
	#ifdef QCOM_RX_DWORD_BOUNDARY
	trans_dm9.inblk = info9.lagcy_inblk_ncpy_dwbnd;
	#else
	trans_dm9.inblk = info9.lagcy_inblk_ncpy;
	#endif
	
	#ifdef QCOM_TX_DWORD_BOUNDARY
	trans_dm9.outblk = info9.lagcy_outblk_dwbnd;
	#else
	trans_dm9.outblk = info9.lagcy_outblk;
	#endif
	
  /*
  dm9.iorb = std_spi_read_reg;
  dm9.iowb = std_spi_write_reg;
  
  //dm9.inblk_noncpy = stdRX;
  //dm9.outblk = stdTX;
  dm9.inblk_virpacket = std_read_rx_buf_virtual; //'virRX';
  
//  dm9.burst_inblk_ncpy= ;
//  dm9.burst_inblk_ncpy_dwbnd= ;
  dm9.lagcy_inblk_ncpy= std_read_rx_buf_ncpy;
  dm9.lagcy_inblk_ncpy_dwbnd= std_read_rx_buf_ncpy_dword_boundary;
  
  dm9.lagcy_outblk= std_write_tx_buf;
  dm9.lagcy_outblk_dwbnd= std_write_tx_buf_dword_boundary;
  */
  #endif
}

void callback_update(void) {
	trans_dm9.inblk_vir = trans_dm9.inblk;
}
#endif

//----------------------------------------------------------------------------------------
//[Usage definitions]
#define dmaXFER dma_spi_xfer_buf //Prototype

#ifdef QCOM_BURST_MODE
#define stdXFER std_spi_xfer_buf_burst //Prototype of 'std_spi_xfer_buf'
#else
#define stdXFER std_spi_xfer_buf_lagecy //Prototype of 'std_spi_xfer_buf'
#endif

//----------------------------------------------------------------------------------------
//[spinb_c]
#if DEF_SPIRW
#ifdef QCOM_BURST_MODE
// Prototype of:   std_spi_xfer_buf(db, txb, rxb, len)
static int std_spi_xfer_buf_burst(board_info_t * db, u8 * txb, u8 * rxb, unsigned len) {
  int ret = 0;
  #ifdef QCOM_BURST_MODE
  db -> spi_xfer2[0].tx_buf = txb;
  db -> spi_xfer2[0].rx_buf = NULL;
  db -> spi_xfer2[0].len = 1;
  //db->spi_xfer2[0].cs_change = 0;
  if (rxb == NULL) {
    db -> spi_xfer2[1].tx_buf = txb + 1;
    db -> spi_xfer2[1].rx_buf = NULL;
    db -> spi_xfer2[1].len = len;
  } else {
    db -> spi_xfer2[1].tx_buf = txb + 1;
    db -> spi_xfer2[1].rx_buf = rxb; //from [db->spi_xfer2[1].rx_buf = rxb+1];
    db -> spi_xfer2[1].len = len;
  }
  //db->spi_xfer2[1].cs_change = 0;
  ret = spi_sync(db -> spidev, & db -> spi_msg2);
  #endif
  if (ret < 0) {
    dbg_log("spi burst cmd %c\n", txb[0]);
    if (rxb)
      dbg_log("spi rxbuf %02x %02x\n", rxb[0], rxb[1]);
    else
      dbg_log("spi rxbuf NULL\n");
    dbg_log("spi LEN %d (db->fer->len %d)\n", len, len + 1);
    dbg_log("spi communication fail! ret=%d\n", ret);
  }
  return ret;
}
#endif
#ifndef QCOM_BURST_MODE
// Prototype of:   std_spi_xfer_buf(db, txb, rxb, len)
static int std_spi_xfer_buf_lagecy(board_info_t * db, u8 * txb, u8 * rxb, unsigned len) {
  int ret;
  db -> fer -> tx_buf = txb;
  db -> fer -> rx_buf = rxb;
  db -> fer -> len = len + 1;
  db -> fer -> cs_change = 0;
  ret = spi_sync(db -> spidev, db -> msg);
  if (ret < 0) {
    dbg_log("spi lagecy cmd %c\n", txb[0]);
    if (rxb)
      dbg_log("spi rxbuf %02x %02x\n", rxb[0], rxb[1]);
    else
      dbg_log("spi rxbuf NULL\n");
    dbg_log("spi LEN %d (db->fer->len %d)\n", len, len + 1);
    dbg_log("spi communication fail! ret=%d\n", ret);
  }
  return ret;
}
#endif
#endif

static int disp_std_spi_xfer_Reg(board_info_t * db, unsigned reg) {
  int ret = 0;
  if (reg == DM9051_PIDL || reg == DM9051_PIDH) {
    printk("dm905.MOSI.p.[%02x][..]\n", reg);
  }
  if (reg == DM9051_PIDL || reg == DM9051_PIDH) {
    printk("dm905.MISO.e.[..][%02x]\n", db -> spi_sypc_buf[1]); //'TxDatBuf'
  }
  return ret;
}

//[spinb_c]
#if DEF_SPIRW
static u8 std_spi_read_reg(board_info_t * db, unsigned reg) {
  u8 txb[2] = {
    0
  };
  u8 rxb[2] = {
    0
  };

  txb[0] = (DM_SPI_RD | reg);
  stdXFER(db, (u8 * ) txb, rxb, 1); //cb.xfer_buf_cb(db, (u8 *)txb, rxb, 1); //std_spi_xfer_buf(db, (u8 *)txb, rxb, 1); //'dm9051_spi_xfer_buf'
  #ifdef QCOM_BURST_MODE
  db -> spi_sypc_buf[1] = rxb[0];
  #else
  db -> spi_sypc_buf[1] = rxb[1]; //.std.read_reg //'TxDatBuf'
  #endif
  disp_std_spi_xfer_Reg(db, reg);
  return db -> spi_sypc_buf[1]; //return rxb[1];
}
static void std_spi_write_reg(board_info_t * db, unsigned reg, unsigned val) {
  u8 txb[2] = {
    0
  };
  //if (!enable._dma) {
  //}
  txb[0] = (DM_SPI_WR | reg);
  txb[1] = val;
  stdXFER(db, (u8 * ) txb, NULL, 1); //cb.xfer_buf_cb(db, (u8 *)txb, NULL, 1); //std_spi_xfer_buf(db, (u8 *)txb, NULL, 1); //'dm9051_spi_xfer_buf'
}
#endif

/*
#if DMA3_P2_RSEL_1024F
//#define stdRX	std_read_rx_buf_1024
#else
#ifdef QCOM_RX_DWORD_BOUNDARY
#define stdRX std_read_rx_buf_ncpy_dword_boundary
#else
#define stdRX std_read_rx_buf_ncpy
#endif
#endif
*/
static void std_read_rx_buf_ncpy(board_info_t * db, u8 * buff, unsigned len) {
  //[this is for the 0_buf application.][It's no-copy]
  u8 txb[1];
  txb[0] = DM_SPI_RD | DM_SPI_MRCMD;
  stdXFER(db, txb, buff, len);
}

//#ifdef QCOM_RX_DWORD_BOUNDARY
static void std_read_rx_buf_ncpy_dword_boundary(board_info_t * db, u8 * buff, unsigned len) {

  unsigned remain_len = len;
  unsigned offset = 0;

  #define INTNL_4N1_CODE 1

  #ifdef INTERNAL_ONEBYTE_SPI_SYNC
  #undef INTNL_4N1_CODE
  #define INTNL_4N1_CODE 0
  #endif

  #if INTNL_4N1_CODE
  unsigned pkg_len = len;
  if ((pkg_len + 1) >= 4) {
    u8 txbf[1];
    pkg_len = ((pkg_len + 1) >> 2) << 2; //[new for dword boundary]
    pkg_len--;
    //pkg_len = ((((pkg_len + 1) + 3) /4 )*4) - 4; //[new for dword boundary]
    //pkg_len  -= 1;

    //[do.here]
    txbf[0] = DM_SPI_RD | DM_SPI_MRCMD;
    //#ifdef _QCOM_BURST_MODE
    //	stdXFER(db, txbf, &buff[offset], pkg_len);
    //#else
    stdXFER(db, txbf, & buff[offset], pkg_len);
    //#endif

    remain_len -= pkg_len;
    offset += pkg_len;
  }
  #endif

  while (remain_len > 0) {
    u8 txb[2] = {
      0
    };
    u8 rxb[2] = {
      0
    };
    txb[0] = DM_SPI_MRCMD; //(DM_SPI_RD | reg);
    stdXFER(db, (u8 * ) txb, rxb, 1);
    #ifdef QCOM_BURST_MODE
    buff[offset++] = rxb[0];
    #else
    buff[++offset] = rxb[1];
    #endif
    remain_len--;
  }
}
//#endif //QCOM_RX_DWORD_BOUNDARY

/*
#if DMA3_P2_TSEL_1024F
//#define stdTX 	std_write_tx_buf_1024
#else
#ifdef QCOM_TX_DWORD_BOUNDARY
#define stdTX std_write_tx_buf_dword_boundary
#else
#define stdTX std_write_tx_buf
#endif
#endif
*/
static int std_write_tx_buf(board_info_t * db, u8 * buff, unsigned len) {
  unsigned remain_len = len;
  unsigned pkg_len, offset = 0;
  do {
    // 1 byte for cmd
    if (remain_len <= (SPI_SYNC_TRANSFER_BUF_LEN - 1)) {
      pkg_len = remain_len;
      remain_len = 0;
    } else {
      pkg_len = (SPI_SYNC_TRANSFER_BUF_LEN - 1);
      remain_len -= (SPI_SYNC_TRANSFER_BUF_LEN - 1);
    }
    db -> spi_sypc_buf[0] = DM_SPI_WR | DM_SPI_MWCMD; //if (!enable._dma) { } //'TxDatBuf'
    memcpy( & db -> spi_sypc_buf[1], buff + offset, pkg_len); //'TxDatBuf'
    offset += pkg_len;
    stdXFER(db, db -> spi_sypc_buf, NULL, pkg_len); //cb.xfer_buf_cb(db, db->TxDatBuf, NULL, pkg_len); //std_spi_xfer_buf(db, db->TxDatBuf, NULL, pkg_len); //'dm9051_spi_xfer_buf'

  } while (remain_len > 0);
  return 0;
}

//#ifdef QCOM_TX_DWORD_BOUNDARY
static int std_write_tx_buf_dword_boundary(board_info_t * db, u8 * buff, unsigned len) {
  unsigned remain_len = len;
  unsigned pkg_len, offset = 0;

  do {
    // 1 byte for cmd
    if (remain_len <= (SPI_SYNC_TRANSFER_BUF_LEN - 1)) {
      pkg_len = remain_len;
    } else {
      pkg_len = (SPI_SYNC_TRANSFER_BUF_LEN - 1);
    }

    if ((pkg_len + 1) < 4) {
      pkg_len = 1;
    } else {
      pkg_len = ((pkg_len + 1) >> 2) << 2; //[new for dword boundary]
      pkg_len--;
    }
    remain_len -= pkg_len;
    db -> spi_sypc_buf[0] = DM_SPI_WR | DM_SPI_MWCMD; //if (!enable._dma) { } //'TxDatBuf'
    memcpy( & db -> spi_sypc_buf[1], buff + offset, pkg_len); //'TxDatBuf'
    offset += pkg_len;
    stdXFER(db, db -> spi_sypc_buf, NULL, pkg_len); //cb.xfer_buf_cb(db, db->TxDatBuf, NULL, pkg_len); //std_spi_xfer_buf(db, db->TxDatBuf, NULL, pkg_len); //'dm9051_spi_xfer_buf'
  } while (remain_len > 0);
  return 0;
}
//#endif

//
//[One Time Usage definitions]
//#define 'virRX' std_read_rx_buf_virtual
//
static void std_read_rx_buf_virtual(board_info_t * db, u8 * buff, unsigned len) {
//[stdRX]
  #ifdef QCOM_BURST_MODE
  dm9inblk(db, buff, len);
  //#ifdef QCOM_RX_DWORD_BOUNDARY
  //std_read_rx_buf_ncpy_dword_boundary(db, buff, len);
  //#else
  //std_read_rx_buf_ncpy(db, buff, len);
  //#endif
  #else
  u8 bf = buff[0]; //legency
  dm9inblk(db, buff, len);
  //#ifdef QCOM_RX_DWORD_BOUNDARY
  //std_read_rx_buf_ncpy_dword_boundary(db, buff, len);
  //#else
  //std_read_rx_buf_ncpy(db, buff, len);
  //#endif
  buff[0] = bf; //legency
  #endif

  db -> rx_scan_packets++; //;[_inblk_virpacket] ;[= _virRX] ;[= _OpenFirstIn]
  if (db -> rx_scan_packets == 1) {
	callback_update(); //trans_dm9.inblk_vir = trans_dm9.inblk;
	//#ifdef QCOM_RX_DWORD_BOUNDARY
	//dm9.inblk_virpacket = std_read_rx_buf_ncpy_dword_boundary;
	//#else
    //dm9.inblk_virpacket = std_read_rx_buf_ncpy; //No more print rx_fifo packet content
	//#endif
  }	
  printk(" [dm9 1stPkt: reg_s %04x, calc_e %04x]\n", 
  	db -> mdra_reg_end, 
  	db -> mdra_reg_end + 4+len // by a calculation instead.
  	);
  printk(" [dm9 FIRST_RXHDR_SIZE: len 4 + (0x%x= %d) = 0x%x]\n", len, len, 4+len); //"bndLEN 0x%x", 4+len
  printnb_rx_fifo(db -> bC.dummy_pad_pointer, RXHDR_SIZE, //print rx_fifo packet content, for-notify-check
    db -> bC.dummy_pad_pointer + RXHDR_SIZE, len);
}


#if DMA3_P2_RSEL_1024F
/*static void std_read_rx_buf_1024(board_info_t *db, u8 *buff, unsigned len)
{
        //[this is for the 1024_buf application.(with copy operations)][It's better no-copy]
	u8 txb[1];
	int const pkt_count = (len + 1) / CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1) % CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	//.if((len + 1)>CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES){	
		txb[0] = DM_SPI_RD | DM_SPI_MRCMD;
		if (pkt_count) {
			int blkLen;
			//(1)
			blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1; // minus 1, so real all is 1024 * n
			stdXFER(db, txb, db->spi_sypc_buf, blkLen);
			memcpy(&buff[1], &db->spi_sypc_buf[1], blkLen);
	        //.printk("dm9rx_EvenPar_OvLimit(%d ... \n", blkLen);
			//(1P)
			if (remainder) {
			  //.blkLen= remainder;
			  stdXFER(db, txb, db->spi_sypc_buf, remainder);
			  memcpy(buff + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count), &db->spi_sypc_buf[1], remainder);
		//.printk("dm9rx_EvenPar_OvRemainder(%d ... \n", blkLen);
			}
			return;
		}
		//(2)	
		if (remainder) {
			//stdXFER(db, txb, db->spi_sypc_buf, remainder-1);
			//memcpy(&buff[1], &db->spi_sypc_buf[1], remainder-1);
			//note: len= remainder-1
			stdXFER(db, txb, buff, len);
		}
		return;
	//.}
}*/
#else
#endif


#if DMA3_P2_TSEL_1024F
/*static int std_write_tx_buf_1024(board_info_t *db, u8 *buff, unsigned len)
{
	int blkLen;
	int const pkt_count = (len + 1)/ CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1)% CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	unsigned offset = 0;
	
	if((len + 1)>CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES){
		//(1)
		blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1;
		db->spi_sypc_buf[0] = DM_SPI_WR | DM_SPI_MWCMD;
		memcpy(&db->spi_sypc_buf[1], &buff[offset], blkLen);
                offset += blkLen;
		stdXFER(db, db->spi_sypc_buf, NULL, blkLen);
        //.printk("dm9tx_std_EvenPar_OvLimit(%d ... \n", blkLen);

		//(2)	
		blkLen= remainder;
		memcpy(&db->spi_sypc_buf[1], &buff[offset], blkLen);
                //offset += blkLen;
		stdXFER(db, db->spi_sypc_buf, NULL, blkLen);
        //.printk("dm9tx_std_EvenPar_OvRemainder(%d ... \n", blkLen);
	} else {
		db->spi_sypc_buf[0] = DM_SPI_WR | DM_SPI_MWCMD;
		memcpy(&db->spi_sypc_buf[1], buff, len);
		stdXFER(db, db->spi_sypc_buf, NULL, len);
	}
   return 0;
}*/
#else
#endif

#if DEF_PRO
//SPI:
// do before: mutex_lock(&db->addr_lock); | (spin_lock_irqsave(&db->statelock,flags);)
// do mid: spin_unlock_irqrestore(&db->statelock,flags);, spin_lock_irqsave(&db->statelock,flags);
// do after: (spin_unlock_irqrestore(&db->statelock,flags);) | mutex_unlock(&db->addr_lock);
#define DM9051_PHY 0x40 /* PHY address 0x01 */
static int dm9051_phy_read(struct net_device * dev, int phy_reg_unused, int reg) {
  board_info_t * db = netdev_priv(dev);
  int ret;

  /* Fill the phyxcer register into REG_0C */
  iiow(db, DM9051_EPAR, DM9051_PHY | reg);
  iiow(db, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS); /* Issue phyxcer read command */

  //dm9051_msleep(db, 1);		/* Wait read complete */
  //= 
  while (ior(db, DM9051_EPCR) & EPCR_ERRE);

  iiow(db, DM9051_EPCR, 0x0); /* Clear phyxcer read command */
  /* The read data keeps on REG_0D & REG_0E */
  ret = (ior(db, DM9051_EPDRH) << 8) | ior(db, DM9051_EPDRL);
  return ret;
}

static void dm9051_phy_write(struct net_device * dev,
  int phyaddr_unused, int reg, int value) {
  board_info_t * db = netdev_priv(dev);

  printk("iowPHY[%02d %04x]\n", reg, value);
  /* Fill the phyxcer register into REG_0C */
  iow(db, DM9051_EPAR, DM9051_PHY | reg);
  /* Fill the written data into REG_0D & REG_0E */
  iiow(db, DM9051_EPDRL, value);
  iiow(db, DM9051_EPDRH, value >> 8);
  iow(db, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW); /* Issue phyxcer write command */

  //dm9051_msleep(db, 1);		/* Wait write complete */
  //= 
  while (ior(db, DM9051_EPCR) & EPCR_ERRE);

  iow(db, DM9051_EPCR, 0x0); /* Clear phyxcer write command */
}

static int dm9051_phy_read_lock(struct net_device * dev, int phy_reg_unused, int reg) {
  int val;
  board_info_t * db = netdev_priv(dev);
  mutex_lock( & db -> addr_lock);
  val = dm9051_phy_read(dev, 0, reg);
  mutex_unlock( & db -> addr_lock);
  return val;
}
static void dm9051_phy_write_lock(struct net_device * dev, int phyaddr_unused, int reg, int value) {
  board_info_t * db = netdev_priv(dev);
  mutex_lock( & db -> addr_lock);
  dm9051_phy_write(dev, 0, reg, value);
  mutex_unlock( & db -> addr_lock);
}
#endif
