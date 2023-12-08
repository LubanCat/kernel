/*
 *  linux/drivers/serial/99100.c
 *
 *  Based on drivers/serial/8250.c by Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This code is modified to support AX99100 series serial devices
 */

#include <linux/version.h>
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
#include <linux/config.h>
#endif

#if defined(CONFIG_SERIAL_99xx_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/console.h>
#include <linux/sysrq.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
#include <linux/mca.h>
#endif

#include <linux/sched.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/bitops.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include "ax99100_sp.h"
#include "ax99100_spi.h"

#include <linux/ioctl.h>
#include "ioctl.h"

#define UART99100_NR  16

static char version[] =
KERN_INFO "ASIX AX99100 PCIe Bridg to Serial Port:v" DRV_VERSION
	"    http://www.asix.com.tw\n";
	
//All transactions are with memory mapped registers
#define MEM_AXS 1

/*
 * Definitions for PCI support.
 */
#define FL_BASE_MASK			0x0007
#define FL_BASE0			0x0000
#define FL_BASE1			0x0001
#define FL_BASE2			0x0002
#define FL_BASE3			0x0003
#define FL_BASE4			0x0004
#define FL_BASE5			0x0005
#define FL_GET_BASE(x)			(x & FL_BASE_MASK)

#if 0
#define DEBUG(fmt...)	printk(KERN_ERR fmt)
#else
#define DEBUG(fmt...)	do { } while (0)
#endif

#if 0
#define MP_DBG(fmt...)	printk(KERN_ERR fmt)
#else
#define MP_DBG(fmt...)	do { } while (0)
#endif

#if 0
#define TR_DBG(fmt...)	printk(KERN_ERR fmt)
#else
#define TR_DBG(fmt...)	do { } while (0)
#endif

#if 0
#define BR_DBG(fmt...)	printk(KERN_ERR fmt)
#else
#define BR_DBG(fmt...)	do { } while (0)
#endif

#if 0
#define INIT_DBG(fmt...)	printk(KERN_ERR fmt)
#else
#define INIT_DBG(fmt...)	do { } while (0)
#endif

#if 0
#define RXDMA_DBG(fmt...)	if ( up->function_number == 0 ) printk(KERN_ERR fmt)
#else
#define RXDMA_DBG(fmt...)	do { } while (0)
#endif

#if 0
#define INT_DBG(fmt...)	printk(fmt)
#else
#define INT_DBG(fmt...)	do { } while (0)
#endif

#if 0
#define DMA_DEBUG(fmt...)	if ( up->function_number == 0 ) printk(KERN_ERR fmt)
#else
#define DMA_DEBUG(fmt...)	do { } while (0)
#endif

#if 0
#define DMATX_DEBUG(fmt...)	if ( up->function_number == 3 ) printk(KERN_ERR fmt)
#else
#define DMATX_DEBUG(fmt...)	do { } while (0)
#endif

#if 0
#define TX_DEBUG(fmt...)	if ( up->function_number == 3 ) printk(KERN_ERR fmt)
#else
#define TX_DEBUG(fmt...)	do { } while (0)
#endif

#if 0
#define TFC_DEBUG(fmt...)	printk(KERN_ERR fmt)
#else
#define TFC_DEBUG(fmt...)	do { } while (0)
#endif

#if 0
#define D_T_DEBUG(fmt...)	printk(KERN_ERR fmt)
#else
#define D_T_DEBUG(fmt...)	do { } while (0)
#endif



					
int gpio_mode = 3; //4 serial port mode

int suspend_count = 0;

int dma_rx_count = 0;

struct custom_eeprom CusEEbuffer;

/* ASUS setting */
unsigned long		TtempValue = 0;
unsigned long		GpioSetValueGroup0 = 0;
unsigned long		GpioSetValueGroup1 = 0;
unsigned long		GpioSetValueGroup2 = 0;
unsigned long		GpioSetValueGroup3 = 0;
unsigned long		SetGpioValue = 0;

struct uart_99100_port {
	struct uart_port	port;
	spinlock_t		lock_99100;			//Per port lock
	int			serialise_txdma;		//Variable to serialise the start_tx calls in dma mode
	unsigned int 		dma_tx;				//TX DMA enable or not
	unsigned int 		dma_rx;				//RX DMA enable or not
	u8			ier; 				//Interrupt Enable Register
	u8 			lcr;				//Line Control Register
	u8			mcr;				//Modem Control Register 
	u8 			acr;				//Additional Control Register
	u8			fcr;				//FIFO Control Register
	int			gier;				//Global Interrupt Enable Register
	unsigned int		capabilities;			//port capabilities
	int			rxfifotrigger;		
	int 			txfifotrigger;
	u32			dma_tx_cnt;			//Amount of data to be DMA in TX
	u32			dma_rx_cnt;			//Amount of data to be DMA in RX
	int			first_tx_dma;
	int			pre_need2recv_cnt;		
	char 	*		dma_tx_buf_v;			//Virtual Address of DMA Buffer for TX
	dma_addr_t 		dma_tx_buf_p;			//Physical Address of DMA Buffer for TX
	char 	*		dma_tx_buf_v_start;		//Virtual Address of DMA Buffer for Strat TX 
	dma_addr_t 		dma_tx_buf_p_start;		//Physical Address of DMA Buffer for Start TX
	char	*		dma_rx_buf_v;			//Virtual Address of DMA Buffer for RX
	dma_addr_t		dma_rx_buf_p;			//Physical Address of DMA Buffer for TX
	u32			part_done_recv_cnt;		//RX DMA CIRC buffer Read index
	int 			rx_dma_done_cnt;	
	int			uart_mode;			//SERIAL TYPE
	int			flow_control;			//Flow control is enabled or not
	int			flow_ctrl_type;			//Type of Flow control
	u8 			x_on;				//X-ON Character
	u8 			x_off;				//X-OFF Character
	u32 			ser_dcr_din_reg;		//Device control register
	u32 			ser_ven_reg;			//Vendor register
	struct uart_99100_port   *next_port;
	struct uart_99100_port   *prev_port;
	int 			dma_start_offset;

	int 			custom_setting;			//Custom application setup
	int			custom_baud;
	int			baud_base_clock;
	int			custom_dlm;
	int			custom_dll;
	int			custom_sampling_clock;

	int			function_number;		//PCI function number
	unsigned char __iomem	*bar5membase;			//BAR5 memory resource address
	int			ax99100_port_mode;		//AX99100_SERIAL_PORT
								//AX99100_MF_PORT

	int 			ltc2872_te485;			//LTC2872 Transceiver Setup
	int 			ltc2872_dz;
	int 			ltc2872_lb;
	int 			ltc2872_fen;
	
	u32			dma_delay_timeout;
	u32			boundary_check;
	u32			old_spssr2;
	
	struct tasklet_struct	tasklet_dma_rx;
	struct tasklet_struct	tasklet_dma_tx;
	int			k_gir; /* used in rx_kevent */
	u8			k_lsr;
	/* ASUS's application */
	u8			oriDTR;
	u8			oriCTS;
	/* 9-Bit mode setting */
	u8			mode_9bit;
	u8			nodeID_9bit;
	u8			enable_slave_9bit;
};

static struct uart_99100_port serial99100_ports[UART99100_NR];

struct uart_99100_contxt {
	int rx_dma_en;		
				//0 -I/O mode of RX
				//1 -DMA mode of RX
	int tx_dma_en;		
				//0 -I/O mode of TX 
				//1 -DMA mode of TX
	int uart_mode;		
				//AX99100_RS232_MODE
				//AX99100_RS422_MODE
				//AX99100_RS485_HALF_DUPLEX
				//AX99100_RS485_HALF_DUPLEX_ECHO
				//AX99100_RS485_FULL_DUPLEX
				//AX99100_RS485_FULL_DUPLEX_TXEN
	int en_flow_control;  
				//0 -No H/W Flow Control	 
				//1 -H/W Flow Control
	int flow_ctrl_type;	
				//AX99100_DTR_DSR_HW_FLOWCONTROL
				//AX99100_XON_XOFF_HW_FLOWCONTROL
				//AX99100_RTS_CTS_HW_FLOWCONTROL
	int rxfifotrigger;	//0-127
	int txfifotrigger;	//0-127
	int x_on;
	int x_off;

	/* 2872 parameters */
	int ltc2872_te485;	//0 -open
				//1 -enable 120Ohm TX termination
	int ltc2872_dz;		//0 -open
				//1 -enable 120Ohm RX termination 
	int ltc2872_lb;		//0 -disable loopback
				//1 -enable loopback
	int ltc2872_fen;	//0 -disable fast mode
				//1 -enable fast mode

	/* PCIE ASPM parameters */
	int pci_config_l0s;	//0 -disable L0s HW power saving
				//1 -enable L0s HW power saving
	int pci_config_l1;	//0 -disable L1 ASPM HW power saving
				//1 -enable L1 ASPM HW power saving
	/* 9-bit mode */
	u8  mode_9bit;
	u8  nodeID_9bit;
};

static struct uart_99100_contxt uart_99100_contxts[] = {
	//Port 0
	{
		.rx_dma_en	= 0,
 		.tx_dma_en	= 0,
		.uart_mode	= AX99100_RS232_MODE,
		.en_flow_control= 0,
		.flow_ctrl_type = AX99100_RTS_CTS_HW_FLOWCONTROL,
		.rxfifotrigger	= 1,
		.txfifotrigger	= 1,		
		.x_on		= SERIAL_DEF_XON,
		.x_off		= SERIAL_DEF_XOFF,

		.ltc2872_te485 	= 0,
		.ltc2872_dz 	= 0,
		.ltc2872_lb 	= 0,
		.ltc2872_fen 	= 0,
		.pci_config_l0s	= 0,
		.pci_config_l1 	= 0,
		.mode_9bit	= MODE_9BIT_DISABLE,
		.nodeID_9bit	= 0,
	},
	//Port 1
	{
		.rx_dma_en	= 0,
		.tx_dma_en	= 0,
		.uart_mode	= AX99100_RS232_MODE,
		.en_flow_control= 0,
		.flow_ctrl_type = AX99100_RTS_CTS_HW_FLOWCONTROL,
		.rxfifotrigger  = 1,
		.txfifotrigger  = 1,		
		.x_on		= SERIAL_DEF_XON,
		.x_off		= SERIAL_DEF_XOFF,

		.ltc2872_te485 	= 0,
		.ltc2872_dz 	= 0,
		.ltc2872_lb 	= 0,
		.ltc2872_fen 	= 0,
		.pci_config_l0s	= 0,
		.pci_config_l1 	= 0,
		.mode_9bit	= MODE_9BIT_DISABLE,
		.nodeID_9bit	= 0,		
	},
	//Port 2
	{
		.rx_dma_en	= 0,
		.tx_dma_en	= 0,
		.uart_mode	= AX99100_RS232_MODE,
		.en_flow_control= 0,
		.flow_ctrl_type = AX99100_RTS_CTS_HW_FLOWCONTROL,
		.rxfifotrigger	= 1,
		.txfifotrigger	= 1,		
		.x_on		= SERIAL_DEF_XON,
		.x_off		= SERIAL_DEF_XOFF,

		.ltc2872_te485 	= 0,
		.ltc2872_dz 	= 0,
		.ltc2872_lb 	= 0,
		.ltc2872_fen 	= 0,
		.pci_config_l0s	= 0,
		.pci_config_l1 	= 0,
		.mode_9bit	= MODE_9BIT_DISABLE,
		.nodeID_9bit	= 0,
	},
	//Port 3
	{
		.rx_dma_en	= 0,
		.tx_dma_en	= 0,
		.uart_mode	= AX99100_RS232_MODE,
		.en_flow_control= 0,
		.flow_ctrl_type = AX99100_RTS_CTS_HW_FLOWCONTROL,
		.rxfifotrigger	= 1,
		.txfifotrigger	= 1,		
		.x_on		= SERIAL_DEF_XON,
		.x_off		= SERIAL_DEF_XOFF,

		.ltc2872_te485 	= 0,
		.ltc2872_dz 	= 0,
		.ltc2872_lb 	= 0,
		.ltc2872_fen 	= 0,
		.pci_config_l0s	= 0,
		.pci_config_l1 	= 0,
		.mode_9bit	= MODE_9BIT_DISABLE,
		.nodeID_9bit	= 0,
	},
};

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
static const struct serial99100_config uart_config[] = {
	[PORT_UNKNOWN] = {
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16450] = {
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550] = {
		.fifo_size	= 16,
		.tx_loadsz	= 14,
	},
	[PORT_16550A] = {
		.fifo_size	= 256,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_01,
		.flags		= UART_CAP_FIFO,
	},
	[PORT_16650] = {
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},
	[PORT_16750] = {
		.fifo_size	= 64,
		.tx_loadsz	= 64,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10 |UART_FCR7_64BYTE,
		.flags		= UART_CAP_FIFO | UART_CAP_SLEEP | UART_CAP_AFE,
	},
	[PORT_16850] = {
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},
	[PORT_16C950] = {
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO,
		.flags		= UART_CAP_FIFO,
	},
	[PORT_ENHANCED]= {
		.fifo_size	= 256,
		.tx_loadsz 	= 256,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_01,
		.flags		= UART_CAP_FIFO,
	},
};


//helper function for IO type read
static _INLINE_ u8 serial_in(struct uart_99100_port *up, int offset)
{
#if MEM_AXS
        u8 tmp1;
        tmp1=readl(up->port.membase+0x280+(offset*4));
        return tmp1;
#else
	return inb(up->port.iobase + offset);
#endif
}

//helper function for IO type write
static _INLINE_ void serial_out(struct uart_99100_port *up, int offset, int value)
{
#if MEM_AXS
        writel(value, up->port.membase+0x280+(offset*4));
#else
	outb(value, up->port.iobase + offset);
#endif
}

//Helper function to write to index control register
static void serial_icr_write(struct uart_99100_port *up, int offset, int value)
{
	DEBUG("UART_LCR=0x%x\n",serial_in(up,UART_LCR));
        serial_out(up, UART_SCR, offset);
        serial_out(up, UART_ICR, value);
	serial_out(up, UART_SCR, 0x00);
}

//Helper function to read from index control register
static unsigned int serial_icr_read(struct uart_99100_port *up, int offset)
{
	unsigned int value;
	serial_icr_write(up, UART_ACR, up->acr | UART_ACR_ICRRD);
	serial_out(up, UART_SCR, offset);
	value = serial_in(up, UART_ICR);
	serial_icr_write(up, UART_ACR, up->acr);
	return value;
}

//Helper function to set the 950 mode
void setserial_ENHANC_mode(struct uart_99100_port *up)
{
	u8 lcr,efr;
	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);

	lcr=serial_in(up,UART_LCR);
	serial_out(up, UART_LCR, 0xBF);

	efr=serial_in(up,UART_EFR);
	efr |= UART_EFR_ECB;
	serial_out(up, UART_EFR,efr);

	serial_out(up, UART_LCR, lcr);	

	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
}

// Helper function to clear the FIFO
static inline void serial99100_clear_fifos(struct uart_99100_port *p)
{
	if (p->capabilities & UART_CAP_FIFO) {
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_out(p, UART_FCR, 0);
	}
}

//Helper function to set the the UART to sleep mode
static inline void serial99100_set_sleep(struct uart_99100_port *p, int sleep)
{
	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	if (p->capabilities & UART_CAP_SLEEP) {
		if (p->capabilities & UART_CAP_EFR) {
			serial_out(p, UART_LCR, 0xBF);
			serial_out(p, UART_EFR, UART_EFR_ECB);
			serial_out(p, UART_LCR, 0);
		}
		serial_out(p, UART_IER, sleep ? UART_IERX_SLEEP : 0);
		if (p->capabilities & UART_CAP_EFR) {
			serial_out(p, UART_LCR, 0xBF);
			serial_out(p, UART_EFR, 0);
			serial_out(p, UART_LCR, 0);
		}
	}
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
}

//Member function of the port operations to stop the data transfer
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,13)
static void serial99100_stop_tx(struct uart_port *port, unsigned int tty_stop)
#else
static void serial99100_stop_tx(struct uart_port *port)
#endif
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	u32	value=0;	
	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);

	if(up->dma_tx){
		up->serialise_txdma=0;
		value |= TX_DMA_STOP_BIT;
		writel(value, up->port.membase + REG_TX_DMA_STOP);
	}else{	//IO mode
		if (up->ier & UART_IER_THRI) {
			up->ier &= ~UART_IER_THRI;
			serial_out(up, UART_IER, up->ier);
		}
	}

	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
}

//Member function of the port operations to start the data transfer
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,13)
static void serial99100_start_tx(struct uart_port *port, unsigned int tty_start)
#else
static void serial99100_start_tx(struct uart_port *port)
#endif
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	struct circ_buf *xmit = &up->port.info->xmit;
#else
	struct circ_buf *xmit = &up->port.state->xmit;
#endif
	u32	length=0,len2end,txdma_status=0;
	int tail,head,tobe_transferred;
	unsigned long flags;
	
	D_T_DEBUG("In %s ---------0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
	TX_DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	
	if (up->first_tx_dma == 1) {
		txdma_status = 1;
		up->first_tx_dma = 0;
	} else
		txdma_status = readl(up->port.membase+REG_TX_DMA_STATUS);
	
	D_T_DEBUG("In %s ---------0x%x\n",__FUNCTION__,txdma_status);
	tobe_transferred=readl(up->port.membase+REG_TX_BYTES_TRANSFERRED);
	DMATX_DEBUG("In %s -------------tobe_transferred=%d--------------------------START\n",__FUNCTION__,tobe_transferred);
	
	if(up->dma_tx && ((txdma_status & 0x01) == 1) && up->serialise_txdma == 0){	  
		
		TX_DEBUG(" I WAS IN DMA OF START_TX\n");

		//CALCULATING THE AMOUNT OF DATA AVAILABLE FOR THE NEXT TRANSFER
		//AND COPYING THE DATA TO THE DMA BUFFER
		length = uart_circ_chars_pending(xmit);

		if (length == 0) {
			TX_DEBUG("In %s TX length = 0\n",__FUNCTION__);
			return;
		}

		head=xmit->head;
		tail=xmit->tail;
		len2end = CIRC_CNT_TO_END(head, tail, UART_XMIT_SIZE); //size 4096
		TX_DEBUG("In %s -------------------xmit->tail=%d, xmit->head=%d,length=%d,length2end=%d\n",__FUNCTION__,tail,head,length,len2end);			
				
		if(tail < head){
			if(length <= DMA_TX_BUFFER_SZ){
				TX_DEBUG("In %s normal circ buffer\n",__FUNCTION__);
				memcpy(up->dma_tx_buf_v_start,&xmit->buf[tail],length);  //xmit->buf + xmit->tail
				D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
				up->dma_tx_cnt = length;
			}else{
				memcpy(up->dma_tx_buf_v_start,&xmit->buf[tail],DMA_TX_BUFFER_SZ);
				D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
				up->dma_tx_cnt = DMA_TX_BUFFER_SZ;
			}
		}else{
			if(length <= DMA_TX_BUFFER_SZ){
				TX_DEBUG("In %s 2 mode circ buffer\n",__FUNCTION__);
				memcpy(up->dma_tx_buf_v_start, &xmit->buf[tail], len2end);
				memcpy(up->dma_tx_buf_v_start+len2end, xmit->buf, head);
				D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
				up->dma_tx_cnt = length;
			}else{
				if(len2end <= DMA_TX_BUFFER_SZ){
					memcpy(up->dma_tx_buf_v_start,&xmit->buf[tail],len2end);
					memcpy(up->dma_tx_buf_v_start+len2end, xmit->buf, DMA_TX_BUFFER_SZ-len2end);
					D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
					up->dma_tx_cnt = len2end;
				}else{
					memcpy(up->dma_tx_buf_v_start,&xmit->buf[tail],DMA_TX_BUFFER_SZ);
					D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
					up->dma_tx_cnt = DMA_TX_BUFFER_SZ;
				}
			}
		}		

		DMATX_DEBUG("In %s -------------xmit->tail=%d--------------------------START\n",__FUNCTION__,xmit->tail);
		xmit->tail = ((xmit->tail) + up->dma_tx_cnt) & (UART_XMIT_SIZE-1);
		DMATX_DEBUG("In %s -------------xmit->tail2=%d--------------------------START\n",__FUNCTION__,xmit->tail);

		up->serialise_txdma++;

spin_lock_irqsave(&up->lock_99100, flags);			
		//variable to serialise the DMA tx calls	
		writel(up->dma_tx_buf_p_start,up->port.membase + REG_TX_DMA_START_ADDRESS_LOW);
		writel(up->dma_tx_cnt,up->port.membase+REG_TX_DMA_LENGTH);
		writel(TX_DMA_START_BIT, up->port.membase + REG_TX_DMA_START);	
spin_unlock_irqrestore(&up->lock_99100, flags);
		
		TX_DEBUG("In %s programmed registers\n",__FUNCTION__);		
		//UPDATING THE xmit FIFO WITH THE AMOUNT OF DATA TRANSFERRED		
		txdma_status=0;
	}else{
		if (!(up->ier & UART_IER_THRI)) {
			up->ier |= UART_IER_THRI;
			serial_out(up, UART_IER, up->ier);
		}
 	}
	 
 	TX_DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
}

//Member function of the port operations to stop receiving the data
static void serial99100_stop_rx(struct uart_port *port)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	//u32	value=0;
	
	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	if(up->dma_rx){
		//value |= RX_DMA_STOP_BIT;
		//writel(value, up->port.membase + REG_RX_DMA_STOP);
	}else{
		up->ier &= ~UART_IER_RLSI;
		up->port.read_status_mask &= ~UART_LSR_DR;
		serial_out(up, UART_IER, up->ier);
	}	
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
}


//Member function of the port operations to enable modem status change interrupt
static void serial99100_enable_ms(struct uart_port *port)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];

	DEBUG("In %s --------------------------------------- START\n",__FUNCTION__);
	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

//Function to check modem statuss
static _INLINE_ void check_modem_status(struct uart_99100_port *up)
{
	u8 status;	

	DEBUG("In %s -------------------- START\n",__FUNCTION__);
	status = serial_in(up, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		up->port.icount.rng++;
	if (status & UART_MSR_DDSR)
		up->port.icount.dsr++;
	if (status & UART_MSR_DDCD)
		uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	wake_up_interruptible(&up->port.info->delta_msr_wait);
#else
	wake_up_interruptible(&up->port.state->port.delta_msr_wait);
#endif
	DEBUG("In %s -------------------- END\n",__FUNCTION__);
}

//Helper function used in ISR to receive the charecters from the UART
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
static _INLINE_ void receive_chars(struct uart_99100_port *up, u8 *status)
#else
static _INLINE_ void receive_chars(struct uart_99100_port *up, u8 *status, struct pt_regs *regs)
#endif
{
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26))
	struct tty_struct *tty = up->port.info->tty;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	struct tty_struct *tty = up->port.info->port.tty;
#else
	struct tty_struct *tty = up->port.state->port.tty;
#endif

	u8 ch,lsr = *status;
	u8 lsr_PE = 0;
	int max_count = 256;
	unsigned int flag;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	do {
		/* The following is not allowed by the tty layer and
		   unsafe. It should be fixed ASAP */
		#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
		if (unlikely(tty->flip.count >= TTY_FLIPBUF_SIZE)) {
			if (tty->low_latency) {
				spin_unlock(&up->port.lock);
				tty_flip_buffer_push(tty);
				spin_lock(&up->port.lock);
			}
			/*
			 * If this failed then we will throw away the
			 * bytes but must do so to clear interrupts
			 */
		}
		#endif
		ch = serial_in(up, UART_RX);		
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE |UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE) {
				lsr_PE = 1;
				if (up->mode_9bit == MODE_9BIT_DISABLE)
					up->port.icount.parity++;
			} else if (lsr & UART_LSR_FE)
				up->port.icount.frame++;
			if (lsr & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= up->port.read_status_mask;			

			if (lsr & UART_LSR_BI) {
				DEBUG("handling break....");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE) {
				if (up->mode_9bit == MODE_9BIT_DISABLE)
					flag = TTY_PARITY;
			} else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
		if (uart_handle_sysrq_char(&up->port, ch, regs))
			goto ignore_char;
#else
		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;
#endif
		if (up->mode_9bit == MODE_9BIT_DATA) {
			//printk("%s - D char: %x PE: %x \n", __FUNCTION__ , ch, lsr_PE);
			uart_insert_char(&up->port,
					(lsr & ~UART_LSR_PE),
					 UART_LSR_OE,
					(lsr_PE)?'1':'0',
					 flag);
		}
		if (up->mode_9bit == MODE_9BIT_SLAVE_SW) {
			//printk("%s - S char: %x PE: %x \n", __FUNCTION__ , ch, lsr_PE);
			if (lsr_PE) {
				up->enable_slave_9bit =
					(ch == up->nodeID_9bit)?1:0;
				//printk("%s - S1 char: %x PE: %x \n", __FUNCTION__ , ch, (ch == up->nodeID_9bit)?1:0);
			} else {
				if (up->enable_slave_9bit == 1) {
				//printk("%s - S0 char: %x PE: %x \n", __FUNCTION__ , ch, lsr_PE);
					uart_insert_char(&up->port, 
							lsr,
							UART_LSR_OE, ch, flag);
				}				
			}			
			
		} else if (up->mode_9bit == MODE_9BIT_MASTER) {
			//printk("%s - M char: %x Flag: %x \n", __FUNCTION__ , ch, flag);
			if (!lsr_PE) {
				//printk("%s - M1 char: %x PE: %x \n", __FUNCTION__ , ch, lsr_PE);
				uart_insert_char(&up->port, lsr,
						UART_LSR_OE, ch, flag);
			}
		} else {
			//printk("%s - char: %x PE: %x \n", __FUNCTION__ , ch, lsr_PE);
			uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);
		}
		
ignore_char:

		lsr = serial_in(up, UART_LSR);
	} while ((lsr & UART_LSR_DR) && (max_count-- > 0));	
	spin_unlock(&up->port.lock);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	tty_flip_buffer_push(tty->port);
#else
	tty_flip_buffer_push(tty);
#endif
	spin_lock(&up->port.lock);
	
	*status = lsr;
	DEBUG("In %s -------------------------------------END\n",__FUNCTION__);
}


//Helper function used in ISR to send the data to the UART
static _INLINE_ void transmit_chars(struct uart_99100_port *up)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	struct circ_buf *xmit = &up->port.info->xmit;
#else
	struct circ_buf *xmit = &up->port.state->xmit;
#endif
	int count;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,13)
		serial99100_stop_tx(&up->port, 0);
		#else
		serial99100_stop_tx(&up->port);
		#endif		
		return;
	}

	count = uart_config[up->port.type].tx_loadsz;
	DEBUG("In %s-----------up->port.type=%d,tx_loadsz=%d\n",__FUNCTION__,up->port.type,count);
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit)){
		#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,13)
		serial99100_stop_tx(&up->port, 0);
		#else
		serial99100_stop_tx(&up->port);
		#endif
	}
	DEBUG("In %s --------------------------------------2END\n",__FUNCTION__);
}

//Helper function to stop the characters transmission in DMA mode
/*
static void transmit_chars_dma_stop_done(struct uart_99100_port * up)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
        struct circ_buf *xmit = &up->port.info->xmit;
#else
        struct circ_buf *xmit = &up->port.state->xmit;
#endif

		long int transferred;
		DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
		//UPDATING THE TRANSMIT FIFO WITH THE AMOUNT OF DATA TRANSFERRED
		transferred=readl(up->port.membase+REG_TX_BYTES_TRANSFERRED);
		xmit->tail=((xmit->tail)+transferred) & (UART_XMIT_SIZE-1);
		up->port.icount.tx += transferred;
		up->serialise_txdma=0;
		
		memset(up->dma_tx_buf_v,0,DMA_TX_BUFFER_SZ);
		DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
}
*/

//Helper function to do the necessary action upon the successful completion of data transfer in DMA mode
static int transmit_chars_dma_done(struct uart_99100_port * up)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	struct circ_buf *xmit = &up->port.info->xmit;
#else
	struct circ_buf *xmit = &up->port.state->xmit;
#endif
	int length,len2end;
	
	
	D_T_DEBUG("In %s ---------0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
	DMATX_DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);

	up->port.icount.tx += up->dma_tx_cnt;			
	length = uart_circ_chars_pending(xmit); 
	DMATX_DEBUG("In %s circ_buf lenght=%d after\n",__FUNCTION__,length); 
	
	DMATX_DEBUG("In %s up->dma_tx_buf_v=0x%x ---------------------------------------\n",__FUNCTION__,(unsigned int)up->dma_tx_buf_v);
		
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)){
		up->serialise_txdma=0;			
		if (length < WAKEUP_CHARS)
			uart_write_wakeup(&up->port);
		return 0;		
	}

	//CALCULATING THE AMOUNT OF DATA AVAILABLE FOR THE NEXT TRANSFER 
	//AND COPYING THE DATA TO THE DMA BUFFER
	
	length = uart_circ_chars_pending(xmit);
	len2end = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE); 
	DMATX_DEBUG("In %s -------xmit->tail=%d, xmit->head=%d,length=%d,length2end=%d\n",__FUNCTION__,xmit->tail,xmit->head,length,len2end);
	
	if(xmit->tail < xmit->head){	
		if(length <= DMA_TX_BUFFER_SZ){
			memcpy(up->dma_tx_buf_v,&xmit->buf[xmit->tail],length);  //xmit->buf + xmit->tail
			D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
			up->dma_tx_cnt = length;
			DMATX_DEBUG("In %s Normal mode\n",__FUNCTION__); 
		}else{
			memcpy(up->dma_tx_buf_v,&xmit->buf[xmit->tail],DMA_TX_BUFFER_SZ);
			D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
			up->dma_tx_cnt = DMA_TX_BUFFER_SZ;
		}	
	}else{
		if(length <= DMA_TX_BUFFER_SZ){
			DMATX_DEBUG("In %s 2nd mode\n",__FUNCTION__); 
			memcpy(up->dma_tx_buf_v,&xmit->buf[xmit->tail],len2end);
			memcpy(up->dma_tx_buf_v+len2end,xmit->buf,xmit->head);
			D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
			up->dma_tx_cnt = length;
		}else{
			if(len2end <= DMA_TX_BUFFER_SZ){
				memcpy(up->dma_tx_buf_v,&xmit->buf[xmit->tail],len2end);
				memcpy(up->dma_tx_buf_v+len2end, xmit->buf, DMA_TX_BUFFER_SZ-len2end);
				D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
				up->dma_tx_cnt = len2end;
			}else{
				memcpy(up->dma_tx_buf_v,&xmit->buf[xmit->tail],DMA_TX_BUFFER_SZ);
				D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
				up->dma_tx_cnt = DMA_TX_BUFFER_SZ;
			}	
		}
	}
	
	//UPDATING THE xmit FIFO WITH THE AMOUNT OF DATA TRANSFERRED
	DMATX_DEBUG("In %s -------------xmit->tail=%d--------------------------START\n",__FUNCTION__,xmit->tail);
	xmit->tail = ((xmit->tail) + up->dma_tx_cnt) & (UART_XMIT_SIZE-1);
	DMATX_DEBUG("In %s -------------xmit->tail2=%d--------------------------START\n",__FUNCTION__,xmit->tail);
	

	DMATX_DEBUG("In %s length=%d\n",__FUNCTION__,length);
	
	
	//INITIATING THE NEXT TRANSFER
	writel(up->dma_tx_buf_p,up->port.membase + REG_TX_DMA_START_ADDRESS_LOW);	
	//Writing the length of data to the TX DMA Length register	
	writel(up->dma_tx_cnt,up->port.membase+REG_TX_DMA_LENGTH);
	//Start the DMA data transfer
	writel(TX_DMA_START_BIT,up->port.membase+REG_TX_DMA_START);

	D_T_DEBUG("In %s ---1-----0x%x\n",__FUNCTION__,readl(up->port.membase+REG_TX_DMA_STATUS));
	
	up->serialise_txdma=0;	
	
	// Requesting more data to send out from the TTY layer to the driver
	if (length < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);	
	
	DMATX_DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
	
	return 0;
}

//Helper function to do the necessary action upon the successful completion of data receive in DMA mode
static void receive_chars_dma_done(struct uart_99100_port * up, int iirg)
{
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26))
	struct tty_struct *tty = up->port.info->tty;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
        struct tty_struct *tty = up->port.info->port.tty;
#else
	struct tty_struct *tty = up->port.state->port.tty;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
	u8 ch;
#endif

// #if defined(__arm__) || defined(__aarch64__)
// 	u8 ch;
// #endif



	int i;		
	u32 received_bytes;
	u32 need2recv,temp_spssr2=0;

	up->k_lsr = serial_in(up, UART_LSR);
	
	
	RXDMA_DBG("In %s ---------iirg=0x%x------------------------------START\n",__FUNCTION__,iirg);
	//checking for the flip buffer size and asking to clear it upon some threshold
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
	if (unlikely(tty->flip.count >= TTY_FLIPBUF_SIZE)) {
		if (tty->low_latency) {
			spin_unlock(&up->port.lock);
			tty_flip_buffer_push(tty);
			spin_lock(&up->port.lock);
		}
	}
#endif
	need2recv=readl(up->port.membase + REG_RX_BYTES_NEED_TO_RECV);
	RXDMA_DBG("In %s --------Receive DMA Part Done need2recv=%d\n",__FUNCTION__,need2recv);
	RXDMA_DBG("In %s --------pre_need2recv_cnt=%d\n",__FUNCTION__,up->pre_need2recv_cnt);	
	
	if ( need2recv == 0 && up->pre_need2recv_cnt == 0) {
		if (iirg & SPINTR_RXDMA_COMPLETE_ISR && !(iirg & SPINTR_RXDMA_DONE))
			goto COMPLETED;
	}
	
	up->pre_need2recv_cnt = need2recv;
	
	if ((iirg & SPINTR_RXDMA_DONE || iirg & SPINTR_RXDMA_COMPLETE_ISR)){
	  
		if (up->rx_dma_done_cnt == (DMA_RX_BUFFER_SZ/DMA_RX_SZ)){
			up->rx_dma_done_cnt=0;
		}
		
		if (up->rx_dma_done_cnt == 0) 
			received_bytes=(DMA_RX_SZ -(need2recv + up->part_done_recv_cnt));
		else
			received_bytes=(DMA_RX_BUFFER_SZ -(need2recv + up->part_done_recv_cnt));		
					
		//copiying the recived bytes to the TTY layers flip buffer
		if (tty){
			
			for (i = 1; i <= received_bytes; i++){
				/* if we insert more than TTY_FLIPBUF_SIZE characters, tty layer will drop them. */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
				if(tty->flip.count >= TTY_FLIPBUF_SIZE){
					tty_flip_buffer_push(tty);
				}
#endif				
				if (uart_handle_sysrq_char(&up->port, ch))
					goto ignore_char;

				/* this doesn't actually push the data through unless tty->low_latency is set */					
				uart_insert_char(&up->port, up->k_lsr, UART_LSR_OE, up->dma_rx_buf_v[up->part_done_recv_cnt], TTY_NORMAL);
ignore_char:
				up->part_done_recv_cnt++;

				if(up->part_done_recv_cnt == DMA_RX_BUFFER_SZ)
					up->part_done_recv_cnt = 0;				
			}				

		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
		tty_flip_buffer_push(tty->port);
#else
		tty_flip_buffer_push(tty);
#endif
		up->port.icount.rx += received_bytes;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
		if (unlikely(tty->flip.count >= TTY_FLIPBUF_SIZE)) {
			if (tty->low_latency) {
				tty_flip_buffer_push(tty);
			}
		}
#endif
	}
	if (up->boundary_check == 0 && need2recv < 256) {
		up->boundary_check = 1;
		temp_spssr2=readl(up->port.membase+SP_SETTING_REG2);
		up->old_spssr2=temp_spssr2;
			
		temp_spssr2&=0x03ffffff;
		temp_spssr2|=0x03fffc00;
		writel(temp_spssr2,up->port.membase+SP_SETTING_REG2);		
	}
	
COMPLETED:
	if (iirg & SPINTR_RXDMA_COMPLETE_ISR) {
			
		up->pre_need2recv_cnt = DMA_RX_SZ;
		up->boundary_check = 0;
		
		if (up->rx_dma_done_cnt == 0) 
			up->dma_start_offset = DMA_RX_SZ;
		else 
			up->dma_start_offset = 0;
		
		//Reinitialise the DMA
		writel(up->dma_rx_buf_p + up->dma_start_offset, up->port.membase + REG_RX_DMA_START_ADDRESS_LOW);
		//writel(0,up->port.membase+REG_RX_DMA_START_ADDRESS_HIGH);
		writel(DMA_RX_SZ,up->port.membase+REG_RX_DMA_LENGTH);			
		writel(RX_DMA_START_BIT,up->port.membase+REG_RX_DMA_START);
		
		writel(up->old_spssr2,up->port.membase+SP_SETTING_REG2);			
			
		up->rx_dma_done_cnt++;
	}	
	
}



//This handles the interrupt from a port in IO mode.
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
static inline void serial99100_handle_port(struct uart_99100_port *up)
#else
static inline void serial99100_handle_port(struct uart_99100_port *up, struct pt_regs *regs)
#endif
{
	u8 status = serial_in(up, UART_LSR);

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	DEBUG("UART_LSR = %x...", status);

	if((status & UART_LSR_DR) && !up->dma_rx){
		DEBUG("RECEIVE_CHARS\n");
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
		receive_chars(up, &status);
	#else
		receive_chars(up, &status, regs);
	#endif
	
	}
	
	if ((status & UART_LSR_THRE) && !up->dma_tx){
		DEBUG("TRANSMIT_CHARS\n");
		transmit_chars(up);
	}
	
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);	
}



// This is the 99100 type serial driver's interrupt routine.

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static irqreturn_t serial99100_interrupt(int irq, void *dev_id)
#else
static irqreturn_t serial99100_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
	struct uart_99100_port *up = dev_id;
	u32 gir=0;
	u8 iir=0;
	int handled=0;	
	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	if (suspend_count == 4 && ((gpio_mode == GPIO_4MP_MODE) || (gpio_mode == GPIO_2S_2MP_MODE) 
				|| (gpio_mode == GPIO_4S_MODE)))
		gir = 0;
	else
		gir= readl(up->port.membase+ REG_GLBL_ISR);
	
	//clear glbl int
	writel(gir,up->port.membase+REG_GLBL_ICLR);	

	//if (gir == 0)
	//	return IRQ_RETVAL(0);
		
	//DMA RX
	// DEBUG("In %s ---------------------------------------DMA RX\n",__FUNCTION__);
	if (gir & SPINTR_RXDMA) {	
		DEBUG("In %s ---------------------------------------DMA RX START_line=%d\n",__FUNCTION__,__LINE__);	
		up->k_gir = gir & SPINTR_RXDMA;
		tasklet_schedule(&up->tasklet_dma_rx);
		handled=1;			
	} 
	//DMA TX
	// DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	if (gir & SPINTR_TXDMA_ISR) {
		DEBUG("In %s ---------------------------------------DMA TX START_line=%d\n",__FUNCTION__,__LINE__);
		tasklet_schedule(&up->tasklet_dma_tx);
		handled=1;
	} 
	
	//FIFO
	if (gir & SPINTR_ISR) {				
		DEBUG("In %s ---------------------------------------FIFO START_line=%d\n",__FUNCTION__,__LINE__);
		iir = serial_in(up, UART_IIR);
		if (!(iir & UART_IIR_NO_INT)) {
			DEBUG("In %s ---------------------------------------Line=%d\n",__FUNCTION__,__LINE__);			
			spin_lock(&up->port.lock);
			
			if ((iir & 0x3F) == 0x0) {
				DEBUG("In %s ---------------------------------------LINE=%d\n",__FUNCTION__,__LINE__);
				check_modem_status(up);
			} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
				DEBUG("In %s ---------------------------------------run serial99100_handle_port line=%d\n",__FUNCTION__,__LINE__);
				serial99100_handle_port(up);
				
#else
				DEBUG("In %s ---------------------------------------run serial99100_handle_port line=%d\n",__FUNCTION__,__LINE__);
				serial99100_handle_port(up, regs);
#endif	
			}
			spin_unlock(&up->port.lock);
			handled = 1;
		}		
	}

	return IRQ_RETVAL(handled);
}

//This is a port ops helper function to verify whether the transmitter is empty of not
static unsigned int serial99100_tx_empty(struct uart_port *port)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	unsigned long flags;
	unsigned int ret;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	spin_lock_irqsave(&up->lock_99100, flags);
	ret = serial_in(up, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->lock_99100, flags);
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
	
	return ret;
}

//This is a port ops helper function to find the current state of the modem control
static unsigned int serial99100_get_mctrl(struct uart_port *port)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	unsigned long flags;
	u8 status;
	unsigned int ret;

	spin_lock_irqsave(&up->lock_99100, flags);
	status = serial_in(up, UART_MSR);
	spin_unlock_irqrestore(&up->lock_99100, flags);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;	
	
	return ret;
}

//This is a port ops helper function to set the modem control lines
static void serial99100_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	u8 mcr = 0;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;
	
	mcr |= up->mcr;

	serial_out(up, UART_MCR, mcr);

	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
}

//This is a port ops helper function to control the transmission of a break signal
static void serial99100_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	unsigned long flags;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	spin_lock_irqsave(&up->lock_99100, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->lock_99100, flags);
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
}

void serial99100_serialSettingGPIO(struct uart_99100_port *up);

//This is a port ops helper function to enable the port for reception
static int serial99100_startup(struct uart_port *port)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	unsigned long flags;
	u8 fcr=0,efr=0,lcr=0,cks=0,acr=0,mcr=0;
	u32 ser_dcr_din_val=0,ser_ven_val=0, offset3c0=0, val;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	DEBUG("Device structure address id %x\n",up);
	DEBUG(" In startup port->line is %d\n",up->port.line);
	DEBUG("membase is 0x%x\n",up->port.membase);
	DEBUG("mapbase is 0x%x\n",up->port.mapbase);
	DEBUG("iobase is 0x%x\n",up->port.iobase);
	DEBUG("port.type is %d\n",up->port.type);

	DEBUG("fifo size is %d \n",uart_config[up->port.type].fifo_size);

	port->fifosize = uart_config[up->port.type].fifo_size;

	up->capabilities = uart_config[up->port.type].flags;
	up->mcr = 0;
	up->part_done_recv_cnt = 0;	
	up->rx_dma_done_cnt = 0;
	up->dma_start_offset = 0;
	up->first_tx_dma = 1;	
	up->boundary_check = 0;
	up->old_spssr2 = 0;
	up->serialise_txdma = 0;
	up->k_lsr = 0;
	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial99100_clear_fifos(up);

	/*
	 * Clear the interrupt registers.
	 */
	if(up->dma_tx || up->dma_rx)
		(void) writel(0xFF,up->port.membase+REG_GLBL_ICLR);
	
		
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);
	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->lock_99100, flags);	
	up->port.mctrl |= TIOCM_OUT2;
	serial99100_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->lock_99100, flags);


	//Rx data transfer Interrupts
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_out(up, UART_IER, up->ier);
	
	//up->ier |= UART_IER_MSI;
	//serial_out(up, UART_IER, up->ier);
	
	/* ASUS setting */
	if ((CusEEbuffer.cus_mod == CUS_ASUS) && (up->function_number > 1))
		serial99100_serialSettingGPIO(up);
	
	if(up->port.type == PORT_16550A){
		DEBUG("In %s 550EX mode\n",__FUNCTION__);
		ser_dcr_din_val=readl(up->port.membase+SP_SETTING_REG0);
		ser_dcr_din_val |= COM_550EX_MODE_EN;
		writel(ser_dcr_din_val,up->port.membase+SP_SETTING_REG0);
		DEBUG("In %s 550EX mode SP_SETTING_REG0=0x%x\n",__FUNCTION__,readl(up->port.membase+SP_SETTING_REG0));
	
		if(up->flow_control){
			DEBUG("Enabled the Auto Hardware Flowcontrol\n");
			mcr = serial_in(up,UART_MCR);
			DEBUG("In %s mcr=0x%x and up->port.mctrl=0x%x\n",__FUNCTION__,mcr,up->port.mctrl);
			
			up->mcr |= UART_MCR_AFE;
			serial99100_set_mctrl(&up->port,up->port.mctrl);	
			mcr = serial_in(up,UART_MCR);
			DEBUG("In %s mcr=0x%x and up->port.mctrl=0x%x\n",__FUNCTION__,mcr,up->port.mctrl);
		}	

		if (up->capabilities & UART_CAP_FIFO && uart_config[port->type].fifo_size > 1) {
				fcr = uart_config[up->port.type].fcr;
				serial_out(up,UART_FCR,fcr);
		}
	}

	
	if((up->port.type == PORT_ENHANCED) || (up->custom_setting == 1)){
		//Setting the Enhanced Mode Features
		setserial_ENHANC_mode(up);		

		if (up->capabilities & UART_CAP_FIFO && uart_config[port->type].fifo_size > 1) {
			fcr = uart_config[up->port.type].fcr;
			serial_out(up,UART_FCR,fcr);
		}
		/* 9-bit mode */
		if (up->mode_9bit == MODE_9BIT_MASTER 	||
		    up->mode_9bit == MODE_9BIT_SLAVE_SW ||
		    up->mode_9bit == MODE_9BIT_DATA) {
			serial_icr_write(up, UART_NMR , (NMR_NBE | NMR_NBIE));
			val=readl(up->port.membase + SP_SETTING_REG0);
			val &= 0xDFFFFFFF;
			val |= 0x20000000;
			writel(val, up->port.membase + SP_SETTING_REG0);
		} else if (up->mode_9bit == MODE_9BIT_SLAVE_HW) {
			serial_icr_write(up, UART_NMR , (NMR_NBE | NMR_NBIE | NMR_AME));
			val=readl(up->port.membase + SP_SETTING_REG0);
			val &= 0xDFFFFFFF;
			val |= 0x20000000;
			writel(val, up->port.membase + SP_SETTING_REG0);
		} else {
			serial_icr_write(up, UART_NMR , 0);
		}

		switch(up->uart_mode){

			case AX99100_RS485_HALF_DUPLEX:
				//Commset Registers Offset 0
				//0x0008 0000  -19thBit -1 SW RS485 enable
				//0x0000 0000  -17thBit -0 SW FD enable RS485
				//0x0000 2000  -13thBit -0 RS485 RTS enable
 				//0x0000 0000  -14thBit -0 SW RTS enable

				DEBUG("TranceiverMode AX99100_RS485_MODE - RS485_HALF_DUPLEX\n");
				ser_dcr_din_val = readl(up->port.membase+SP_SETTING_REG0);
				ser_dcr_din_val &= 0xfff00fff;
				ser_dcr_din_val |= 0x00080000;

				//Commset Registers Offset 1
				//0xff00 0000  -[31-24] ff
				ser_ven_val = readl(up->port.membase+SP_SETTING_REG1);
				ser_ven_val &= 0x00ffffff;
				ser_ven_val |= 0x00000000;

				//CKS - 0x00
				//serialAX99100_XON_XOFF_HW_FLOWCONTROL_icr_write(up,UART_CKS,0x00);
				cks=serial_icr_read(up,UART_CKS);
				cks |= 0x00;

				//ACR - [4:3]-10 -> 0x10, 
				acr = 0x18;
				break;
				
			case AX99100_RS485_HALF_DUPLEX_ECHO:
				//Commset Registers Offset 0
				//0x0008 0000  -19thBit -1 SW RS485 enable
				//0x0004 0000  -18thBit -0 SW ECHO RS485 enable
				//0x0000 0000  -17thBit -0 SW FD enable RS485
				//0x0000 2000  -13thBit -0 RS485 RTS enable
				//0x0000 4000  -14thBit -1 SW RTS enable
				DEBUG("TranceiverMode AX99100_RS485_MODE - RS485_HALF_DUPLEX_ECHO\n");
				ser_dcr_din_val = readl(up->port.membase+SP_SETTING_REG0);
				ser_dcr_din_val &= 0xfff00fff;
				ser_dcr_din_val |= 0x00084000;

				//Commset Registers Offset 1
				//0xff00 0000  -[31-24] ff
				ser_ven_val = readl(up->port.membase+SP_SETTING_REG1);
				ser_ven_val &= 0x00ffffff;
				ser_ven_val |= 0xff000000;

				//CKS - 0x00
				cks = serial_icr_read(up,UART_CKS);
				cks |= 0x00;

				//ACR - [4:3]-11 -> 0x18, 
				acr = 0x18;
				break;

			case AX99100_RS485_FULL_DUPLEX_TXEN:
				//Commset Registers Offset 0
				//0x0008 0000  -19thBit -1 SW RS485 enable
				//0x0002 0000  -17thBit -1 SW FD enable RS485
				//0x0000 0000  -13thBit -0 RS485 RTS enable(should be 0)
				//0x0000 4000  -14thBit -0 SW RTS enable
				DEBUG("TranceiverMode AX99100_RS485_MODE - RS485_FULL_DUPLEX_TXEN\n");
				ser_dcr_din_val = readl(up->port.membase+SP_SETTING_REG0);
				ser_dcr_din_val &= 0xfff00fff;
				ser_dcr_din_val |= 0x000A0000;

				//TXEN always on
				ser_dcr_din_val |= 0x00210000;

				//Commset Registers Offset 1
				//0xff00 0000  -[31-24] ff
				ser_ven_val = readl(up->port.membase+SP_SETTING_REG1);
				ser_ven_val &= 0x00ffffff;
				ser_ven_val |= 0xff000000;

				//CKS - 0x00
				cks = serial_icr_read(up,UART_CKS);
				cks |= 0x00;

				//ACR - 0x10, 
				acr = 0x10;
				break;
			case AX99100_RS485_FULL_DUPLEX:
			case AX99100_RS422_MODE:

				//Commset Registers Offset 0
				//0x0008 0000  -19thBit -1 SW RS485 enable
				//0x0002 0000  -17thBit -1 SW FD enable RS485
				//0x0000 0000  -13thBit -0 RS485 RTS enable(should be 0)
				//0x0000 4000  -14thBit -0 SW RTS enable
				DEBUG("TranceiverMode AX99100_RS485_MODE - RS485_FULL_DUPLEX\n");
				ser_dcr_din_val = readl(up->port.membase+SP_SETTING_REG0);
				ser_dcr_din_val &= 0xfff00fff;
				ser_dcr_din_val |= 0x000A0000;			  

				//Commset Registers Offset 1
				//0xff00 0000  -[31-24] ff
				ser_ven_val = readl(up->port.membase+SP_SETTING_REG1);
				ser_ven_val &= 0x00ffffff;
				ser_ven_val |= 0xff000000;

				//CKS - 0x00
				cks = serial_icr_read(up,UART_CKS);
				cks |= 0x00;

				//ACR - 0x10, 
				acr = 0x18;
				break;
			default:
				DEBUG("Tranceiver RS_232 mode\n");
				up->port.mctrl &= ~TIOCM_DTR;
				up->port.mctrl &= ~TIOCM_RTS;
				serial99100_set_mctrl(&up->port, up->port.mctrl);
				break;
		}

		if(up->uart_mode > 0){	
			up->acr = up->acr|acr;

			writel(ser_dcr_din_val,up->port.membase+SP_SETTING_REG0);
			writel(ser_ven_val,up->port.membase+SP_SETTING_REG1);
			serial_icr_write(up,UART_CKS,cks);
			serial_icr_write(up,UART_ACR,up->acr);
			
			DEBUG("SP_SETTING_REG0=0x%x   SP_SETTING_REG1=0x%x   UART_CKS=0x%x   UART_ACR=0x%x\n",
				readl(up->port.membase+SP_SETTING_REG0),readl(up->port.membase+SP_SETTING_REG1),cks,up->acr);
		}

		//Setting the trigger Levels
		serial_icr_write(up,UART_RTL,up->rxfifotrigger);
		serial_icr_write(up,UART_TTL,up->txfifotrigger);
		up->acr |= UART_ACR_TLENB;
		serial_icr_write(up,UART_ACR,up->acr);

		//If Hardware Flow Control is to be enabled. The RTS/CTS, DTR/DSR is possible only in 232 mode. 
		if(up->flow_control && up->uart_mode == AX99100_RS232_MODE){
			
			//Setting the auto hardware flow control trigger levels
			serial_icr_write(up,UART_FCL,16);
			serial_icr_write(up,UART_FCH,240);

			//Setting the hw Flow control
			switch(up->flow_ctrl_type){
				case AX99100_DTR_DSR_HW_FLOWCONTROL:
					if(up->uart_mode == AX99100_RS232_MODE){
						DEBUG("H/W Flow Control AX99100_DTR_DSR_HW_FLOWCONTROL enabled\n");
						DEBUG("UART_ACR=0x%x and up->acr=0x%x\n",serial_icr_read(up,UART_ACR),up->acr);
						up->acr |= 0x0C;
						serial_icr_write(up, UART_ACR, up->acr);
						up->port.mctrl |= TIOCM_DTR;
						serial99100_set_mctrl(&up->port, up->port.mctrl);
						break;
					}else{
						DEBUG("No flow control enabled\n");
						break;
					}			
					
				case AX99100_XON_XOFF_HW_FLOWCONTROL:
					DEBUG("Enabled HwFlowControl AX99100_XON_XOFF_HW_FLOWCONTROL\n");
					if (up->mode_9bit != MODE_9BIT_SLAVE_HW) {
						lcr = serial_in(up,UART_LCR);

						serial_out(up,UART_LCR,0xBF);

						efr=serial_in(up,UART_EFR);
						efr |= 0x1A;
						serial_out(up,UART_EFR,efr);
						serial_out(up,UART_XON1,up->x_on);
						serial_out(up,UART_XOFF1,up->x_off);
						serial_out(up,UART_XON2,up->x_on);
						serial_out(up,UART_XOFF2,up->x_off);
						serial_out(up,UART_EFR,efr);
						serial_out(up,UART_LCR,lcr);
					}
					break;

				case AX99100_RTS_CTS_HW_FLOWCONTROL:
				default:
					if(up->uart_mode == AX99100_RS232_MODE){
						DEBUG("H/W Flow Control AX99100_RTS_CTS_HW_FLOWCONTROL enabled\n");
						lcr = serial_in(up,UART_LCR);
						serial_out(up,UART_LCR,0xBF);	
						efr=serial_in(up,UART_EFR);
						efr |= 0xD0;
						serial_out(up,UART_EFR,efr);									
						serial_out(up,UART_LCR,lcr);
						
						up->port.mctrl |= TIOCM_RTS;
						serial99100_set_mctrl(&up->port, up->port.mctrl);
						break;	
					}else{
						DEBUG("No H/W flow control enabled\n");
					}

			}
		}
		/* 9-bit mode - slave HW*/
		if (up->mode_9bit == MODE_9BIT_SLAVE_HW) {
			lcr = serial_in(up,UART_LCR);

			serial_out(up,UART_LCR,0xBF);

			efr=serial_in(up,UART_EFR);
			//efr |= 0x1A;
			//serial_out(up,UART_EFR,efr);
			//serial_out(up,UART_XON1,up->nodeID_9bit);
			//serial_out(up,UART_XOFF1,0x31);
			//serial_out(up,UART_XON2,0x32);
			serial_out(up,UART_XOFF2,up->nodeID_9bit);
			serial_out(up,UART_EFR,efr);
			serial_out(up,UART_LCR,lcr);
		}		

		// 2872 setup
		if (up->ax99100_port_mode == AX99100_MF_PORT) {

			offset3c0 = readl(up->bar5membase + 0x3C0);

			MP_DBG(KERN_ERR"   offset3c0 = 0x%x\n", offset3c0);

			if (up->function_number <= 1) { // function 0 or 1

				// LB
				if (up->ltc2872_lb == 1)
					offset3c0 |= (1 << 5);
				else
					offset3c0 &= ~(1 << 5);

				// H/F
				if (up->uart_mode == AX99100_RS485_HALF_DUPLEX || up->uart_mode == AX99100_RS485_HALF_DUPLEX_ECHO)
					offset3c0 |= (1 << 4);
				else
					offset3c0 &= ~(1 << 4);				

				// FEN
				if (up->ltc2872_fen == 1)
					offset3c0 |= (1 << 3);
				else
					offset3c0 &= ~(1 << 3);

			} else { // function 2 or 3
				// LB
				if (up->ltc2872_lb == 1)
					offset3c0 |= (1 << 2);
				else
					offset3c0 &= ~(1 << 2);

				// H/F
				if (up->uart_mode == AX99100_RS485_HALF_DUPLEX || up->uart_mode == AX99100_RS485_HALF_DUPLEX_ECHO)
					offset3c0 |= (1 << 1);
				else
					offset3c0 &= ~(1 << 1);				

				// FEN
				if (up->ltc2872_fen == 1)
					offset3c0 |= (1 << 0);
				else
					offset3c0 &= ~(1 << 0);
			}

			writel(offset3c0 , up->bar5membase + 0x3C0);

			MP_DBG(KERN_ERR"   set offset3c0 = 0x%x\n", offset3c0);
			MP_DBG(KERN_ERR"   re-read offset3c0 = 0x%x\n", readl(up->bar5membase + 0x3C0));

			ser_dcr_din_val = readl(up->port.membase + SP_SETTING_REG0);

			MP_DBG(KERN_ERR"   read set ser_dcr_din_val = 0x%x\n", ser_dcr_din_val);

			// TE485
			if (up->ltc2872_te485 == 1)
				ser_dcr_din_val |= (1 << 30);
			else
				ser_dcr_din_val &= ~(1 << 30);

			// DZ
			if (up->ltc2872_dz == 1)
				ser_dcr_din_val |= (((u32)1) << 31);
			else
				ser_dcr_din_val &= ~(((u32)1) << 31);

			// 485/232
			if (up->uart_mode == AX99100_RS232_MODE) {	
				ser_dcr_din_val &= ~(1 << 19);
			} else {
				ser_dcr_din_val |= (1 << 19);
			}
		}				
		
		if (up->uart_mode == AX99100_RS232_MODE) {
			if (up->ax99100_port_mode == AX99100_MF_PORT)
				ser_dcr_din_val |= (1 << 11);
			else
				ser_dcr_din_val &= ~(1 << 11);
			writel(ser_dcr_din_val , up->port.membase + SP_SETTING_REG0);
			MP_DBG(KERN_ERR"   set ser_dcr_din_val = 0x%x\n", ser_dcr_din_val);
			MP_DBG(KERN_ERR"   re-read set ser_dcr_din_val = 0x%x\n", readl(up->port.membase + SP_SETTING_REG0));
		} else {
			ser_dcr_din_val |= (1 << 11);
			writel(ser_dcr_din_val , up->port.membase + SP_SETTING_REG0);
			MP_DBG(KERN_ERR"   set ser_dcr_din_val = 0x%x\n", ser_dcr_din_val);
			MP_DBG(KERN_ERR"   re-read set ser_dcr_din_val = 0x%x\n", readl(up->port.membase + SP_SETTING_REG0));
		}
	}	
	
	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	if(up->dma_rx || up->dma_tx){		
		writel(0xFE,up->port.membase+REG_GLBL_IER);
		

		if(up->dma_rx){
			//Set the comset DMA register to enable DMA
			DEBUG("DMA bit in DCR register was set ");
			ser_dcr_din_val=readl(up->port.membase+SP_SETTING_REG0);
			ser_dcr_din_val |= COM_DMA_MODE_EN;
			writel(ser_dcr_din_val,up->port.membase+SP_SETTING_REG0);
			DEBUG("SP_SETTING_REG0=0x%x\n",readl(up->port.membase+SP_SETTING_REG0));	
		}

		
		if (!up->dma_tx && up->dma_rx) {
			serial_out(up,UART_IER, UART_IER_RDI | UART_IER_RLSI /*| UART_IER_MSI*/ | UART_IER_THRI);
		} else {
			serial_out(up,UART_IER, UART_IER_RDI | UART_IER_RLSI /*| UART_IER_MSI*/);
		}

		if(up->dma_rx){
			DEBUG("RX_DMA engine started\n");
			writel(up->dma_rx_buf_p,up->port.membase+REG_RX_DMA_START_ADDRESS_LOW);
			writel(0,up->port.membase+REG_RX_DMA_START_ADDRESS_HIGH);
			writel(DMA_RX_SZ,up->port.membase+REG_RX_DMA_LENGTH);
			writel(RX_DMA_START_BIT,up->port.membase+REG_RX_DMA_START);			
			up->pre_need2recv_cnt = DMA_RX_SZ;
		}
		if(up->dma_tx)			
			writel(0,up->port.membase + REG_TX_DMA_START_ADDRESS_HIGH);
	} else {
		serial_out(up,UART_IER,UART_IER_THRI);
	}

	/*
	 * And clear the interrupt generating registers again for luck.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	DEBUG("start end dll %02x\n", serial_in(up, UART_DLL));
	DEBUG("start end dlm %02x\n", serial_in(up, UART_DLM));
	DEBUG("staqrt end tcr %02x\n", serial_icr_read(up, UART_TCR));


	DEBUG("In %s --------------------------------------- %02X %02X %02X END\n",__FUNCTION__,
								serial_in(up, UART_DLL),
								serial_in(up, UART_DLM),
								serial_icr_read(up, UART_TCR));

	return 0;
}

//This is a port ops helper function to disable the port, disable any break condition that may be in
//effect, and free any interrupt resources.
static void serial99100_shutdown(struct uart_port *port)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	unsigned long flags, ser_dcr_din_val;
	u32 val;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	DEBUG("up= %x up->prev_port :%x and up->next_port :%x\n" ,up,up->prev_port,up->next_port);
	DEBUG("Device structure address for port%d id %u\n",port->line,up);
	DEBUG("membase is 0x%x\n",up->port.membase);
        DEBUG("mapbase is 0x%x\n",up->port.mapbase);
        DEBUG("iobase is 0x%x\n",up->port.iobase);

	printk("No of Errors In ttyF%d brake=%d frame=%d parity=%d overrun=%d\n",
		port->line, 
		port->icount.brk,
		port->icount.frame,
		port->icount.parity,
		port->icount.overrun);

	// disable 9-bit mode
	serial_icr_write(up, UART_NMR , 0);
	val=readl(up->port.membase + SP_SETTING_REG0);
	val &= 0xDFFFFFFF;
	val |= 0x20000000;
	writel(val, up->port.membase + SP_SETTING_REG0);
	serial_out(up, UART_SCR, 0);
	up->mode_9bit = uart_99100_contxts[port->line].mode_9bit;
	up->nodeID_9bit = uart_99100_contxts[port->line].nodeID_9bit;


	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_out(up, UART_IER, 0);
	
	//tasklet kill
	tasklet_kill(&up->tasklet_dma_rx);
	tasklet_kill(&up->tasklet_dma_tx);

	spin_lock_irqsave(&up->lock_99100, flags);
	up->port.mctrl &= ~TIOCM_OUT2;

	serial99100_set_mctrl(&up->port, up->port.mctrl);

	spin_unlock_irqrestore(&up->lock_99100, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial99100_clear_fifos(up);

	/*
	 * Read data port to reset things
	 */
	(void) serial_in(up, UART_LSR);
        (void) serial_in(up, UART_IIR);
        (void) serial_in(up, UART_MSR);
	(void) serial_in(up, UART_RX);

	up->lcr = 0;
	up->mcr = 0;
	up->ser_dcr_din_reg = 0;
	up->ser_ven_reg = 0;

	//Reset the UART upon port close
	if(up->port.type == PORT_ENHANCED){

		DEBUG("In ENHANCED MODE\n");
		DEBUG("up= %x up->prev_port :%x and up->next_port :%x\n" ,up,up->prev_port,up->next_port);
		DEBUG("membase is 0x%x\n",up->port.membase);
	        DEBUG("mapbase is 0x%x\n",up->port.mapbase);
        	DEBUG("iobase is 0x%x\n",up->port.iobase);
		up->acr = 0x00;
	
		// ENHANCED Mode reset
		serial_icr_write(up, UART_CSR, 0x00);
		serial_icr_write(up, UART_CSR, 0xFF);
	}

	// Serial soft reset
	writel(0x01,up->port.membase+SER_SOFT_RESET_REG);
	if(up->dma_rx){
		//Set the comset DMA register to enable DMA
		DEBUG("DMA bit in DCR register was set ");
		ser_dcr_din_val=readl(up->port.membase+SP_SETTING_REG0);
		ser_dcr_din_val &= ~(COM_DMA_MODE_EN);
		writel(ser_dcr_din_val,up->port.membase+SP_SETTING_REG0);
		DEBUG("SP_SETTING_REG0=0x%x\n",readl(up->port.membase+SP_SETTING_REG0));	
	}
	// Setup DXEN direction to input in RS485 mode
	if (up->uart_mode != AX99100_RS232_MODE) {
		writel(readl(up->port.membase + SP_SETTING_REG0) & ~(1 << 11) , up->port.membase + SP_SETTING_REG0);
	}	

	DEBUG("In %s --------------------------------------END\n",__FUNCTION__);
}

//This is a port ops helper function to return the divsor (baud_base / baud) for the selected baud rate 
//	specified by termios.
static unsigned int serial99100_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);

	quot = uart_get_divisor(port, baud);

	DEBUG("In %s quot=%u----baud=%u-----------------------------END\n",__FUNCTION__,quot,baud);
	return quot;	
}

//This is a port ops function to set the terminal settings.
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,1,0)
static void serial99100_set_termios(struct uart_port *port, struct ktermios *termios, const struct ktermios *old)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void serial99100_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
#else
static void serial99100_set_termios(struct uart_port *port, struct termios *termios, struct termios *old)
#endif
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	u8 cval,fcr=0;
	unsigned long flags;
	unsigned int baud, quot;
	unsigned int source_select = 0;  //Internal 1=1.8382M 0=125M 
	unsigned int sampling_clock = 16; // 16:use default 16 bits sampling clock
	u32 sp_clk_val = 0;
	u32 internal_clk_val = 0;	
	u32 gpi0_3D4_value;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = 0x00;
		break;
	case CS6:
		cval = 0x01;
		break;
	case CS7:
		cval = 0x02;
		break;
	default:
	case CS8:
		cval = 0x03;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif
	
	/*
	 * Ask the core to calculate the divisor for us.
	 */
	

	if (up->custom_setting == 1) {	
		if (!(readl(up->bar5membase + EDS_REG) & EDS_EOES) && up->baud_base_clock == CLK_EXTERNAL) {
			printk("AX99100_SP:Extenal Oscillator does not exist!\n");
			printk("AX99100_SP:Baud Rate setting: 115200\n");	
			up->baud_base_clock = 0x00;
			baud = 115200;
			up->custom_dlm = 0x00;
			up->custom_dll = 0x01;
			up->custom_sampling_clock = 0x10;
		}
	  
		source_select = up->baud_base_clock;		
		baud = up->custom_baud;	
		BR_DBG(KERN_ERR"   (custom)function %d BaudRate = %d\n", up->function_number , baud);		
		switch (up->baud_base_clock) {
			case CLK_125M:
				port->uartclk = BASE_CLK_125M;
				up->dma_delay_timeout = (5 * ((up->custom_dlm*256)+up->custom_dll) * 768)/1000+1;				
				break;
			case CLK_EXTERNAL:
				port->uartclk = BASE_CLK_24M;
				if (CusEEbuffer.ext_clk != 0xFFFFFFFF)
					port->uartclk = CusEEbuffer.ext_clk;
					
				up->dma_delay_timeout = (42 * ((up->custom_dlm*256)+up->custom_dll) * 768)/1000+1;
				break;
			case CLK_1_8382M:
			default :
				port->uartclk = BASE_CLK_1_838235;
				up->dma_delay_timeout = (540 * ((up->custom_dlm*256)+up->custom_dll) * 768)/1000+1;
				break;
		}		

		sp_clk_val = (readl(up->port.membase + SP_BR_CLK_SEL_REG) & CLK_MASK) | source_select | 0x30;		
		writel(sp_clk_val, up->port.membase + SP_BR_CLK_SEL_REG);
		BR_DBG(KERN_ERR"   (custom)function %d SP_BR_CLK_SEL_REG = 0x%x\n",up->function_number , readl(up->port.membase + SP_BR_CLK_SEL_REG));
		
		if ( source_select == CLK_EXTERNAL ){		  
			do {
				 mdelay(1);
				 gpi0_3D4_value = readl(up->bar5membase + EDS_REG);
				 if ( gpi0_3D4_value & EDS_ECSS )
					break;
			} while (1);
		}
		
		internal_clk_val = readl(up->bar5membase + 0x70);
		if (source_select == CLK_1_8382M)
			internal_clk_val |= (1 << up->function_number);
		else
			internal_clk_val &= ~(1 << up->function_number);
  
		BR_DBG(KERN_ERR"   (custom)function %d bar5 + 0x70 = 0x%X\n",up->function_number , internal_clk_val);
		writel(internal_clk_val, up->bar5membase + 0x70);	

		BR_DBG(KERN_ERR"   (custom)function %d dlm = %d, dll = %d\n",up->function_number ,up->custom_dlm, up->custom_dll);
		serial_out(up, UART_LCR, cval | UART_LCR_DLAB);	/* set DLAB */
		serial_out(up, UART_DLL, up->custom_dll);	/* LS of divisor */
		serial_out(up, UART_DLM, up->custom_dlm);	/* MS of divisor */
		serial_out(up, UART_LCR, cval);			/* reset DLAB */
		up->lcr = cval;					/* Save LCR */

		sampling_clock = up->custom_sampling_clock;
		BR_DBG(KERN_ERR"   (custom)function %d sampling clock = 0x%X\n",up->function_number ,sampling_clock);

	} else {
		port->uartclk = DEFAULT99100_BAUD * 16;	
		baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk);
		DEBUG("In %s -------------------baud=%u\n",__FUNCTION__,baud);
		BR_DBG(KERN_ERR"   (standard)function %d BaudRate = %d\n",up->function_number , baud);
		// default source_select = CLK_1_8382M
		internal_clk_val = readl(up->bar5membase + 0x70);
		internal_clk_val |= (1 << up->function_number);
		
		BR_DBG(KERN_ERR"   (standard)function %d bar5 + 0x70 = 0x%X\n",up->function_number , internal_clk_val);
		writel(internal_clk_val, up->bar5membase + 0x70);

		// default source_select = CLK_1_8382M
		source_select	= CLK_1_8382M;		
		
		if(baud > 115200) {

			quot = 1;

			switch (baud) {				
				case 230400:
					port->uartclk = BASE_CLK_125M;
					source_select = CLK_125M;
					sampling_clock = 17;
					quot = 32;
					break;
				case 460800:
					port->uartclk = BASE_CLK_125M;
					source_select = CLK_125M;
					sampling_clock = 17;
					quot = 16;
					break;
				case 576000:
					port->uartclk = BASE_CLK_125M;
					source_select = CLK_125M;
					sampling_clock = 31;
					quot = 7;
					break;
				case 921600:
					port->uartclk = BASE_CLK_125M;
					source_select = CLK_125M;
					sampling_clock = 17;
					quot = 8;
					break;
				case 1152000:
					port->uartclk = BASE_CLK_125M;
					source_select = CLK_125M;
					sampling_clock = 27;
					quot = 4;
					break;
				case 1500000:
					port->uartclk = BASE_CLK_24M;
					source_select = CLK_EXTERNAL;
					sampling_clock = 16;
					break;
				case 2000000:
					port->uartclk = BASE_CLK_24M;
					source_select = CLK_EXTERNAL;
					sampling_clock = 12;					
					break;
				case 3000000:
					port->uartclk = BASE_CLK_24M;
					source_select = CLK_EXTERNAL;
					sampling_clock = 8;					
					break;
				case 4000000:
					port->uartclk = BASE_CLK_24M;
					source_select = CLK_EXTERNAL;
					sampling_clock = 6;					
					break;
				default:
					printk("Now setup baud rate = 115200 bps.\n");
					break;
			}
		} else {
			quot = serial99100_get_divisor(port, baud);
		}		

		sp_clk_val = (readl(up->port.membase + SP_BR_CLK_SEL_REG) & CLK_MASK) | source_select | 0x30;
		BR_DBG(KERN_ERR"   (standard)function %d SP_BR_CLK_SEL_REG = 0x%X\n",up->function_number , sp_clk_val);
		writel(sp_clk_val, up->port.membase + SP_BR_CLK_SEL_REG);

		BR_DBG(KERN_ERR"   (standard)function %d dlm = %d, dll = %d\n",up->function_number ,quot >> 8, quot & 0xff);
		serial_out(up, UART_LCR, cval | UART_LCR_DLAB);	/* set DLAB */
		serial_out(up, UART_DLL, quot & 0xff);		/* LS of divisor */
		serial_out(up, UART_DLM, quot >> 8);		/* MS of divisor */
		serial_out(up, UART_LCR, cval);			/* reset DLAB */
		up->lcr = cval;					/* Save LCR */

		BR_DBG(KERN_ERR"   (standard)function %d sampling clock = 0x%X\n",up->function_number ,sampling_clock);
		up->dma_delay_timeout = (540 * quot * 768)/1000+1;
	}

	if (up->capabilities & UART_CAP_FIFO && uart_config[port->type].fifo_size > 1) {
		if (baud < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
		else
			fcr = uart_config[up->port.type].fcr;
	}
	
	/*
	 * MCR-based auto flow control.  When AFE is enabled, RTS will be
	 * deasserted when the receive FIFO contains more characters than
	 * the trigger, or the MCR RTS bit is cleared.  In the case where
	 * the remote UART is not using CTS auto flow control, we must
	 * have sufficient FIFO entries for the latency of the remote
	 * UART to respond.  IOW, at least 32 bytes of FIFO.
	 */
	if (up->capabilities & UART_CAP_AFE && uart_config[port->type].fifo_size >= 32) {
		up->mcr &= ~UART_MCR_AFE;
		if (termios->c_cflag & CRTSCTS)
			up->mcr |= UART_MCR_AFE;
	}

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->lock_99100, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;

	serial_out(up, UART_IER, up->ier);
	if (up->capabilities & UART_CAP_EFR) {
		unsigned char efr = 0;
		/*
		 * TI16C752/Startech hardware flow control.  FIXME:
		 * - TI16C752 requires control thresholds to be set.
		 * - UART_MCR_RTS is ineffective if auto-RTS mode is enabled.

		 */
		if (termios->c_cflag & CRTSCTS)
			efr |= UART_EFR_CTS;

		serial_out(up, UART_LCR, 0xBF);
		serial_out(up, UART_EFR, efr);
	}

	if (up->port.type != PORT_16750) {
		if (fcr & UART_FCR_ENABLE_FIFO) {
			/* emulated UARTs (Lucent Venus 167x) need two steps */
			serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
			
		}
		serial_out(up,UART_FCR,fcr);		/* set fcr */
		DEBUG("In %s UART_FCR is written with fcr=0x%x\n",__FUNCTION__,fcr);	
	}

	serial_icr_write(up, UART_TCR, sampling_clock);

	serial99100_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->lock_99100, flags);
	DEBUG("In %s ------------------------------END\n",__FUNCTION__);
}

static void serial99100_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
	struct uart_99100_port *p = &serial99100_ports[port->line];
	serial99100_set_sleep(p, state != 0);
}

//Helper function to relase the kernel resources used by the port
static void serial99100_release_port(struct uart_port *port)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);
	
	iounmap(up->port.membase);
	
	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
}

//Helper function to get the necessary kernel resources for the port
static int serial99100_request_port(struct uart_port *port)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	#if defined(__x86_64__) || defined(__amd64__)
	int ret = 0,size=8,mem_size=4096;
	#endif
	#if defined(__arm__) || defined(__aarch64__)
	int ret = 0,mem_size=4096;
	#endif

	//todo:the mem_size and the mem2_size are not yet known properly
	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);
#if defined(__x86_64__) || defined(__amd64__)
	if(!request_region(up->port.iobase, size, "AX99100"))
		return -EBUSY;
#endif
	if(!request_mem_region(up->port.mapbase, mem_size, "AX99100")){
		ret = -EBUSY;
		goto release;
	}
	
	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
	return ret;
	
release:
#if defined(__x86_64__) || defined(__amd64__)
	release_region(up->port.iobase,size);
#endif
	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
	return ret;
}

static const char *serial99100_type(struct uart_port *port)
{
	return "AX99100";
}

static void serial99100_getCustomModeFromEeprom(struct uart_99100_port *port)
{
	//EEPROM_DATA eepromData;
	unsigned long writeValue = 0, offset;
	unsigned short sum, i;
	unsigned char index;
	u8 *tmp = (u8*) &CusEEbuffer;
	
	
	DEBUG("AX99100: serial99100_getCustomModeFromEeprom #C\n");
	
	index = 0x28; 	//Device I2C address, we should have a way to determine address
	offset = 0x60;
	
	if(!port->bar5membase)
	{
		DEBUG("AX99100: GPIO/EEPROM base address not found\n");
		return;
	} else {
		DEBUG("REG_I2CCR %08X\n", readl(port->bar5membase + REG_I2CCR));
		DEBUG("REG_I2CSCLPR %08X\n", readl(port->bar5membase + REG_I2CSCLPR));
		DEBUG("REG_I2CSCLCR %08X\n", readl(port->bar5membase + REG_I2CSCLCR));
		DEBUG("REG_I2CBFTR %08X\n", readl(port->bar5membase + REG_I2CBFTR));
	}
	

	for (i = 0; i < CUSTOM_EEPROM_LEN; i++) 
	{
		unsigned long start;			
#if EEPROM8BIT
		writeValue = ((index << 25) | (offset << 8)) & (~(1 << 24)); //8bits
#else
		writeValue = (index << 25) | (1 << 24) | (offset << 8); //16bits
#endif

		start = jiffies;
		do {
			writel(writeValue, port->bar5membase + REG_I2CCR);
			
			if (time_after(jiffies, start + HZ / 100)) {
				DEBUG("AX99100: I2CSCLCR_CHECK error %08X\n",
					readl(port->bar5membase + REG_I2CSCLCR));
				return;
			}

		} while(readl(port->bar5membase + REG_I2CSCLCR) & I2CSCLCR_CHECK);

		*(tmp + i) = (u8)(readl(port->bar5membase + REG_I2CCR) & 0xFF);
		
		DEBUG("AX99100: eBuffer[%d]:%02X\n", i, *(tmp + i));
		offset++;
		
		writeValue = 0x0;
	}
	if(CusEEbuffer.cus_mod == 0xFFFF) {
		memset(&CusEEbuffer, 0, sizeof(CusEEbuffer));
		return;
	}
	DEBUG("cus_mod %04x\n", CusEEbuffer.cus_mod);
	for (i = 0; i < 4; i++) {
		DEBUG("pt_setting[%d] en_cusbaud_clksrc %02x dll %02x dlm %02x sample_rate %02x\n", i,
			CusEEbuffer.pt_setting[i].en_cusbaud_clksrc,
			CusEEbuffer.pt_setting[i].dll,
			CusEEbuffer.pt_setting[i].dlm,
			CusEEbuffer.pt_setting[i].sample_rate);
	}
	DEBUG("ext_clk %d\n", CusEEbuffer.ext_clk);
	DEBUG("chksum %02x\n", CusEEbuffer.chksum);

	sum = 0;
	for (i = 0; i < CUSTOM_EEPROM_LEN; i++)
	{
		sum += *(tmp + i);
	}
	if (sum >> 8)
		sum = ((sum >> 8) & 0x00FF) + (sum & 0x00FF);
		
	if (sum != 0x79)
	{
		DEBUG("AX99100: incorrect checksum\n");
		return;
	}
}

void serial99100_serialSettingGPIO(struct uart_99100_port *up)
{
	unsigned long lSetGpioValue = 0;

	if (up->oriDTR == DTR_UNKNOWN)
		up->oriDTR = (serial_in(up,UART_MCR) & TIOCM_DTR) >> 1;

	GpioSetValueGroup0 = (SetGpioValue & 1) | ((SetGpioValue & 4) >> 1);
	GpioSetValueGroup1 = ((SetGpioValue & 2) >> 1) | ((SetGpioValue & 8) >> 2);
	GpioSetValueGroup2 = ((SetGpioValue & 0x10) >> 3) | ((SetGpioValue & 0x40) >> 6);
	GpioSetValueGroup3 = ((SetGpioValue & 0x20) >> 4) | ((SetGpioValue & 0x80) >> 7);
    
	if (!up->oriDTR) {   
		TtempValue = 0;
		INIT_DBG(KERN_ERR "ASUS 485 Mode\n");

		if (up->function_number == 2) {
            		TtempValue = 2;
			GpioSetValueGroup0 = 0x0;
			GpioSetValueGroup2 = 0x0 | TtempValue;
		    
		} else if (up->function_number == 3) {
			TtempValue = 2;
			GpioSetValueGroup1 = 0x0;
			GpioSetValueGroup3 = 0x0 | TtempValue;
		}

		lSetGpioValue = ((GpioSetValueGroup0 & 0x01) | ((GpioSetValueGroup0 & 0x02) << 1)

				| ((GpioSetValueGroup1 & 0x01) << 1) | ((GpioSetValueGroup1 & 0x02) << 2)
				| ((GpioSetValueGroup2 & 0x01) << 6) | ((GpioSetValueGroup2 & 0x02) << 3)
				| ((GpioSetValueGroup3 & 0x01) << 7) | ((GpioSetValueGroup3 & 0x02) << 4));

		//Set GPIO 0~7
		writel((unsigned long)0xFFFF00, up->bar5membase + REG_GPIODIR); //Set DIR
		writel(lSetGpioValue, up->bar5membase + REG_GPIOPIN);      //Set Value
		SetGpioValue = lSetGpioValue;
		writel((unsigned long)0x04, up->port.membase + SP_GPIO_ENABLE_REG); //Enable
		
		if (up->oriCTS == 1) {
			//RS485_HALF_DUPLEX
			writel((unsigned long)0x24, up->port.membase + SP_GPIO_ENABLE_REG);
			writel( (unsigned long)0x1F, up->port.membase + SP_GPIO_OUTPUT_REG);
			up->uart_mode = AX99100_RS485_HALF_DUPLEX;
		} else {
			//RS485_FULL_DUPLEX
			writel( (unsigned long)0x04, up->port.membase + SP_GPIO_ENABLE_REG);
			up->uart_mode = AX99100_RS485_FULL_DUPLEX;
		}


		INIT_DBG(KERN_ERR "CtsInitValue = 0x%x.\r\n", up->oriCTS);

		

	} else {
		INIT_DBG(KERN_ERR "ASUS 232 Mode\n");
		up->uart_mode = AX99100_RS232_MODE;

		TtempValue = 0;

		if (up->function_number == 2)
		{
		    GpioSetValueGroup0 = 0x1;
		    GpioSetValueGroup2 = 0x1 | TtempValue;
		} else if (up->function_number == 3)
		{
		    GpioSetValueGroup1 = 0x1;
		    GpioSetValueGroup3 = 0x1 | TtempValue;
		}


		lSetGpioValue = ((GpioSetValueGroup0 & 0x01) | ((GpioSetValueGroup0 & 0x02) << 1)
				| ((GpioSetValueGroup1 & 0x01) << 1) | ((GpioSetValueGroup1 & 0x02) << 2)
				| ((GpioSetValueGroup2 & 0x01) << 6) | ((GpioSetValueGroup2 & 0x02) << 3)
				| ((GpioSetValueGroup3 & 0x01) << 7) | ((GpioSetValueGroup3 & 0x02) << 4));
		//Set GPIO 0~7
		writel((unsigned long)(0xFFFF00), up->bar5membase + REG_GPIODIR);
		writel(lSetGpioValue, up->bar5membase + REG_GPIOPIN);
		SetGpioValue = lSetGpioValue;
	}               
}

static int serial99100_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct uart_99100_port *up = &serial99100_ports[port->line];
	struct _slave_mode_config slave;
	int __user *p = (int __user*)arg;
	int gpio_dir_99100 = 0;
	int gpio_output_99100 = 0;
	u32 eds, val;
	u8 lcr, efr, start_value;

	switch (cmd) {
	case IOCTL_GET_CUSTOM:
		__put_user(up->custom_baud, p);
		break;
        case IOCTL_SET_CUSTOM:
		if (arg == 0) {
			up->custom_setting = 0;
			up->custom_baud = 0;

		} else {
			up->custom_setting = 1;
			up->custom_baud = arg;
		}
		break;
	case IOCTL_GET_PRODUCT:
		__put_user(up->ax99100_port_mode, p);
		break;
        case IOCTL_SET_PARAMETER:
		up->baud_base_clock = (arg >> 20) & 0xFF;		
		up->custom_dlm = (arg >> 12) & 0xFF;		
		up->custom_dll = (arg >> 4) & 0xFF;				
		break;
	case IOCTL_SET_SAMPLING:
		up->custom_sampling_clock = (arg >> 0);
		break;

	case IOCTL_GPIO_DIR:
		gpio_dir_99100 = arg & 0xFF;
		writel(gpio_dir_99100, up->bar5membase + 0x3C4);
		break;

	case IOCTL_GPIO_STATUS:
		gpio_dir_99100 = readl(up->bar5membase + 0x3C4) & 0XFF;
		gpio_output_99100 = readl(up->bar5membase + 0x3C0) & 0XFF;
		__put_user((gpio_dir_99100 << 8) | gpio_output_99100, p);
 		break;
	case IOCTL_GPIO_OUTPUT:
		gpio_output_99100 = arg & 0xFF;
		writel(gpio_output_99100, up->bar5membase + 0x3C0);
		break;	 
	case IOCTL_GET_EDS_EOES: /* 2016/08/02 For ext_clk */
		eds = readl(up->bar5membase + EDS_REG) & 0x0F000000;
		__put_user(eds, p);
		break;
	case IOCTL_SET_9BIT_DATA: /* 9-bit mode */
		if(copy_from_user(&start_value, p, sizeof(start_value)))
			return -ENOIOCTLCMD;
		serial_out(up, UART_SCR, (start_value & 0x01));
		break;
	case IOCTL_SET_SLAVE_MODE: /* 9-bit mode */		
		if(copy_from_user(&slave, p, sizeof(slave)))
			return -ENOIOCTLCMD;
		if (slave.slave_mode >= MODE_9BIT_DISABLE &&
	            slave.slave_mode < (MODE_9BIT_DATA + 1)) {
			up->mode_9bit = slave.slave_mode;
			up->nodeID_9bit = (slave.slave_id & 0xFF);
			if (up->mode_9bit == MODE_9BIT_MASTER   ||
			    up->mode_9bit == MODE_9BIT_SLAVE_SW ||
			    up->mode_9bit == MODE_9BIT_DATA) {
				/* Enable 9 bit */
				serial_icr_write(up, UART_NMR , (NMR_NBE | NMR_NBIE));	
				
				val=readl(up->port.membase + SP_SETTING_REG0);
				val &= 0xDFFFFFFF;
				val |= 0x20000000;
				writel(val, up->port.membase + SP_SETTING_REG0);			
			} else if (up->mode_9bit == MODE_9BIT_SLAVE_HW) {
				/* Enable 9 bit & addesss mode */
				serial_icr_write(up, UART_NMR , (NMR_NBE | NMR_NBIE | NMR_AME));

				val=readl(up->port.membase + SP_SETTING_REG0);
				val &= 0xDFFFFFFF;
				val |= 0x20000000;
				writel(val, up->port.membase + SP_SETTING_REG0);

				lcr = serial_in(up,UART_LCR);
				serial_out(up,UART_LCR,0xBF);
				efr=serial_in(up,UART_EFR);			
				serial_out(up,UART_XOFF2,up->nodeID_9bit);
				serial_out(up,UART_EFR,efr);
				serial_out(up,UART_LCR,lcr);				
			} else {
				/* Disable 9 bit */
				serial_icr_write(up, UART_NMR , 0);
				up->mode_9bit = MODE_9BIT_DISABLE;
				up->nodeID_9bit = 0;
			}
		}
		
		break;
	default:
		return -ENOIOCTLCMD;	
	}

	return 0;
}

static struct uart_ops serial99100_pops = {
	.tx_empty		= serial99100_tx_empty,
	.set_mctrl		= serial99100_set_mctrl,
	.get_mctrl		= serial99100_get_mctrl,
	.stop_tx		= serial99100_stop_tx,
	.start_tx		= serial99100_start_tx,
	.stop_rx		= serial99100_stop_rx,
	.enable_ms		= serial99100_enable_ms,
	.break_ctl		= serial99100_break_ctl,
	.startup		= serial99100_startup,
	.shutdown		= serial99100_shutdown,
	.set_termios		= serial99100_set_termios,
	.pm			= serial99100_pm,
	.type			= serial99100_type,
	.release_port		= serial99100_release_port,
	.request_port		= serial99100_request_port,
	.ioctl			= serial99100_ioctl,

};

static void serial99100_dma_rx_tasklet (unsigned long param);
static void serial99100_dma_tx_tasklet (unsigned long param);

//Initialising the global per port context array to the default values
static void serial99100_init_port(struct uart_99100_port *up)
{
	spin_lock_init(&up->port.lock);
	spin_lock_init(&up->lock_99100);
	up->port.ops 	= &serial99100_pops;
	up->port.iotype = UPIO_PORT;
	up->port.type 	= PORT_16550A;
	up->port.flags |= UPF_SHARE_IRQ;
	//tasklet
	up->tasklet_dma_rx.func	= serial99100_dma_rx_tasklet;
	up->tasklet_dma_rx.data	= (unsigned long) up;
	up->tasklet_dma_tx.func	= serial99100_dma_tx_tasklet;
	up->tasklet_dma_tx.data	= (unsigned long) up;

	// 9bit mode
	up->mode_9bit = MODE_9BIT_DISABLE;
	up->nodeID_9bit = 0;
	up->enable_slave_9bit = 0;
}

//Initialising the maximum allowed per port Structures with the default values
static void __init serial99100_init_ports(void)
{
	int i;
	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);

	memset(serial99100_ports, 0, UART99100_NR * sizeof(struct uart_99100_port));
	for (i = 0; i < UART99100_NR; i++) {
		serial99100_init_port(&serial99100_ports[i]);
		serial99100_ports[i].port.line = i;
	}
	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
}

static void receive_chars_dma_done(struct uart_99100_port * up, int iirg);
static int transmit_chars_dma_done(struct uart_99100_port * up);

static void serial99100_dma_rx_tasklet (unsigned long param)
{
	struct uart_99100_port *up = (struct uart_99100_port *) param;
	
	u8 iir = serial_in(up, UART_IIR);
	
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26))
	struct tty_struct *tty=up->port.info->tty;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	struct tty_struct *tty = up->port.info->port.tty;
#else
	struct tty_struct *tty = up->port.state->port.tty;
#endif
	
	receive_chars_dma_done(up,up->k_gir);

	if (!(iir & UART_IIR_NO_INT)) {
		if (up->k_lsr & (UART_LSR_BI | UART_LSR_PE |UART_LSR_FE | UART_LSR_OE)){
		//For statistics only
			if (up->k_lsr & UART_LSR_BI) {
				up->k_lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					return;
			} else if (up->k_lsr & UART_LSR_PE) {
				up->port.icount.parity++;
			} else if (up->k_lsr & UART_LSR_FE) {
				up->port.icount.frame++;
			} else if (up->k_lsr & UART_LSR_OE) {
				up->port.icount.overrun++;
						//Mask off conditions which should be ignored.
				up->k_lsr &= up->port.read_status_mask;		
			}
		}	
		if (up->k_lsr & ~up->port.ignore_status_mask & UART_LSR_OE)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
			tty_insert_flip_char(tty->port, 0, TTY_OVERRUN);
#else
			tty_insert_flip_char(tty, 0, TTY_OVERRUN);
#endif
	}
}

static void serial99100_dma_tx_tasklet (unsigned long param)
{
	struct uart_99100_port *up = (struct uart_99100_port *) param;

	transmit_chars_dma_done(up);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
static DECLARE_MUTEX(serial99100_sem);
#else
static DEFINE_SEMAPHORE(serial99100_sem);
#endif

static struct uart_driver starex_serial_driver = {
        .owner                  = THIS_MODULE,
        .driver_name            = "AX99100",
        .dev_name               = "ttyF",//E",//D",
        .major                  = 200,
        .minor                  = 0,
        .nr                     = UART99100_NR,
        .cons                   = NULL,
};
//uint counter=0;
int serial99100_find_match_or_unused(struct uart_port *port)
{
	int i;
	// printk("In %s counter=%d-----line %d",__FUNCTION__,counter++,__LINE__);

	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
#if defined(__x86_64__) || defined(__amd64__)
	for (i = 0; i < UART99100_NR; i++){
		if (serial99100_ports[i].port.iobase == 0){
			return i;
			}
	}	
#endif


#if defined(__arm__) || defined(__aarch64__)
	for (i = 0; i < UART99100_NR; i++){
		if (serial99100_ports[i].port.membase == 0){
			return i;
			// printk("In %s counter=%d-----line %d",__FUNCTION__,counter,__LINE__);

			}
	}	
#endif
	/*
	 * That also failed.  Last resort is to find any entry which
	 * doesn't have a real port associated with it.
	 */
	for (i = 0; i < UART99100_NR; i++){
		if (serial99100_ports[i].port.type == PORT_UNKNOWN){
			return i;
			}	
	}		
	return -1;
}
//uint counter1=0;

static void serial99100_getCustomModeFromEeprom(struct uart_99100_port *port);

int serial99100_register_port(struct uart_port *port,struct pci_dev *dev)
{
	// printk("In %s counter1=%d-----line %d",__FUNCTION__,++counter1,__LINE__);	
	unsigned long base, len;
	int index,ret = -ENOSPC, i = 0;
	// printk("In %s uartclk=%d-----line %d",__FUNCTION__,port->uartclk,__LINE__);
	if (port->uartclk == 0){
		// printk("In %s -----line %d",__FUNCTION__,__LINE__);
		return -EINVAL;
	}


	down(&serial99100_sem);

	port->iotype = UPIO_PORT;
	port->flags |= UPF_SHARE_IRQ;
	port->type = PORT_ENHANCED;

	index = serial99100_find_match_or_unused(port);
	if (index >= 0) {

		serial99100_ports[index].port.ops = &serial99100_pops;

		serial99100_ports[index].port.iobase	= port->iobase;
		serial99100_ports[index].port.membase	= port->membase;
		serial99100_ports[index].port.irq	= port->irq;
		serial99100_ports[index].port.uartclk	= port->uartclk;
		serial99100_ports[index].port.fifosize	= port->fifosize;
		serial99100_ports[index].port.regshift	= port->regshift;
		serial99100_ports[index].port.iotype	= port->iotype;
		serial99100_ports[index].port.flags	= port->flags;
		serial99100_ports[index].port.mapbase	= port->mapbase;
		serial99100_ports[index].port.line		= index;
		//This is the default value
		serial99100_ports[index].port.type 		= PORT_ENHANCED;

		serial99100_ports[index].custom_setting		= 0;
		serial99100_ports[index].custom_baud		= 0;
		serial99100_ports[index].baud_base_clock	= 0;
		serial99100_ports[index].custom_dlm		= 0;
		serial99100_ports[index].custom_dll		= 0;
		serial99100_ports[index].custom_sampling_clock	= 0;

		serial99100_ports[index].function_number = dev->devfn;

		/* Ausu */
		serial99100_ports[index].oriDTR = DTR_UNKNOWN;
		serial99100_ports[index].oriCTS = CTS_UNKNOWN;
		len =  pci_resource_len(dev, FL_BASE5);
		base = pci_resource_start(dev, FL_BASE5);
		serial99100_ports[index].bar5membase = kmalloc(sizeof(unsigned char), GFP_KERNEL);
		serial99100_ports[index].bar5membase = ioremap(base,len);
		INIT_DBG(KERN_ERR"   base = 0x%lX\n", base);
		INIT_DBG(KERN_ERR"   bar5membase = 0x%X\n", (int)serial99100_ports[index].bar5membase);

		gpio_mode = readl(serial99100_ports[index].bar5membase + 0x3D8) >> 29;
		INIT_DBG(KERN_ERR"   gpio_mode = %d\n", gpio_mode);

		if (gpio_mode == GPIO_4MP_MODE || gpio_mode == GPIO_2MP_1SPI_MODE || gpio_mode == GPIO_2MP_1P_MODE) {
			serial99100_ports[index].ax99100_port_mode = AX99100_MF_PORT;
		} else if (gpio_mode == GPIO_2S_1SPI_MODE || gpio_mode == GPIO_4S_MODE || gpio_mode == GPIO_2S_1P_MODE) {
			serial99100_ports[index].ax99100_port_mode = AX99100_SERIAL_PORT;
		} else if (gpio_mode == GPIO_2S_2MP_MODE) {
			if (serial99100_ports[index].function_number == 0)
				serial99100_ports[index].ax99100_port_mode = AX99100_SERIAL_PORT;
			else if (serial99100_ports[index].function_number == 1)
				serial99100_ports[index].ax99100_port_mode = AX99100_SERIAL_PORT;
			else if (serial99100_ports[index].function_number == 2)
				serial99100_ports[index].ax99100_port_mode = AX99100_MF_PORT;
			else if (serial99100_ports[index].function_number == 3)
				serial99100_ports[index].ax99100_port_mode = AX99100_MF_PORT;
		} else {
			serial99100_ports[index].ax99100_port_mode = AX99100_SERIAL_PORT;
		}

		if (serial99100_ports[index].ax99100_port_mode == AX99100_MF_PORT) {
			serial99100_ports[index].port.type = PORT_ENHANCED;

			// Setup the GPIO = out to control 2872
			if (serial99100_ports[index].function_number == 0) {
				writel(readl(serial99100_ports[index].bar5membase + 0x3C4) & 0xFFFFFFC7, 
					serial99100_ports[index].bar5membase + 0x3C4);
			} else if (serial99100_ports[index].function_number == 2) {
				writel(readl(serial99100_ports[index].bar5membase + 0x3C4) & 0xFFFFFFF8, 
					serial99100_ports[index].bar5membase + 0x3C4);
			}
			INIT_DBG(KERN_ERR"   0x3C4 = 0x%x\n", readl(serial99100_ports[index].bar5membase + 0x3C4));
		}

		if (serial99100_ports[index].ax99100_port_mode == AX99100_MF_PORT)
			INIT_DBG(KERN_ERR"   ax99100_port_mode = multi-protocol\n");
		else if (serial99100_ports[index].ax99100_port_mode == AX99100_SERIAL_PORT)
			INIT_DBG(KERN_ERR"   ax99100_port_mode = serial port\n");
		else
			INIT_DBG(KERN_ERR"   ax99100_port_mode = unknown\n");

		if (port->dev)
			serial99100_ports[index].port.dev = port->dev;

		ret = uart_add_one_port(&starex_serial_driver,&serial99100_ports[index].port);
		if (ret<0)
			DEBUG("uart_add_one_port ----------failed\n");
		if (ret == 0)
			ret = serial99100_ports[index].port.line;

		if (uart_99100_contxts[index].mode_9bit != MODE_9BIT_DISABLE) {
			uart_99100_contxts[index].tx_dma_en = 0;
			uart_99100_contxts[index].rx_dma_en = 0;
			serial99100_ports[index].mode_9bit = uart_99100_contxts[index].mode_9bit;
			serial99100_ports[index].nodeID_9bit = uart_99100_contxts[index].nodeID_9bit;
		}	
		
		if (uart_99100_contxts[index].tx_dma_en == 1) {
			serial99100_ports[index].dma_tx=1;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,15,0)
			serial99100_ports[index].dma_tx_buf_v =
				(char *)pci_alloc_consistent(dev,DMA_TX_BUFFER_SZ,&serial99100_ports[index].dma_tx_buf_p);
			serial99100_ports[index].dma_tx_buf_v_start =
				(char *)pci_alloc_consistent(dev,DMA_TX_BUFFER_SZ,&serial99100_ports[index].dma_tx_buf_p_start);
#else
			serial99100_ports[index].dma_tx_buf_v =
				(char *)dma_alloc_coherent(&dev->dev,DMA_TX_BUFFER_SZ,&serial99100_ports[index].dma_tx_buf_p, GFP_ATOMIC);
			serial99100_ports[index].dma_tx_buf_v_start =
				(char *)dma_alloc_coherent(&dev->dev,DMA_TX_BUFFER_SZ,&serial99100_ports[index].dma_tx_buf_p_start, GFP_ATOMIC);
#endif


			memset(serial99100_ports[index].dma_tx_buf_v,0,DMA_TX_BUFFER_SZ);
			memset(serial99100_ports[index].dma_tx_buf_v_start,0,DMA_TX_BUFFER_SZ);

			serial99100_ports[index].serialise_txdma=0;
			serial99100_ports[index].first_tx_dma=1;

			DEBUG("dma_tx_buf_v=0x%x\n dma_tx_buf_p=0x%x\n", (unsigned int)serial99100_ports[index].dma_tx_buf_v,
				(unsigned int)serial99100_ports[index].dma_tx_buf_p);
		} else {
			serial99100_ports[index].dma_tx=0;
			serial99100_ports[index].dma_tx_buf_v=NULL;
			serial99100_ports[index].dma_tx_buf_v_start=NULL;
		}
		
		if (uart_99100_contxts[index].rx_dma_en == 1) {
			serial99100_ports[index].dma_rx=1;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,15,0)
			serial99100_ports[index].dma_rx_buf_v =
				(char *)pci_alloc_consistent(dev,DMA_RX_BUFFER_SZ,&serial99100_ports[index].dma_rx_buf_p);
#else
			serial99100_ports[index].dma_rx_buf_v =
				(char *)dma_alloc_coherent(&dev->dev,DMA_RX_BUFFER_SZ,&serial99100_ports[index].dma_rx_buf_p, GFP_ATOMIC);		
#endif	
			memset(serial99100_ports[index].dma_rx_buf_v,0,DMA_RX_BUFFER_SZ);			
			serial99100_ports[index].part_done_recv_cnt=0;	
			serial99100_ports[index].rx_dma_done_cnt=0;
			DEBUG("dma_rx_buf_v=0x%x\n dma_rx_buf_p=0x%x\n",(unsigned int)serial99100_ports[index].dma_rx_buf_v,
				(unsigned int)serial99100_ports[index].dma_rx_buf_p);
		} else {
			serial99100_ports[index].dma_rx=0;
			serial99100_ports[index].dma_rx_buf_v=NULL;
		}		

		serial99100_ports[index].uart_mode = uart_99100_contxts[index].uart_mode;
		serial99100_ports[index].flow_control = uart_99100_contxts[index].en_flow_control;
		serial99100_ports[index].flow_ctrl_type = uart_99100_contxts[index].flow_ctrl_type;
		serial99100_ports[index].x_on = uart_99100_contxts[index].x_on;
		serial99100_ports[index].x_off = uart_99100_contxts[index].x_off;
		serial99100_ports[index].ltc2872_te485 = uart_99100_contxts[index].ltc2872_te485;
		serial99100_ports[index].ltc2872_lb = uart_99100_contxts[index].ltc2872_lb;
		serial99100_ports[index].ltc2872_dz = uart_99100_contxts[index].ltc2872_dz;
		serial99100_ports[index].ltc2872_fen = uart_99100_contxts[index].ltc2872_fen;
		serial99100_ports[index].ser_dcr_din_reg = 0;
		serial99100_ports[index].ser_ven_reg = 0;
		serial99100_ports[index].acr = 0;
		serial99100_ports[index].lcr = 0;
		serial99100_ports[index].mcr = 0;

		serial99100_getCustomModeFromEeprom(&serial99100_ports[index]);
		i = serial99100_ports[index].function_number;
		if (CusEEbuffer.pt_setting[i].en_cusbaud_clksrc & EN_CUS_BAUD) {
			unsigned int uartclk = 0;

			serial99100_ports[index].custom_setting		= 1;
			serial99100_ports[index].custom_baud		= 0;
			serial99100_ports[index].baud_base_clock	= CusEEbuffer.pt_setting[i].en_cusbaud_clksrc & 0x3;
			serial99100_ports[index].custom_dlm		= CusEEbuffer.pt_setting[i].dlm;
			serial99100_ports[index].custom_sampling_clock =
					(!CusEEbuffer.pt_setting[i].sample_rate ? 4 : CusEEbuffer.pt_setting[i].sample_rate);
			serial99100_ports[index].custom_dll = (!CusEEbuffer.pt_setting[i].dll ? 1 : CusEEbuffer.pt_setting[i].dll); 

			switch (serial99100_ports[index].baud_base_clock) {
				case CLK_125M:
					uartclk = BASE_CLK_125M;
					break;
				case CLK_EXTERNAL:
					uartclk = BASE_CLK_24M;
					if (CusEEbuffer.ext_clk != 0xFFFFFFFF)
						uartclk = CusEEbuffer.ext_clk;
					break;
				case CLK_1_8382M:
				default :
					uartclk = BASE_CLK_1_838235;
					break;
			}

			serial99100_ports[index].custom_baud = uartclk / (serial99100_ports[index].custom_sampling_clock *
					(serial99100_ports[index].custom_dlm * 256 + serial99100_ports[index].custom_dll));
			
		}

		if ((CusEEbuffer.cus_mod == CUS_ASUS) && (serial99100_ports[index].function_number > 1)) {
			serial99100_ports[index].oriDTR = (readl(serial99100_ports[index].bar5membase + EDE_REG) >> 24) & 0xF;
			serial99100_ports[index].oriDTR >>= serial99100_ports[index].function_number;
			serial99100_ports[index].oriDTR &= 1;
			writel((unsigned long)0x04, serial99100_ports[index].port.membase + SP_GPIO_ENABLE_REG); //Enable	
			serial99100_ports[index].oriCTS = (readl(serial99100_ports[index].port.membase + SP_GPIO_INPUT_REG) >> 2) & 1;
			serial99100_serialSettingGPIO(&serial99100_ports[index]);
			INIT_DBG(KERN_ERR "function %d oriDTR %d oriCTS %d\n", serial99100_ports[index].function_number,
					 serial99100_ports[index].oriDTR, serial99100_ports[index].oriCTS);
		}
		
		if (uart_99100_contxts[index].uart_mode == AX99100_RS485_FULL_DUPLEX ||
  			uart_99100_contxts[index].uart_mode == AX99100_RS485_HALF_DUPLEX ||
			uart_99100_contxts[index].uart_mode == AX99100_RS485_HALF_DUPLEX_ECHO ||
			uart_99100_contxts[index].uart_mode == AX99100_RS422_MODE ||
			uart_99100_contxts[index].uart_mode == AX99100_IRDA_MODE) {
			
			serial99100_ports[index].port.type = PORT_ENHANCED;
			serial99100_ports[index].rxfifotrigger = uart_99100_contxts[index].rxfifotrigger;
			serial99100_ports[index].txfifotrigger = uart_99100_contxts[index].txfifotrigger;			
		}
		
		if (serial99100_ports[index].flow_control && (serial99100_ports[index].uart_mode == AX99100_RS232_MODE)) {
			if (uart_99100_contxts[index].flow_ctrl_type == AX99100_DTR_DSR_HW_FLOWCONTROL || 
				uart_99100_contxts[index].flow_ctrl_type == AX99100_XON_XOFF_HW_FLOWCONTROL || 
				uart_99100_contxts[index].flow_ctrl_type == AX99100_RTS_CTS_HW_FLOWCONTROL) {
				serial99100_ports[index].port.type = PORT_ENHANCED;
				serial99100_ports[index].rxfifotrigger = uart_99100_contxts[index].rxfifotrigger;
				serial99100_ports[index].txfifotrigger = uart_99100_contxts[index].txfifotrigger;		
			}
		}

		if (serial99100_ports[index].port.type == PORT_ENHANCED) {
			serial99100_ports[index].rxfifotrigger = uart_99100_contxts[index].rxfifotrigger;
			serial99100_ports[index].txfifotrigger = uart_99100_contxts[index].txfifotrigger;		
		}
	}
	// printk("In %s counter1=%d-----line %d",__FUNCTION__,counter1,__LINE__);
	up(&serial99100_sem);

	return ret;
}


static struct pci_device_id serial99100_pci_tbl[] = {
	//{PCI_VENDOR_ID_NETMOS, PCI_ANY_ID, PCI_SUBDEV_ID_AX99100, PCI_SUBVEN_ID_AX99100, 0, 0, 0},	
	{0x125B, 0x9100, PCI_SUBDEV_ID_AX99100, PCI_SUBVEN_ID_AX99100_SP, 0, 0, 0},
	{0, },
};

//PCI driver remove function. Rlease the resources used by the port
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static void serial99100_remove_one(struct pci_dev *dev)
#else
static void __devexit serial99100_remove_one(struct pci_dev *dev)
#endif
{
	int i;
	unsigned long base;
	struct uart_99100_port *uart=NULL;
	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	if  (dev->subsystem_device != PCI_SUBVEN_ID_AX99100_SP) {
		dev_err(&dev->dev, "Not AX99100 SP device when remove!\n");
		return;
	} 
	
	base = pci_resource_start(dev, FL_BASE1);

	for (i = 0; i < UART99100_NR; i++){
		if(serial99100_ports[i].port.mapbase == base){
			uart=&serial99100_ports[i];
			break;
		}
	}

	if(uart){
		//printk("uart->port.irq=%d, &serial99100_ports=%p\n",uart->port.irq,&serial99100_ports[i]);
		//Free the IRQ
		//free_irq(uart->port.irq,&serial99100_ports[i]);
		free_irq(dev->irq,&serial99100_ports[i]);
		
		
#if INT_TYPE	// 0: using legacy interrupt	
		// 1: using msi interrupt	
		//printk("In %s -----line %d\n",__FUNCTION__,__LINE__);	
		pci_disable_msi(dev);
		//printk("In %s -----line %d\n",__FUNCTION__,__LINE__);
#endif		

		DEBUG("value at address 3FC is %x\n",readl(uart->port.membase + 0x3FC));
		writel(1,uart->port.membase + 0x3FC);
		DEBUG("value at address 3FC after configuring is %x\n",readl(uart->port.membase + 0x3FC));
	
		down(&serial99100_sem);
		uart_remove_one_port(&starex_serial_driver, &uart->port);
		uart->port.dev = NULL;		
		up(&serial99100_sem);
		

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,15,0)
		pci_free_consistent(dev,DMA_TX_BUFFER_SZ,uart->dma_tx_buf_v,uart->dma_tx_buf_p);
		pci_free_consistent(dev,DMA_TX_BUFFER_SZ,uart->dma_tx_buf_v_start,uart->dma_tx_buf_p_start);
		pci_free_consistent(dev,DMA_RX_BUFFER_SZ,uart->dma_rx_buf_v,uart->dma_rx_buf_p);
#else
		dma_free_coherent(&dev->dev,DMA_TX_BUFFER_SZ,uart->dma_tx_buf_v,uart->dma_tx_buf_p);
		dma_free_coherent(&dev->dev,DMA_TX_BUFFER_SZ,uart->dma_tx_buf_v_start,uart->dma_tx_buf_p_start);
		dma_free_coherent(&dev->dev,DMA_RX_BUFFER_SZ,uart->dma_rx_buf_v,uart->dma_rx_buf_p);
#endif		
		pci_disable_device(dev);

		//Initialise the uart_99100_port arrays port specific element to the default state
		serial99100_init_port(&serial99100_ports[uart->port.line]);
	}
	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
}

//PCI drivers probe function
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static int serial99100_probe(struct pci_dev *dev,
			     const struct pci_device_id *ent)
#else
static int __devinit serial99100_probe(struct pci_dev *dev,
				       const struct pci_device_id *ent)
#endif
{
	struct uart_port serial_port;
	unsigned long base, len;
	int retval;
	
#if INT_TYPE	// 0: using legacy interrupt	
		// 1: using msi interrupt
	int ret;	
#endif	

	retval = pci_enable_device(dev);
	//printk("In %s retval=%d-----line %d\n",__FUNCTION__,retval,__LINE__);
	
	if (retval) {
		dev_err(&dev->dev, "Device enable FAILED, line %d\n",__LINE__);
		return retval;
	}

	//To verify whether it is a serial communication hardware
	if ((((dev->class >> 8) != PCI_CLASS_COMMUNICATION_SERIAL) &&
		((dev->class >> 8) != PCI_CLASS_COMMUNICATION_MODEM)) ||
		(dev->class & 0xff) > 6){
		dev_err(&dev->dev, "Not a serial communication hardware\n");
		retval = -ENODEV;
		goto disable;
	}

	//To verify whether it is a AX99100 type BARs
#if defined(__x86_64__) || defined(__amd64__)
	if(((pci_resource_flags(dev,FL_BASE0) & BAR_FMT) ^ BAR_IO) ||
			((pci_resource_flags(dev,FL_BASE2) & BAR_FMT) ^ BAR_MEM) ||
			((pci_resource_flags(dev,FL_BASE4) & BAR_FMT) ^ BAR_MEM)) {
#else
	if(((pci_resource_flags(dev,FL_BASE2) & BAR_FMT) ^ BAR_MEM) ||
	   ((pci_resource_flags(dev,FL_BASE4) & BAR_FMT) ^ BAR_MEM)) {
#endif
		dev_err(&dev->dev, "Not a AX99100 type device\n");
		retval = -ENOMEM;
		goto disable;
	}
	
	pci_set_master(dev);	
/*
	retval = pci_set_master(dev);	
	if (retval) {
		dev_err(&dev->dev, "Device enable FAILED\n");
		return retval;
	}

*/

	memset(&serial_port, 0, sizeof(struct uart_port));
	memset(&CusEEbuffer,0, CUSTOM_EEPROM_LEN);

	serial_port.flags = UPF_SHARE_IRQ | UPF_SKIP_TEST;
	serial_port.uartclk = DEFAULT99100_BAUD * 16;
	serial_port.irq = dev->irq;
	serial_port.dev = &dev->dev;
	
	len =  pci_resource_len(dev, FL_BASE1);
	base = pci_resource_start(dev, FL_BASE1);
	serial_port.mapbase = base;
	serial_port.membase = ioremap(base,len);
	

	DEBUG("membase=0x%x\n mapbase=0x%x\n",(unsigned int)serial_port.membase,(unsigned int)serial_port.mapbase);
	DEBUG("value at address 3FC is %x\n",readl(serial_port.membase + 0x3FC));
	writel(0,serial_port.membase + 0x3FC);
	DEBUG("value at address 3FC after configuring is %x\n",readl(serial_port.membase + 0x3FC));
#if defined(__arm__) || defined(__aarch64__)
	serial_port.iobase = 0;
#else
	base = pci_resource_start(dev,FL_BASE0);
	serial_port.iobase = base;
#endif


	retval = serial99100_register_port(&serial_port,dev);
	//printk("In %s retval=%d-----line %d\n",__FUNCTION__,retval,__LINE__);
	if (retval < 0){
		DEBUG(KERN_WARNING "Couldn't register serial port %s, retval=%d: \n", pci_name(dev),retval);
		goto disable;	
	}	

#if INT_TYPE	// 0: using legacy interrupt	
		// 1: using msi interrupt
	//printk("In %s pci_msi_enabled=%d-----line %d\n",__FUNCTION__,pci_msi_enabled(),__LINE__);
	if (pci_msi_enabled()){
		ret = pci_enable_msi(dev);
		//printk("In %s retval=%d-----line %d\n",__FUNCTION__,ret,__LINE__);		
		if (ret) {
			dev_err(&dev->dev, "MSI enable FAILED, line %d\n",__LINE__);
			return ret;
		}		
	}else{
		printk("In %s pci_msi_enabled() retval=%d-----line %d\n",__FUNCTION__,ret,__LINE__);
	}	
#endif	

//Register an ISR
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	if ((retval = request_irq(dev->irq, serial99100_interrupt,SA_SHIRQ,"AX99100",&serial99100_ports[retval]))) 
		goto disable;
#else
	if ((retval = request_irq(dev->irq, serial99100_interrupt,IRQF_SHARED,"AX99100",&serial99100_ports[retval]))) {
		//printk("In %s retval=%d-----line %d\n",__FUNCTION__,retval,__LINE__);
		goto disable;
	}
	//printk("In %s retval=%d-----line %d\n",__FUNCTION__,retval,__LINE__);
	//printk("dev->irq=%d, &serial99100_ports=%p\n",dev->irq,&serial99100_ports[retval]);
#endif
	
	//printk("In %s retval=%d-----line %d\n",__FUNCTION__,retval,__LINE__);
	printk(version);
	return 0;	
	 
disable:
	return retval;
}

static int serial99100_suspend(struct pci_dev *dev, pm_message_t state)
{
	u16 data;
	u32 val;
	int i;
	unsigned long base;
	struct uart_99100_port *uart = NULL;

	base = pci_resource_start(dev, FL_BASE1);
	for (i = 0; i < UART99100_NR; i++) {
		if (serial99100_ports[i].port.mapbase == base) {
			uart = &serial99100_ports[i];
			break;
		}
	}

	if (uart) {
		// Disable all interrupt
		uart->mcr = serial_in(uart, UART_MCR);
		serial_out(uart, UART_MCR, serial_in(uart, UART_MCR) | UART_MCR_OUT2);

		uart->ier = serial_in(uart, UART_IER);
		serial_out(uart, UART_IER, 0);

		uart->gier = readl(uart->port.membase + REG_GLBL_IER);
		writel(0x0, uart->port.membase + REG_GLBL_IER);

		// Enable the remote wake up function
		val=readl(uart->port.membase + SP_SETTING_REG0);
		val |= COM_REMOTE_WAKE_EN | COM_REMOTE_WAKE_ALL;
		writel(val, uart->port.membase + SP_SETTING_REG0);

		// Turn off baud clock
		val=readl(uart->port.membase + 0x284);
		val |= 0x10;
		writel(val, uart->port.membase + 0x284);
	}

	suspend_count++;

	// Enable PME and D3
	if (dev->pm_cap) {
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
		pci_write_config_word(dev, dev->pm_cap + PCI_PM_CTRL, data | PCI_PM_CTRL_PME_ENABLE | PCI_D3hot);
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
	}

	pci_disable_device(dev);
	pci_save_state(dev);
	pci_enable_wake(dev, PCI_D3hot, 1);
	pci_set_power_state(dev, PCI_D3hot);

	return 0;
};

static int serial99100_resume(struct pci_dev *dev)
{
	u16 data;
	u32 val;
	int i;
	unsigned long base;
	struct uart_99100_port *uart = NULL;
	
	pci_set_power_state(dev, PCI_D0);
	pci_restore_state(dev);
	pci_enable_wake(dev, PCI_D0, 0);

	if (pci_enable_device(dev) < 0) {
		printk(KERN_ERR"pci_enable_device failed, ""disabling device\n");
		return -EIO;
	}

	pci_set_master(dev);

	base = pci_resource_start(dev, FL_BASE1);
	for (i = 0; i < UART99100_NR; i++) {
		if (serial99100_ports[i].port.mapbase == base) {
			uart = &serial99100_ports[i];
			break;
		}
	}

	if (uart) {
		// Enable interrupts
		writel(uart->gier, uart->port.membase + REG_GLBL_IER);
		serial_out(uart, UART_IER, uart->ier);
		serial_out(uart, UART_MCR, uart->mcr);

		// Turn on baud clock
		val=readl(uart->port.membase + 0x284);
		val &= ~(0x10);
		writel(val, uart->port.membase + 0x284);
	}

	suspend_count--;

	// Disable PME
	if (dev->pm_cap) {
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
		pci_write_config_word(dev, dev->pm_cap + PCI_PM_CTRL, data & (~PCI_PM_CTRL_PME_ENABLE));
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
	}

	return 0;
};

	
static struct pci_driver starex_pci_driver = {
	.name		= "AX99100",
	.probe		= serial99100_probe,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	.remove		= serial99100_remove_one,
#else
	.remove		= __devexit_p(serial99100_remove_one),
#endif
	.id_table	= serial99100_pci_tbl,
	.suspend	= serial99100_suspend,
	.resume		= serial99100_resume,
};


//Drivers entry function. register with the pci core and the serial core
static int __init serial99100_init(void)
{
	int ret;

	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);
	
	

	serial99100_init_ports();
	
	ret = uart_register_driver(&starex_serial_driver);
	if (ret){
		DEBUG("In %s uart_register_driver FAILED\n",__FUNCTION__);
		return ret;
	}		
	#if FUNC_SELECT && (defined(__x86_64__) || defined(__amd64__))
	ret = spi99100_init();
	if (ret < 0){
		DEBUG("In %s spi_register_driver FAILED\n",__FUNCTION__);
		uart_unregister_driver(&starex_serial_driver);
	}
	#endif		


	ret = pci_register_driver(&starex_pci_driver);
	if (ret < 0){
		DEBUG("In %s pci_register_driver FAILED\n",__FUNCTION__);
		uart_unregister_driver(&starex_serial_driver);
	}
	
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
	return ret;
}

//Drivers exit function. Unregister with the PCI core as well as serial core
static void __exit serial99100_exit(void)
{
	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	pci_unregister_driver(&starex_pci_driver);
	#if FUNC_SELECT && (defined(__x86_64__) || defined(__amd64__))
	spi99100_exit();
	#endif	
	
	uart_unregister_driver(&starex_serial_driver);
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);	
}

module_init(serial99100_init);
module_exit(serial99100_exit);

MODULE_DEVICE_TABLE(pci, serial99100_pci_tbl);
MODULE_DESCRIPTION("ASIX AX99100 Driver Module");
MODULE_LICENSE("GPL");
