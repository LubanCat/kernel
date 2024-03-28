/*
 *  linux/drivers/serial/99xx.h
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
#ifndef _AX99100_SP_MAIN_H
#define _AX99100_SP_MAIN_H

#include <linux/types.h>

#define DRV_VERSION	"1.9.0"


#define FUNC_SELECT 1	// 0: Serial Port only
			// 1: Serial Port+SPI
#define INT_TYPE 0	// 0: using legacy interrupt	
			// 1: using msi interrupt






#define DIV 1
#define CUS_ASUS		1
#define EEPROM8BIT		1

#define EN_CUS_BAUD		0x80

#define UART_CAP_FIFO		(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR		(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP		(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE		(1 << 11)	/* MCR-based hw flow control */
#define UART_CAP_UUE		(1 << 12)	/* UART needs IER bit 6 set (Xscale) */

#define REG_TX_DMA_START_ADDRESS_LOW			((0x80)/DIV)
#define REG_TX_DMA_START_ADDRESS_HIGH			((0x84)/DIV)
#define REG_TX_DMA_LENGTH				((0x88)/DIV)
#define REG_TX_DMA_START				((0x8C)/DIV)
#define REG_TX_DMA_STOP					((0x90)/DIV)
#define REG_TX_DMA_STATUS				((0x94)/DIV)
#define REG_TX_BYTES_TRANSFERRED			((0x98)/DIV)

#define REG_RX_DMA_START_ADDRESS_LOW			((0x100)/DIV)
#define REG_RX_DMA_START_ADDRESS_HIGH			((0x104)/DIV)
#define REG_RX_DMA_LENGTH				((0x108)/DIV)
#define REG_RX_DMA_START				((0x10C)/DIV)
#define REG_RX_DMA_STOP					((0x110)/DIV)
#define REG_RX_DMA_STATUS				((0x114)/DIV)
#define REG_RX_BYTES_NEED_TO_RECV			((0x118)/DIV)
#define REG_RX_MEM_4K_LMT				((0x11C)/DIV)

// Serial port interrupt register
#define REG_GLBL_ISR			((0x3A0)/DIV)
#define REG_GLBL_ICLR			((0x3A4)/DIV)
#define REG_GLBL_IER			((0x3A8)/DIV)

#define TX_DMA_START_BIT		1<<0
#define TX_DMA_STOP_BIT			1<<0
#define TX_DMA_STOP_DONE_BIT 		1<<0
#define TX_DMA_DONE			1<<0
#define TX_DMA_BUSY			1<<0
#define TX_DMA_RDY			1<<0
#define TX_DMA_SR_COMPLETE		0x01
#define TX_DMA_SR_STOP_BIT		0x02

#define RX_DMA_START_BIT		1<<0
#define RX_DMA_STOP_BIT			1<<0
#define RX_DMA_STOP_DONE_BIT 		1<<0
#define RX_DMA_DONE			1<<0
#define RX_DMA_BUSY			1<<0
#define RX_DMA_RDY			1<<0
#define RX_DMA_SR_COMPLETE		0x01
#define RX_DMA_SR_STOP_BIT		0x02

#define NEED_PART_DONE			0x01
#define NOT_NEED_PART_DONE		0x00

#define SPINTR_ISR			0x01
#define SPINTR_TXDMA_ISR		0x02
#define SPINTR_RXDMA_COMPLETE_ISR	0x04
#define SPINTR_RXDMA_DONE		0x08
#define SPINTR_RXDMA			0x0C
#define GPIO_ISR			0x10

#define SERIAL_450MODE		0x5470
#define SERIAL_550MODE		0x5471
#define SERIAL_550AMODE		0x5472
#define SERIAL_650MODE		0x5473
#define SERIAL_750MODE		0x5474
#define SERIAL_850MODE		0x5475
#define SERIAL_950MODE		0x5476

// CLOCK_SOURCE
#define CLK_EXTERNAL	2
#define CLK_125M	1
#define CLK_1_8382M	0

// UART CLOCK
#define BASE_CLK_1_838235	1838235
#define BASE_CLK_125M		125000000
#define BASE_CLK_24M		24000000

//EDS
#define EDS_ECSS		(0x01000000)
#define EDS_EOES		(0x02000000)

// CLOCK setting mask
#define CLK_MASK		0xFFFF0FF8

// Default xon/xoff characters.
#define SERIAL_DEF_XON		0x11
#define SERIAL_DEF_XOFF		0x13

// UART mode
#define AX99100_RS232_MODE			0
#define AX99100_RS422_MODE			1
#define AX99100_RS485_HALF_DUPLEX		2
#define AX99100_RS485_HALF_DUPLEX_ECHO		4
#define AX99100_RS485_FULL_DUPLEX		5
#define AX99100_RS485_FULL_DUPLEX_TXEN		10
#define AX99100_DTR_DSR_HW_FLOWCONTROL 		6
#define AX99100_XON_XOFF_HW_FLOWCONTROL		7
#define AX99100_RTS_CTS_HW_FLOWCONTROL		8
#define AX99100_IRDA_MODE			9

#define PORT_ENHANCED				14

// GPIO mode
#define GPIO_4MP_MODE			7 //4S
#define GPIO_2S_1SPI_MODE		6 //2S
#define GPIO_2S_2MP_MODE		5 //4S
#define GPIO_2MP_1SPI_MODE		4 //2S
#define GPIO_4S_MODE			3 //4S
#define GPIO_2MP_1P_MODE		2 //2S
#define GPIO_2S_1P_MODE			1 //2S
#define GPIO_LB_MODE			0 //4S

#define AX99100_SERIAL_PORT	0
#define AX99100_MF_PORT		1

//CommSet Registers
//99100
//Common Registers Set (memory mapped)
#define SP_SETTING_REG0			((0x200)/DIV)
#define SP_SETTING_REG1			((0x204)/DIV)
#define SP_GPIO_ENABLE_REG		((0x208)/DIV)
#define SP_GPIO_OUTPUT_REG		((0x20C)/DIV)
#define SP_GPIO_PULLUP_REG		((0x210)/DIV)
#define SP_BR_CLK_SEL_REG		((0x214)/DIV)
#define SP_GPIO_INPUT_REG		((0x218)/DIV)
#define SP_SETTING_REG2			((0x21C)/DIV)
#define SP_TX_FIFO_COUNTER		((0x220)/DIV)
#define SP_RX_FIFO_COUNTER		((0x224)/DIV)
#define SP_TX_TRIG_LVL			((0x228)/DIV)
#define SP_RX_TRIG_LVL			((0x22C)/DIV)
#define SP_FLOW_LOW_CTRIG_LVL		((0x230)/DIV)
#define SP_FLOW_UPP_CTRIG_LVL		((0x234)/DIV)
#define SER_SOFT_RESET_REG		((0x238)/DIV) 
#define PP_DIV_REG			((0x250)/DIV)
#define PP_RX_TRIG_LEVEL		((0x254)/DIV)
#define PP_TX_TRIG_LEVEL		((0x258)/DIV)
#define PP_PERI_HOST_HIGH_REG		((0x25C)/DIV)


#define COM_DMA_MODE_EN			0x10000000
#define COM_550EX_MODE_EN		0x00001000

#define COM_REMOTE_WAKE_EN		0x08000000
#define COM_REMOTE_WAKE_ALL		0x07C00000
#define COM_REMOTE_WAKE_RX		0x04000000
#define COM_REMOTE_WAKE_DSR		0x02000000
#define COM_REMOTE_WAKE_RI		0x01000000
#define COM_REMOTE_WAKE_DCD		0x00800000
#define COM_REMOTE_WAKE_CTS		0x00400000

#define SP_TX_FIFO_COUNTER		((0x220)/DIV)
#define SP_RX_FIFO_COUNTER		((0x224)/DIV)
#define SER_SOFT_RESET_REG		((0x238)/DIV)

#define EDS_REG				((0x3D4)/DIV)
#define EDE_REG				((0x3D8)/DIV)

#define PCI_SUBVEN_ID_AX99100_SP	0x1000
#define PCI_SUBDEV_ID_AX99100	  	0xa000	

#define BAR_IO		0x001
#define BAR_MEM 	0x000
#define BAR_FMT  	0x001

#define DISPORT_NUM	2

#define DMA_TX_BUFFER_SZ 	4096
#define DMA_RX_SZ 			65535
#define DMA_RX_BUFFER_SZ 	DMA_RX_SZ * 2

#define REG_I2CCR		((0x0C8)/DIV)
#define REG_I2CSCLPR		((0x0CC)/DIV)
#define REG_I2CSCLCR		((0x0D0)/DIV)
	#define	I2CSCLCR_RCVF		((u32)1 << 30)
	#define	I2CSCLCR_NACK		((u32)1 << 31)
	#define I2CSCLCR_CHECK		((u32)3 << 30)
#define REG_I2CBFTR			((0x0D4)/DIV)
#define CUSTOM_EE_RESVED_LEN	41
#define CUSTOM_EEPROM_LEN	64

#define REG_GPIOPIN		((0x3C0)/DIV)
#define REG_GPIODIR		((0x3C4)/DIV)

#define DTR_UNKNOWN		8
#define CTS_UNKNOWN		8

// 9bit mode
#define NMR_NBE		0x01
#define NMR_NBIE	0x02
#define NMR_AME		0x04
	

struct old_serial_port {
	unsigned int 	uart;
	unsigned int 	baud_base;
	unsigned int	port;
	unsigned int	irq;
	unsigned int 	flags;
	unsigned char 	hub6;
	unsigned char 	io_type;
	unsigned char 	*iomem_base;
	unsigned short 	iomem_reg_shift;
};

struct serial99100_config {
	unsigned short	fifo_size;
	unsigned short	tx_loadsz;
	unsigned char	fcr;
	unsigned int	flags;
};


struct port_setting {
	u8 en_cusbaud_clksrc;
	u8 dll;
	u8 dlm;
	u8 sample_rate;
};

struct custom_eeprom {
	u16 cus_mod;
	struct port_setting pt_setting[4];
	u32 ext_clk;
	/* if reservied is modified, CUSTOM_EE_RESVED_LEN need to be change */
	u8 reserved[CUSTOM_EE_RESVED_LEN];
	u8 chksum;
} __attribute__((packed));


#if defined(__i386__) && (defined(CONFIG_M386) || defined(CONFIG_M486))
#define _INLINE_ inline
#else
#define _INLINE_
#endif

#define DEFAULT99100_BAUD 115200
#endif
