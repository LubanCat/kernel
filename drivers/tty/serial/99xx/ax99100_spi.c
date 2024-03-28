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
 * This code is modified to support ASIX 99100 series serial devices
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
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/8250_pci.h>
#include <linux/interrupt.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/netlink.h>
#include <net/sock.h>

#include "ax99100_spi.h"
#include "ax99100_sp.h"
#include "ioctl.h"



static char version_spi[] =

KERN_INFO "ASIX AX99100 PCIe Bridge to SPI:v" DRV_VERSION

	"    http://www.asix.com.tw\n";


#if 0
#define DEBUG(fmt...)	printk(KERN_ERR fmt)
#else
#define DEBUG(fmt...)	;
#endif
/* ================================================================ */
int spi_suspend_count;
#define NUM_DEVICE 	16
static unsigned int spi_major = 241;
static unsigned int spi_min_count = 0;
/* device Class */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,19,0)
static char *ax_devnode(struct device *dev, umode_t *mode)
#else 
static char *ax_devnode(const struct device *dev, umode_t *mode)
#endif
{
	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}

struct class ax_spi_class = {
	.name		= CLASS_NAME,
	.devnode	= ax_devnode,
};
extern struct net 	init_net;
struct sock 	 	*nl_sk = NULL;

struct spi_99100 {
	struct cdev 		spi;

	unsigned int		dev_major;
	unsigned int		dev_minor;
	char 			dev_name[64];
	
	unsigned long		iobase0;			// bar0
	unsigned char __iomem	*membase[2];			// 0: bar1 1:bar5
	resource_size_t		mapbase[2];			// for ioremap
	

	unsigned int		irq;

	
	char 	*		tx_dma_v;			//Virtual Address of DMA Buffer for TX
	dma_addr_t 		tx_dma_p;			//Physical Address of DMA Buffer for TX
	char 	*		rx_dma_v;			//Virtual Address of DMA Buffer for RX
	dma_addr_t 		rx_dma_p;			//Physical Address of DMA Buffer for RX

	int 			tool_pid;	
};

static int		 init_cdev = 0;
static struct spi_99100* axspi_device[NUM_DEVICE];

/* IOCTL*/
PSPI_REG	reg[NUM_DEVICE];
PMMAP_SPI_REG	reg_m[NUM_DEVICE];
PSPI_DMA 	dma[NUM_DEVICE];
/* ================================================================ */

/* memmap read reg */
static _INLINE_ u32 ax99100_dread_mem_reg(int offset, int bar, int line)
{
       return readl(axspi_device[line]->membase[bar] + offset);	
}

/* memmap write reg */
static _INLINE_ void ax99100_dwrite_mem_reg(int offset, int value, int bar, int line)
{
	writel(value, axspi_device[line]->membase[bar] + offset);	
}

/* iomap read reg */
static _INLINE_ u8 ax99100_dread_io_reg(unsigned char offset, int line)
{
       return inb(axspi_device[line]->iobase0 + offset);	
}

/* iomap write reg */
static _INLINE_ void ax99100_dwrite_io_reg(unsigned char offset,unsigned char value, int line)
{
      outb(value, axspi_device[line]->iobase0 + offset);
}

/******************************************************
 * 
 * File Operation
 * 
 * ****************************************************/

static long spi99100_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int line = MINOR(*((dev_t*)filp->private_data));
	unsigned long	length;	
	PSPI_REG	preg = reg[line];
	PMMAP_SPI_REG	preg_m = reg_m[line];
	PSPI_DMA 	pdma = dma[line];
	struct spi_99100* axspi = NULL;

	axspi = axspi_device[line];	
	
	switch (cmd) {	
	case IOCTL_IO_SET_REGISTER:
	{			
		if(copy_from_user(preg, (PSPI_REG)arg, sizeof(SPI_REG)))
			return -ENOIOCTLCMD;
		
		ax99100_dwrite_io_reg(preg->Offset, preg->Value, line);

		DEBUG("IOCTL_IO_SET_REGISTER Offset: 0x%x Value: 0x%x\n", preg->Offset, preg->Value);
		break;
	}
	case IOCTL_IO_READ_REGISTER:
	{			
		DEBUG("IOCTL_IO_READ_REGISTER\n");
		if(copy_from_user(preg, (PSPI_REG)arg, sizeof(SPI_REG)))
			return -ENOIOCTLCMD;
		
		preg->Value = ax99100_dread_io_reg(preg->Offset, line);
		
		if(copy_to_user((PSPI_REG)arg, preg, sizeof(SPI_REG)))
			return -ENOIOCTLCMD;
		
		
		break;
	}
	case IOCTL_MEM_SET_REGISTER:
	{
		DEBUG("IOCTL_MEM_SET_REGISTER\n");
		if(copy_from_user(preg_m, (PMMAP_SPI_REG)arg, sizeof(MMAP_SPI_REG)))
			return -ENOIOCTLCMD;
		
		ax99100_dwrite_mem_reg(preg_m->Offset, preg_m->Value, preg_m->Bar, line);
		break;
	}
	case IOCTL_MEM_READ_REGISTER:
	{
		DEBUG("IOCTL_MEM_READ_REGISTER\n");
		if(copy_from_user(preg_m, (PMMAP_SPI_REG)arg, sizeof(MMAP_SPI_REG)))
			return -ENOIOCTLCMD;
	  
		preg_m->Value = ax99100_dread_mem_reg(preg_m->Offset, preg_m->Bar, line);
		
		if(copy_to_user((PMMAP_SPI_REG)arg, preg_m, sizeof(MMAP_SPI_REG)))
			return -ENOIOCTLCMD;
		
		break;
	}
	case IOCTL_SET_TX_DMA_REG:
	{
		DEBUG("IOCTL_SET_TX_DMA_REG\n");
		if(copy_from_user(&length, (unsigned long *)arg, sizeof(unsigned long)))
			return -ENOIOCTLCMD;
		
		ax99100_dwrite_mem_reg(REG_TDMASAR0, axspi->tx_dma_p, BAR1, line);
		ax99100_dwrite_mem_reg(REG_TDMASAR1, 0x0, BAR1, line);
		ax99100_dwrite_mem_reg(REG_TDMALR, length, BAR1, line);
		ax99100_dwrite_mem_reg(REG_TDMASTAR, START_DMA, BAR1, line);
		
		break;
	}
	case IOCTL_SET_RX_DMA_REG:
	{
		DEBUG("IOCTL_SET_RX_DMA_REG\n");
		if(copy_from_user(&length, (unsigned long *)arg, sizeof(unsigned long)))
			return -ENOIOCTLCMD;	  
		
		ax99100_dwrite_mem_reg(REG_RDMASAR0, axspi->rx_dma_p, BAR1, line);
		ax99100_dwrite_mem_reg(REG_RDMASAR1, 0x0, BAR1, line);
		ax99100_dwrite_mem_reg(REG_RDMALR, length, BAR1, line);
		ax99100_dwrite_mem_reg(REG_RDMASTAR, START_DMA, BAR1, line);
		
		break;
	}
	case IOCTL_TX_DMA_WRITE:
	{
		DEBUG("IOCTL_TX_DMA_WRITE\n");
		if(copy_from_user(pdma, (PSPI_DMA)arg, sizeof(SPI_DMA)))
			return -ENOIOCTLCMD;
		
		memcpy_toio(axspi->tx_dma_v, pdma->Buffer, pdma->Length);
		
		break;
	}
	case IOCTL_RX_DMA_READ:
	{		
		DEBUG("IOCTL_RX_DMA_READ\n");
		if(copy_from_user(pdma, (PSPI_DMA)arg, sizeof(SPI_DMA)))
			return -ENOIOCTLCMD;
		
		memcpy_fromio(pdma->Buffer, axspi->rx_dma_v, pdma->Length);		
		
		if(copy_to_user((PSPI_DMA)arg, pdma, sizeof(SPI_DMA)))
			return -ENOIOCTLCMD;
		
		break;
	}

	default:
		return -ENOIOCTLCMD;	
	}
	return 0;
}

static int spi99100_open (struct inode *inop, struct file *filp)
{
	struct cdev*	cdev = inop->i_cdev;
	int 		line = MINOR(cdev->dev);

	filp->private_data = &cdev->dev;	

	reg[line] = kmalloc(sizeof(SPI_REG), GFP_KERNEL);
	if (reg[line] == NULL) {
		goto err;
	}
	memset(reg[line], 0xFF, sizeof(SPI_REG));	
	
	reg_m[line] = kmalloc(sizeof(MMAP_SPI_REG), GFP_KERNEL);
	if (reg_m[line]== NULL) {
		goto err;
	}
	memset(reg_m[line], 0xFF, sizeof(MMAP_SPI_REG));	
	
	dma[line] = kmalloc(sizeof(SPI_DMA), GFP_KERNEL);
	if (dma[line]== NULL) {
		goto err;
	}
	memset(dma[line], 0xFF, sizeof(SPI_DMA));	

	return 0;
err:
	if (!reg[line])
		kfree(reg[line]);
	if (!reg_m[line]) 
		kfree(reg_m[line]);
	if (!dma[line])
		kfree(dma[line]);
	return -1;
}

static int spi99100_release (struct inode *inop, struct file *filp)
{
	int line = MINOR(*((dev_t*)filp->private_data));

	kfree(reg[line]);
	kfree(reg_m[line]);
	kfree(dma[line]);
	
	return 0;
}

static struct file_operations bridge_fops = {
	.owner		=	THIS_MODULE,	
	.unlocked_ioctl	=	spi99100_ioctl,	
	.open		=	spi99100_open,
	.release	=	spi99100_release,
};
/********************************************************************
 * 
 * NETLINK
 * 
 ********************************************************************/
void netlink_get(struct sk_buff *__skb) {
 	
	struct nlmsghdr *nlh = NULL;
	char str[100];	
	int line;

        if (__skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(__skb);
                memcpy(str, NLMSG_DATA(nlh), sizeof(str));
                printk("%s: received netlink message payload:%s\n",
		__FUNCTION__, (char*)NLMSG_DATA(nlh));
		line = str[0] - '0';
		axspi_device[line]->tool_pid = nlh->nlmsg_pid;
	}
}

void netlink_sendmsg(struct spi_99100* axspi)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int		ret;
	int 		pid = axspi->tool_pid;
	
	char msg[30] = "Interrupt Complete!";

	if (!nl_sk)
		return;

	skb = nlmsg_new(MAX_PAYLOAD_SIZE, GFP_KERNEL);

	if (!skb)
		printk(KERN_ERR "nlmsg_new error");

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_PAYLOAD_SIZE, 0);

	memcpy(NLMSG_DATA(nlh), msg, sizeof(msg));	

	ret = netlink_unicast(nl_sk, skb, pid, MSG_DONTWAIT);	
	if (ret < 0) {
		printk("Netlink sends failed.\n");
	}
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)	
struct netlink_kernel_cfg netlink_kerncfg = {		
		        .input = netlink_get,
		    };
#endif
/********************************************************************
 * 
 * PCIE FUNCTION
 * 
 ********************************************************************/

//PCI driver remove function. Rlease the resources used by the port
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static void spi99100_remove_one(struct pci_dev *dev)
#else
static void __devexit spi99100_remove_one(struct pci_dev *dev)
#endif
{
	dev_t	device;
	struct spi_99100*	axspi = NULL;

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);	

	axspi = (struct spi_99100*)pci_get_drvdata(dev);
	
	device = MKDEV(axspi->dev_major, axspi->dev_minor);

	/* DMA free */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,15,0)			
	pci_free_consistent(dev, DMA_BUFFER_SZ, axspi->tx_dma_v,axspi->tx_dma_p);
	pci_free_consistent(dev, DMA_BUFFER_SZ, axspi->rx_dma_v,axspi->rx_dma_p);
#else
	dma_free_coherent(&dev->dev, DMA_BUFFER_SZ, axspi->tx_dma_v,axspi->tx_dma_p);
	dma_free_coherent(&dev->dev, DMA_BUFFER_SZ, axspi->rx_dma_v,axspi->rx_dma_p);
#endif	
	if  (dev->subsystem_device != PCI_SUBVEN_ID_AX99100_SPI) {
		dev_err(&dev->dev, "Not AX99100 SPI device when remove!\n");
		return;
	} 
	
	/* Remove Char Device & Class */
	device_destroy(&ax_spi_class, device);
	cdev_del(&axspi->spi);	
	if (init_cdev == 1) {
		dev_t	device_tmp = MKDEV(axspi->dev_major, 0);
		unregister_chrdev_region(device_tmp, NUM_DEVICE);
		init_cdev = 0;
	}
	
	/* Remove netlink setting */
	if (nl_sk != NULL) {
		sock_release(nl_sk->sk_socket);	
		nl_sk = NULL;
	}

	free_irq(axspi->irq, axspi);
#if INT_TYPE	// 0: using legacy interrupt	
		// 1: using msi interrupt	
		//printk("In %s -----line %d\n",__FUNCTION__,__LINE__);	
		pci_disable_msi(dev);
		//printk("In %s -----line %d\n",__FUNCTION__,__LINE__);
#endif	
	pci_disable_device(dev);

	kfree(axspi);
	
	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
}

void init_local_data(struct pci_dev *dev)
{
	struct spi_99100*	axspi = NULL;
	unsigned long		base, len;

	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);	

	axspi = (struct spi_99100*)pci_get_drvdata(dev);

	/* memory map  */
	/* bar1 */
	len =  pci_resource_len(dev, FL_BASE1);
	base = pci_resource_start(dev, FL_BASE1);
	axspi->mapbase[0] = base;
	axspi->membase[0] = ioremap(base,len);
	/* bar5 */
	len =  pci_resource_len(dev, FL_BASE5);
	base = pci_resource_start(dev, FL_BASE5);
	axspi->mapbase[1] = base;
	axspi->membase[1] = ioremap(base,len);
	
	
	DEBUG("bar1 membase=0x%x mapbase=0x%x\n",
		(unsigned int)axspi->membase[0],(unsigned int)axspi->mapbase[0]);
	DEBUG("bar5 membase=0x%x mapbase=0x%x\n",
		(unsigned int)axspi->membase[1],(unsigned int)axspi->mapbase[1]);
	
	/* io map */
	base = pci_resource_start(dev,FL_BASE0);
	axspi->iobase0 = base;
	
	DEBUG("bar0 iobase=0x%x\n",(unsigned int)axspi->iobase0);	
	
	
	/* DMA for TX */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,15,0)	
	axspi->tx_dma_v =
		(char *)pci_alloc_consistent(dev,DMA_BUFFER_SZ,&axspi->tx_dma_p);
#else
	axspi->tx_dma_v =
		(char *)dma_alloc_coherent(&dev->dev,DMA_BUFFER_SZ,&axspi->tx_dma_p, GFP_ATOMIC);		
#endif	

	memset(axspi->tx_dma_v,0,DMA_BUFFER_SZ);
	
	DEBUG("tx_dma_v=0x%x tx_dma_p=0x%x\n",(unsigned int)axspi->tx_dma_v,
		(unsigned int)axspi->tx_dma_p);
	
	/* DMA for RX */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,15,0)			
	axspi->rx_dma_v =
		(char *)pci_alloc_consistent(dev,DMA_BUFFER_SZ,&axspi->rx_dma_p);
#else
	axspi->rx_dma_v =
		(char *)dma_alloc_coherent(&dev->dev,DMA_BUFFER_SZ,&axspi->rx_dma_p, GFP_ATOMIC);		
#endif
	memset(axspi->rx_dma_v,0,DMA_BUFFER_SZ);
	
	DEBUG("rx_dma_v=0x%x rx_dma_p=0x%x\n",(unsigned int)axspi->rx_dma_v,
		(unsigned int)axspi->rx_dma_p);
	

	
	
	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static irqreturn_t spi99100_interrupt(int irq, void *dev_id)
#else
static irqreturn_t spi99100_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
	int line = 0;
	int handled = 0;
	unsigned long isr_status = 0;
	unsigned long sdcr = 0;	
	struct spi_99100* axspi = (struct spi_99100*)dev_id;

	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);

	line = axspi->dev_minor;	

	DEBUG("In %s---------line: %d\n",__FUNCTION__, line);
	
	
	/* Read SDCR */	
	sdcr = ax99100_dread_io_reg(REG_SDCR, line);	
	if (!(sdcr & INTERRUPT_ENABLE_MASK))
		return IRQ_RETVAL(0);
	
	/* Read ISR */	
	isr_status = ax99100_dread_io_reg(REG_SPIMISR, line);	
	if (!(isr_status & INTERRUPT_MASK))
		return IRQ_RETVAL(0);

	/* Clear ISR */
	ax99100_dwrite_io_reg(REG_SPIMISR, isr_status, line);

	DEBUG("In %s---------ISR: 0x%x\n",__FUNCTION__, (int)isr_status);

	if (isr_status & SPIMISR_STC) {
		DEBUG("SPI Transceiver Complete\n");
		netlink_sendmsg(axspi_device[line]);
		handled = 1;
	}
	if (isr_status & SPIMISR_STERR) {
		DEBUG("SPI Transceiver Error Indication\n");
		handled = 1;
	}

	DEBUG("In %s---------handled: %d\n",__FUNCTION__, handled);

	DEBUG("In %s--------------------------------------END\n",__FUNCTION__);

	return IRQ_RETVAL(handled);
}

/* helper function to reset device connect to spi */
void spi_reset(int line)
{
	ax99100_dwrite_mem_reg(REG_SWRST, SW_RESET, BAR1, line);	
}

static void axspi_line_name(int index, char *p)
{
	sprintf(p, "%s%d", NODE_NAME, (index & 0xFF));
}

static int register_char_device (struct spi_99100 *spi_device)
{	 
	dev_t		dev = MKDEV(spi_major, spi_min_count);
	struct cdev*	spi = &spi_device->spi;	
	int		alloc_ret = 0,cdev_ret = 0;
	struct device 	*device = NULL;	

	memset(spi,0,sizeof(struct cdev));

	if (init_cdev == 0) {
		alloc_ret = alloc_chrdev_region(&dev, 0, NUM_DEVICE, DEV_NAME);
		if (alloc_ret) {
			DEBUG("alloc_chrdev_region Failed.\n");		
			goto disable;
		}
		spi_major = MAJOR(dev);
		init_cdev++;
	}	
	
	spi_device->dev_major = MAJOR(dev);
	spi_device->dev_minor = MINOR(dev);
	DEBUG("maj: %d,min: %d\n",spi_device->dev_major,spi_device->dev_minor);	

	axspi_line_name(spi_device->dev_minor, spi_device->dev_name);

	DEBUG("device name: %s\n", spi_device->dev_name);	
	
	device = device_create(&ax_spi_class, NULL, dev, NULL, spi_device->dev_name);
	
	if (IS_ERR(device)) { 
		DEBUG("device_create Failed %ld.\n",PTR_ERR(device));	
		goto disable;
	}	
	
	cdev_init(spi, &bridge_fops);
	spi->owner 	= THIS_MODULE;
	spi->ops	= &bridge_fops;
	cdev_ret = cdev_add(spi, dev, 1);	
	if (cdev_ret) {
		DEBUG("cdev_add Failed.\n");
		goto disable;
	}		
	
	spi_min_count++;
	return 1;
	
disable:
	if (cdev_ret != 0)
		cdev_del(spi);
	if (device != NULL)
		device_destroy(&ax_spi_class, dev);	
	if (alloc_ret != 0)
		unregister_chrdev_region(dev, 1);
	return -1;
};
void assign_driver(struct pci_dev*);//PCI drivers probe function
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static int spi99100_probe(struct pci_dev *dev, const struct pci_device_id *ent)
#else
static int __devinit spi99100_probe(struct pci_dev *dev, const struct pci_device_id *ent)
#endif
{
	struct spi_99100*	axspi = NULL;
	int 			retval, ret;	
	
	printk(version_spi);

	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);

	axspi = kmalloc(sizeof(struct spi_99100), GFP_KERNEL);
	if (axspi == NULL) {
		dev_err(&dev->dev, "Allocate AX99100 spi device FAILED\n");
		return -1;
	}

	pci_set_drvdata(dev, axspi);

	memset(axspi,0,sizeof(struct spi_99100));
	
	retval = pci_enable_device(dev);		
	
	if (retval) {
		dev_err(&dev->dev, "Device enable FAILED\n");
                return retval;
	}	

	/* To verify whether it is a local bus communication hardware */
	if ((dev->class >> 16) != PCI_CLASS_OTHERS){
		DEBUG("Not a spi communication hardware\n");
		retval = -ENODEV;
		goto disable;
	}
	
	/* Initial Netlink sock */
	if (nl_sk == NULL) { 	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)	
		nl_sk = netlink_kernel_create(&init_net,NETLINK_TEST, &netlink_kerncfg);
#else
		nl_sk = netlink_kernel_create(&init_net,NETLINK_TEST, 0, netlink_get, NULL, THIS_MODULE);	
#endif	
	}

	DEBUG("In %s nl_sk: 0x%x\n",__FUNCTION__ , nl_sk);
	
	ret = register_char_device(axspi);

	axspi_device[axspi->dev_minor] = axspi;

	if (ret < 0) {
	/* Register this block driver with the kernel */
		  DEBUG("In %s char_device_register FAILED\n",__FUNCTION__);
		  return -1;
	}
	
	pci_set_master(dev);	
	
	init_local_data(dev);	

	spi_reset(axspi->dev_minor);

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


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	if ((retval = request_irq(dev->irq, spi99100_interrupt,
				  SA_SHIRQ,"ax99100_spi", axspi))) 
		goto disable;
#else
	if ((retval = request_irq(dev->irq, spi99100_interrupt,
				IRQF_SHARED,"ax99100_spi", axspi))) 
		goto disable;
#endif

	axspi->irq = dev->irq;

	printk("%s at I/O 0x%x (irq = %d) is a AX99100 SPI\n", 
			axspi->dev_name, 
			(unsigned int)axspi->iobase0, 
			axspi->irq);

	DEBUG("In %s---0-----------------------------------END\n",__FUNCTION__);
	return 0;	
	 
disable:
	pci_disable_device(dev);
	DEBUG("In %s---1-----------------------------------END\n",__FUNCTION__);
	return retval;
}


static int spi99100_suspend(struct pci_dev *dev, pm_message_t state)
{
	u16 data;

	spi_suspend_count++;

	/* Enable PME and D3 */
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

static int spi99100_resume(struct pci_dev *dev)
{
	u16 data;
	
	pci_set_power_state(dev, PCI_D0);
	pci_restore_state(dev);
	pci_enable_wake(dev, PCI_D0, 0);

	if (pci_enable_device(dev) < 0) {
		printk(KERN_ERR"pci_enable_device failed, ""disabling device\n");
		return -EIO;
	}
	pci_set_master(dev);

	spi_suspend_count--;

	/* Disable PME */
	if (dev->pm_cap) {
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
		pci_write_config_word(dev, dev->pm_cap + PCI_PM_CTRL, data & (~PCI_PM_CTRL_PME_ENABLE));
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
	}

	return 0;
};

static struct pci_device_id spi99100_pci_tbl[] = {
	{0x125B, 0x9100, PCI_SUBDEV_ID_AX99100, PCI_SUBVEN_ID_AX99100_SPI, 0, 0, 0},

	{0, },
};
static struct pci_driver starex_spi_driver = {
 	.name		= "AX99100_SPI",
	.probe		= spi99100_probe,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	.remove		= spi99100_remove_one,
#else
	.remove		= __devexit_p(spi99100_remove_one),
#endif
	.id_table	= spi99100_pci_tbl,
	.suspend	= spi99100_suspend,
	.resume		= spi99100_resume,
};
/* Drivers entry function. register with the pci core */
int spi99100_init(void)
{	
	int ret;
	
	
	
	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);	

	ret = class_register(&ax_spi_class);
	if (ret) {
		DEBUG("unable to register ax spi class\n");
		return ret;
	}		
	
	ret = pci_register_driver(&starex_spi_driver);
	if (ret < 0){
		DEBUG("In %s pci_register_driver FAILED\n",__FUNCTION__);
		goto err;
	}
	
	
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
	
	return ret;
err:
	class_unregister(&ax_spi_class);
	return ret;	
}

/* Drivers exit function. Unregister with the PCI core as well as serial core */
void spi99100_exit(void)
{	
	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	
	pci_unregister_driver(&starex_spi_driver);
	class_unregister(&ax_spi_class);
	
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);	
}
