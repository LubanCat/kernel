/* INCLUDE FILE DECLARATIONS */
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <time.h> 
#include <sys/time.h>
#include <sys/socket.h>
#include <linux/netlink.h>
#include "ioctl.h"
#include "ax99100_spi.h"
#include "spi_test.h"

/* STATIC VARIABLE DECLARATIONS */
#define AX99100_TEST_VERSION		"AX99100 Linux SPI Test Tool v1.3.0"

#define ASCII_0		0x30
#define ASCII_7		0x37
#define ENABLE		1
#define DISABLE		0
#define TX		0
#define RX		1
#define INTERRUPT 	0
#define POLLING 	1

ushort	device_no_ede[3] = {
	6,	//0x110
	5,	//0x101
	3	//0x011
};

char* spicmr_name[8] = {
	"SSP",
	"CPHA",
	"CPOL",
	"LSB",
	"SPIMEN",
	"ASS",
	"SWE",
	"SSOE"  
};
char* spicks_name[4] = {
	"125MHz",
	"100MHz",
	"EXT_CLK",
	"EXT_CLK"
};
char* enable_string[2] = {
	"Disable",
	"Enable"
};

char* interrupt_string[2] = {
	"Interrupt",
	"Polling"
};

char* direction_string[3] = {
	"Tx",
	"Rx",
	"TX & RX"
};

struct sockaddr_nl src_addr, dest_addr;
struct nlmsghdr *nlh = NULL;
struct msghdr msg;
struct iovec iov;
int sock_fd;
char num;

int test_area_size = 0x200;

typedef struct _spi_setting {	
	/* initial register */
	uchar 		spicmr;
	uchar		spi_cks;
	uchar		spi_diven;
	uchar 		spibrr;
	/* test performace settong*/
	ushort		spi_device;
	uchar		spi_ede;
	
	uchar		spi_pattern_type;
	uint		spi_test_count;
	uint		spi_data_length;
	uchar		spi_interrupt; 
	uchar		spi_direction;	
	
	
	SPI_DMA		dma_test[2];
	
	struct timeval	starttime;
	struct timeval	endtime;
	double		potency[2];		//Throughpu
	double 		timeuse[2];		//Test Time
	
	uint		test_counter;
} SPI_SETTING, *PSPI_SETTING;

void set_spicmr		(PSPI_SETTING);
void set_spicks		(PSPI_SETTING);
void set_spidiven	(PSPI_SETTING);
void set_spidivider	(PSPI_SETTING);
void set_select_device	(PSPI_SETTING);
void set_pattern_type	(PSPI_SETTING);
void set_test_count	(PSPI_SETTING);
void set_direction	(PSPI_SETTING);
void set_data_length	(PSPI_SETTING);
void set_interrupt	(PSPI_SETTING);

void (*setting_item[9])(PSPI_SETTING) = {
	set_spicmr,
	set_spicks,
	set_spidiven,
	set_select_device,
	set_pattern_type,
	set_test_count,
	set_direction,
	set_data_length,
	set_interrupt
};

void list_spi_setting	(PSPI_SETTING);
void intitial_setting	(PSPI_SETTING);
void throughput		(PSPI_SETTING);

void run_test		(int, PSPI_SETTING);

uint menu();

/*************************************************
 * 
 * Local Bus module API 
 * 
 * **********************************************/
/* Register read/write */
/* MEM mapping - READ*/
BOOL ReadSPIMMCfgReg	(int handle, uint offset, ulong* RegValue)
{
	MMAP_SPI_REG reg;
	
	reg.Offset	= offset;
	reg.Value	= 0; 
	reg.Bar		= BAR1;
	
	
	if (ioctl(handle, IOCTL_MEM_READ_REGISTER, &reg) < 0) {
		printf("IOCTL_MEM_READ_REGISTER failed!!!\n");
		return FALSE;	
	}
	
	*RegValue	= reg.Value;
	return TRUE;
}
/* MEM mapping - WRITE*/
BOOL WriteSPIMMCfgReg	(int handle, uint offset, ulong RegValue)
{
	MMAP_SPI_REG reg;	
	
	reg.Offset	= offset;
	reg.Value	= RegValue;
	reg.Bar		= BAR1;
	
	if (ioctl(handle, IOCTL_MEM_SET_REGISTER, &reg) < 0) {
		printf("IOCTL_MEM_SET_REGISTER failed!!!\n");
		return FALSE;	
	}	
	return TRUE;	
  
}
/* IO mapping - READ*/
BOOL ReadSPIIOCfgReg	(int handle, uint offset, uchar* RegValue)
{
	SPI_REG reg;
	
	reg.Offset	= offset;
	reg.Value	= 0;
	
	if (ioctl(handle, IOCTL_IO_READ_REGISTER, &reg) < 0) {
		printf("IOCTL_IO_READ_REGISTER failed!!!\n");
		return FALSE;	
	}	
	
	*RegValue	= reg.Value;
	return TRUE;
}
/* IO mapping - WRITE*/
BOOL WriteSPIIOCfgReg	(int handle, uint offset, uchar RegValue)
{  
	SPI_REG reg;
	
	reg.Offset	= offset;
	reg.Value	= RegValue;
	
	if (ioctl(handle, IOCTL_IO_SET_REGISTER, &reg) < 0) {
		printf("IOCTL_IO_SET_REGISTER failed!!!\n");
		return FALSE;	
	}	
	return TRUE;
}
/* Get RX DMA DATA */
BOOL GetSpiRxDmaData (int handle, ulong DataLength, PSPI_DMA RxDmaData)
{
	RxDmaData->Length = DataLength;
	
	if (ioctl(handle, IOCTL_RX_DMA_READ, RxDmaData) < 0) {
		printf(" IOCTL_RX_DMA_READ failed!!!\n");
		return FALSE;	
	}	
	return TRUE;
}
/* Set TX DMA DATA */
BOOL SetSpiTxDmaData (int handle, PSPI_DMA TxDmaData)
{
	if (ioctl(handle, IOCTL_TX_DMA_WRITE, TxDmaData) < 0) {
		printf("IOCTL_TX_DMA_WRITE failed!!!\n");
		return FALSE;	
	}	
	return TRUE;
}
/* Set TX DMA REG */
BOOL SetSpiTxDmaReg(int handle, ulong DataLength)
{
	if (ioctl(handle, IOCTL_SET_TX_DMA_REG, &DataLength) < 0) {
		printf("IOCTL_SET_TX_DMA_REG failed!!!\n");
		return FALSE;	
	}	
	return TRUE;
}
/* Set RX DMA REG */
BOOL SetSpiRxDmaReg(int handle, ulong DataLength)
{	
	if (ioctl(handle, IOCTL_SET_RX_DMA_REG, &DataLength) < 0) {
		printf("IOCTL_SET_RX_DMA_REG failed!!!\n");
		return FALSE;	
	}	
	return TRUE;
}

/*************************************************
 * 
 * MAIN PROCESS
 * 
 * **********************************************/

int main(int argc, char* argv[])
{
	SPI_SETTING	item;	//test struct
	
	int	devfd;		//Device
	int	menu_item = 0;
	int	temp = 0;
	char	again[10];

	if (argc > 2 || argc <= 1) {
		printf("%s\n", AX99100_TEST_VERSION);
		printf("----Error parameter!\n");
		printf("\tCommand: ./spi_test [dev] \n");
		printf("\tExample: ./spi_test /dev/spi00\n");
		return -1;
	}
	
	
	/* Open Device */
	devfd = open(argv[1], O_RDONLY);
	if (devfd == -1) {
		printf("Can't open AX99100 SPI device\n");
		return -1;
	} else {
		num = argv[1][8];
	}	

	intitial_setting(&item);
	
	do {
		list_spi_setting(&item);
		menu_item = menu();	
		switch (menu_item) {
		case 0 ... 8:
		{
			setting_item[menu_item](&item);
			break;
		}
		case 9:
		{
			memset(&item.dma_test[TX], '\0', 128*1024);
			memset(&item.dma_test[RX], '\0', 128*1024);
			run_test(devfd, &item);
			do {
				
				printf("\nTest again?(Y/N):");
				scanf("%9s",again);
				if ((again[0] - 0x59) == 0 || (again[0] - 0x79) == 0) {//Y  
					break;
				} else if ((again[0] - 0x4E) == 0 || (again[0] - 0x6E) == 0) { //N
					close(sock_fd);
					close(devfd);
					return 0;
				}				      
			} while(1);
			break;
		}
		case 99:
		{
			close(sock_fd);
			close(devfd);
			return 0;
		}
		}
	  
	} while (1);
	
	close(sock_fd);
	close(devfd);	
	return 0;
}

void set_spicmr	(PSPI_SETTING item)
{
	int 	i;
	char	value[10];
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("SPICMR Setting\n\n");
	printf("Number: ");
	for (i = 0; i < 8; i++)
		 printf("%d\t",i);
	printf("\nName:   ");
	for (i = 0; i < 8; i++)
		 printf("%s\t",spicmr_name[i]);
	printf("\n\nExample: Need to set SPIMEN ASS SSOE bits, and the value is 457\n");
	/* Input the value of SPICMR */
first:	
	item->spicmr = 0;
	printf("Input the value of SPICMR: ");
	scanf("%9s",value);
	for (i = 0; i < strlen(value); i++) {
		if (value[i] < ASCII_0 || value[i] > ASCII_7) {
			printf("Input the error value!\n");
			goto first;
		} else {
			item->spicmr |= (1 << (value[i] - ASCII_0));
		}
	}
	
}

void set_spicks (PSPI_SETTING item)
{
	int     value = 0;
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("SPICSS SPICKS Setting\n\n");
	printf("Valus:\t0:125MHz\n\t1:100MHz\n\t2,3:EXT_CLK\n");
	
	/* Input the value of SPICKS */
first:	
	item->spi_cks = 0;
	printf("Input the value of SPICKS: ");
	scanf("%d",&value);
	if (value < 0 || value > 3) {
		printf("Input the error value!\n");
		goto first;
	} else {
		item->spi_cks = value;
	}	
}

void set_spidiven (PSPI_SETTING item)
{
	int     value = 0;
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("SPICSS SPIDIVEN Setting\n\n");
	printf("Valus:\t0:Disable\n\t1:Enable\n");
	
	/* Input the value of SPIDIVEN */
first:	
	item->spi_diven = 0;
	printf("Input the value of SPIDIVEN: ");
	scanf("%d",&value);
	if (value < 0 || value > 1) {
		printf("Input the error value!\n");
		goto first;
	} else {
		item->spi_diven = (value << 0);
		if (item->spi_diven == ENABLE)
			set_spidivider(item);
		else
			item->spibrr = 0;
	}	
}

void set_spidivider (PSPI_SETTING item)
{
	int     value = 0;
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("SPIDIVIDER Setting\n\n");
	printf("Valus:\t0 - 255\n");
	
	/* Input the value of SPIDIVEN */
first:	
	item->spibrr = 0;
	printf("Input the value of SPIDIVIDER: ");
	scanf("%d",&value);
	if (value < 0 || value > 255) {
		printf("Input the error value!\n");
		goto first;
	} else {
		item->spibrr = value;
	}
	printf("0x%x\n",item->spibrr);
}

void set_select_device (PSPI_SETTING item)
{
	int     value = 0;
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("Externel Decoder Setting\n\n");
	printf("Valus:\t0:Disable\n\t1:Enable\n");
	
	/* Input the value of EDE */
first:	
	item->spi_ede = 0;
	printf("Input the value of EDE: ");
	scanf("%d",&value);
	if (value < 0 || value > 1) {
		printf("Input the error value!\n");
		goto first;
	} else {
		item->spi_ede = value;
	}
	printf("0x%x\n",item->spibrr);	
	if (item->spi_ede == ENABLE) {
		system("clear");
		/* Information */
		printf("Divice Setting\n\n");
		printf("Valus:\t0 - 6 (Device 0 - Device 6)\n");
ede:
		/* Input the value of device */
		item->spi_device = 0;
		printf("Input the value of device: ");
		scanf("%d",&value);
		if (value < 0 || value > 6) {
			printf("Input the error value!\n");
			goto ede;
		} else {
			item->spi_device = value;
		}
		printf("0x%x\n",item->spi_device);	  
	} else {
		system("clear");
		/* Information */
		printf("Divice Setting\n\n");
		printf("Valus:\t0 - 2 (Device 0 - Device 2)\n");
no_ede:
		/* Input the value of device */
		item->spi_device = 0;
		printf("Input the value of device: ");
		scanf("%d",&value);
		if (value < 0 || value > 2) {
			printf("Input the error value!\n");
			goto no_ede;
		} else {
			item->spi_device = device_no_ede[value];
		}
		printf("0x%x\n",item->spi_device);
	  
	  
	}
	
}

void set_pattern_type (PSPI_SETTING item)
{
	int     value = 0;
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("Data Pattern Type Setting\n\n");
	printf("Valus:\t0 - 255 (0x00 - 0xff)\n");
	
	/* Input the value of pattern */
first:	
	item->spi_pattern_type = 0;
	printf("Input the value of pattern: ");
	scanf("%d",&value);
	if (value < 0 || value > 255) {
		printf("Input the error value!\n");
		goto first;
	} else {
		item->spi_pattern_type = value;
	}	
}

void set_test_count (PSPI_SETTING item)
{
	int     value = 0;
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("Data Test Count Setting\n\n");	
	
	/* Input the value of test count */
first:
	item->spi_test_count = 0;
	printf("Input the value of test count: ");
	scanf("%d",&value);
	if (value <= 0) {
		printf("Input the error value!\n");
		goto first;
	} else 
		item->spi_test_count = value;	
}

void set_direction (PSPI_SETTING item)
{
	int     value = 0;
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("Data Tx/Rx Direction Setting\n\n");
	printf("Valus:\t0:TX\n\t1:RX\n\t2:TX & RX\n");	
	
	/* Input the value of test count */
first:
	item->spi_direction = 0;
	printf("Input the value of direction: ");
	scanf("%d",&value);
	if (value < 0 || value > 2) {
		printf("Input the error value!\n");
		goto first;
	} else 
		item->spi_direction = value;
	
	if (item->spi_direction == TX || item->spi_direction == TXRX) 
		set_pattern_type(item);		
	
}

void set_data_length (PSPI_SETTING item)
{
	int     value = 0;
	
	if (item->spi_direction == TX || item->spi_direction == TXRX) {
		item->spi_data_length = 256;		
		return;
	}
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("Data Length Setting\n\n");
	printf("Valus:\t1 - 65535\n");	
	
	/* Input the value of test count */
first:	
	item->spi_data_length = 0;
	printf("Input the value of data length: ");
	scanf("%d",&value);
	
	if (value < 1 || value > 65535) {
		printf("Input the error value!\n");
		goto first;
	} else
		item->spi_data_length = value;
	
}

void set_interrupt (PSPI_SETTING item)
{
	int     value = 0;
	
	system("clear");
	printf("%s\n", AX99100_TEST_VERSION);
	/* Information */
	printf("Data Interrupt Setting\n\n");
	printf("Method:\t0:Interrupt\n\t1:Polling\n");	
	
	/* Input the value of test count */
first:
	item->spi_interrupt = 0;
	printf("Select the mode: ");
	scanf("%d",&value);
	if (value < 0 || value > 1) {
		printf("Input the error value!\n");
		goto first;
	} else 
		item->spi_interrupt = value;	

}

void list_spi_setting (PSPI_SETTING item)
{
	int	i;
	
	system("clear");
	printf("*************************************************\n");
	printf("*               Test SPI Setting                *\n");
	printf("*************************************************\n");

	printf("%s\n", AX99100_TEST_VERSION);
	
	printf("**Initial Information:\n");
	/* SPICMR */
	printf("SPICMR: 0x%x (",item->spicmr);
	for (i = 0; i < 8; i++) 
		if( ((item->spicmr >> i) & 0x01 ) )
			printf(" %s",spicmr_name[i]);
	printf(" )\n");
	
	/* SPICSS */
	printf("SPICKS: 0x%02x ( %s )\t\t",item->spi_cks,spicks_name[item->spi_cks]);
	/* SPIDIVEN */
	printf("SPIDIVEN: 0x%x ( %s )\t",item->spi_diven,enable_string[item->spi_diven]);
	/* SPIDIVIDER */
	if (item->spi_diven != DISABLE)
		printf("SPIDIVIDER: %d\n",item->spibrr);	
	
	printf("\n**Test Setting:\n");
	printf("Test Direction: %s\n",direction_string[item->spi_direction]);
	/* Device number */
	printf("Device Num.:  0x%x\t",item->spi_device);
	/* Test Count */
	printf("Test Count: % 6d\t",item->spi_test_count);	
	/* EDE */
	printf("Externel Decoder: 0x%x ( %s )\n",item->spi_ede,enable_string[item->spi_ede]);
	
	/* Pattern */
	if (item->spi_direction == TX || item->spi_direction == TXRX)	
		printf("Pattern Type: 0x%02x\t",item->spi_pattern_type);
	else
		printf("Pattern Type: None\t");
	/* Length */
	/* Length */
	printf("Data Length: % 5d\t",item->spi_data_length);
	/* Interrupt */
	printf("Interrupt Mode:   %s\n",interrupt_string[item->spi_interrupt]);
}
void intitial_setting (PSPI_SETTING item)
{
	
	/* Initial register */
	set_spicmr(item);
	set_spicks(item);
	set_spidiven(item);	
	
	set_select_device(item);	
	set_test_count(item);
	set_direction(item);
	set_data_length(item);
	set_interrupt(item);
	
	list_spi_setting(item);
	
	/* Initial Netlink sock */
	sock_fd = socket(PF_NETLINK, SOCK_RAW,NETLINK_TEST);

	memset(&src_addr, 0, sizeof(src_addr));
	src_addr.nl_family = AF_NETLINK;
	src_addr.nl_pid = getpid();  /* self pid */
	src_addr.nl_groups = 0;  /* not in mcast groups */
	bind(sock_fd, (struct sockaddr*)&src_addr,
	     sizeof(src_addr));
	memset(&dest_addr, 0, sizeof(dest_addr));
	dest_addr.nl_family = AF_NETLINK;
	dest_addr.nl_pid = 0;   /* For Linux Kernel */
	dest_addr.nl_groups = 0; /* unicast */
	nlh=(struct nlmsghdr *)malloc(
				 NLMSG_SPACE(MAX_PAYLOAD_SIZE));
	/* Fill the netlink message header */
	nlh->nlmsg_len = NLMSG_SPACE(MAX_PAYLOAD_SIZE);
	nlh->nlmsg_pid = getpid();  /* self pid */
	nlh->nlmsg_flags = 0;
	/* Fill in the netlink message payload */
	strcpy(NLMSG_DATA(nlh), (char *)&num);
	
	iov.iov_base = (void *)nlh;
	iov.iov_len = nlh->nlmsg_len;
	msg.msg_name = (void *)&dest_addr;
	msg.msg_namelen = sizeof(dest_addr);
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	
	sendmsg(sock_fd, &msg, 0);
}
unsigned int menu()
{
	int temp = 0;
	
	printf("\n***Initial data***\n");
	printf(" 0: Modify SPICMR\n");
	printf(" 1: Modify SPICKS\n");
	printf(" 2: Modify SPIDIVIDER\n");
	printf("***Test setting***\n");
	printf(" 3: Select Device\n");
	printf(" 4: Modify Data Pattern(for tx)\n");
	printf(" 5: Modify Test Count\n");
	printf(" 6: Select Tx or Rx\n");
	printf(" 7: Modify Data Length\n");
	printf(" 8: Select Interrupt Mode\n\n");
	printf(" 9: Test performace.\n");
	printf("99: Exit the test tool.\n");
	printf("Please choose: ");
	scanf("%d",&temp);
	return temp;
}

void throughput	(PSPI_SETTING item)
{
  
	switch (item->spi_direction) {
	case TX:
	{	
		item->potency[TX] = 256;
		item->potency[TX] = item->potency[TX] * item->test_counter * test_area_size;
		item->potency[TX] /= item->timeuse[TX];
		item->potency[TX] /= 1000;
		item->potency[RX] = 0;
		break;
	}
	case RX:
	{
		item->potency[RX] = item->spi_data_length;
		item->potency[RX] = item->potency[RX] * item->test_counter * test_area_size;
		item->potency[RX] /= item->timeuse[RX];
		item->potency[RX] /= 1000;
		item->potency[TX] = 0;
		break;
	}
	case TXRX:
	{
		item->potency[TX] = 256;
		item->potency[TX] = item->potency[TX] * item->test_counter * test_area_size;
		item->potency[RX] = item->potency[TX];
		item->potency[TX] /= item->timeuse[TX];
		item->potency[TX] /= 1000;
		item->potency[RX] /= item->timeuse[RX];
		item->potency[RX] /= 1000;
		break;
	}
	}
	printf("*************************************************\n");
	printf("*                 Throughput                    *\n");
	printf("*************************************************\n");
	printf("Test Counter: %d\n", item->test_counter);
	
	if (item->potency[TX] != 0)
		printf("TX Throughput: %f MB/s\n",item->potency[TX]);
	if (item->potency[RX] != 0)
		printf("RX Throughput: %f MB/s\n",item->potency[RX]);
}

/*************************************************
 * 
 * Test Help Function
 * 
 * **********************************************/
typedef struct _op_code_value {
	uchar	opcode[8];
	uchar	opcode_length;  
} OP_CODE, *POP_CODE;

OP_CODE flash_opcode[] = {
  /* WREN */
	{
		.opcode		= {OPCODE_WREN,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		.opcode_length	= 0x00
	},
  /* RDSR */
	{
		.opcode		= {OPCODE_RDSR,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		.opcode_length	= 0x00
	},
  /* SE */
	{
		.opcode		= {OPCODE_SE,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		.opcode_length	= 0x03
	},
  /* BE */
	{
		.opcode		= {OPCODE_BE,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		.opcode_length	= 0x03
	},
  /* CE */
	{
		.opcode		= {OPCODE_CE,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		.opcode_length	= 0x00
	},  
};

int (*check_ISR)(int);

uchar	global_sdcr = 0x00;

void run_test			(int, PSPI_SETTING);
void cal_time			(PSPI_SETTING, uint);
int wait_interrupt_polling	(int);
int wait_interrupt		(int);
void set_op_code 		(int, PSPI_SETTING, POP_CODE);
void generate_flash_data	(int, PSPI_SETTING);
void dma_TRx_compare		(PSPI_SETTING);


void rx_test			(int, PSPI_SETTING, POP_CODE);
void tx_test			(int, PSPI_SETTING, POP_CODE);
BOOL check_flash_status		(int, uint);


/*************************************************
 * RX Test Help Function
 * **********************************************/
void run_test (int handle, PSPI_SETTING item)
{
	OP_CODE		test;
	uchar		sdcr = 0x0;
	int		i, fvalue, FLen;
	int		error;
	
	unsigned char	temp = 0;
	char*		data_pattern = NULL;
	
/* Initial register*/
	/* Set SPICMR */ 
	WriteSPIIOCfgReg(handle, REG_SPICMR, item->spicmr);
	/* Set SPICSS */
	temp = 0;
	temp |= item->spi_cks; 		//SPICKS
	temp |= (item->spi_diven << 2);	//SPIDIVEN
	WriteSPIIOCfgReg(handle, REG_SPICSS, temp);	
	/* Set SPIBRR */
	WriteSPIIOCfgReg(handle, REG_SPIBRR, item->spibrr);
	
	for (i = 0; i < 2; i++) {
		item->dma_test[i].Length = item->spi_data_length;
		item->potency[i] = 0;
		item->timeuse[i] = 0;
	}
	
	if (item->spi_interrupt == INTERRUPT) {
		check_ISR 	= wait_interrupt;
		global_sdcr	= 0xC0;
	} else {
		check_ISR = wait_interrupt_polling;	
		global_sdcr	= 0x00;
	}
	
	test.opcode_length = 0x03;
	
	for (item->test_counter = 1; item->test_counter <= item->spi_test_count; item->test_counter++) {
	  
		for (fvalue = 0; fvalue < test_area_size; fvalue++) {			
		  /* Cal Address */
			FLen = fvalue;  //adrress
			test.opcode[1] = test.opcode[2] = test.opcode[3] = 0x00;
			test.opcode[2] = (uchar) (FLen & 0x00FF);
			test.opcode[1] = (uchar) ((FLen & 0xFF00) >> 8);			
			/* Initial for TX */
			if (item->spi_direction == TX || item->spi_direction == TXRX) { 
				if (test.opcode[1] == 0 &&
				    test.opcode[2] == 0 &&
				    test.opcode[3] == 0) {
				/* Set WREN */
					/* Set OP code */
					set_op_code(handle, item, &flash_opcode[WREN]);
					/* Start */
					WriteSPIIOCfgReg(handle, REG_SDCR, global_sdcr | 0x0C);
					/* Wait Interrupt */
					if ((error = check_ISR(handle)) != SUCCESS) {
						printf("1Interrupt error: %d\n", error);
						exit(1);
					}
					/* Check flash status */
					if (!check_flash_status(handle, FLASH_STATUS_WREN)) {
						printf("Check flash status failed! line894\n");
						exit(1);
					}
 				/* Set Chip Erase */
 					/* Set OP code */
 					set_op_code(handle, item, &flash_opcode[CE]);
 					/* Start */
 					WriteSPIIOCfgReg(handle, REG_SDCR, global_sdcr | 0x0C);
 					/* Wait Interrupt */
 					if ((error = check_ISR(handle)) != SUCCESS) {
 						printf("2Interrupt error: %d\n", error);
 						exit(1);
 					}
 					/* Check flash status */
 					if (!check_flash_status(handle, FLASH_STATUS_ERASE)) {
 						printf("Check flash status failed! line909\n");
 						exit(1);
 					}
				}
				generate_flash_data(handle, item);
			}
			/* Test */
			switch (item->spi_direction) {
			case TX :
 				test.opcode[0] = OPCODE_PP;
 				tx_test(handle, item, &test);
				break;
			case RX :
				test.opcode[0] = OPCODE_READ;
				rx_test(handle, item, &test);
				break;			
			case TXRX :
			  /* TX */
				test.opcode[0] = OPCODE_PP;
				tx_test(handle, item, &test);
			  /* RX */
				test.opcode[0] = OPCODE_READ;
				rx_test(handle, item, &test);
			  /* Compare */
				dma_TRx_compare(item);
				break;
			}
		}
		list_spi_setting(item);
		throughput(item);
	}	
}



void rx_test (int handle, PSPI_SETTING item, POP_CODE op)
{
	int	error;
	
	/* Set RX DMA */
	if (!SetSpiRxDmaReg(handle,item->dma_test[TX].Length)) {
		printf("ERROR IOCTL RX DMA SETTING!\n");
		exit(1);
	}	
	/* Set READ op-code */
	set_op_code(handle, item, op);
	
	/* Start to test */
	gettimeofday(&item->starttime,0);
		WriteSPIIOCfgReg(handle, REG_SDCR, global_sdcr | 0x0E);
		/* Wait Interrupt */
		if ((error = check_ISR(handle)) != SUCCESS) {
			printf("RXInterrupt error: %d\n", error);
			exit(1);
		}
	gettimeofday(&item->endtime,0);
	
	/* Calculate time */
	cal_time(item, RX);

	/* Get RX Data */
	GetSpiRxDmaData(handle, item->spi_data_length, &item->dma_test[RX]);
}

void tx_test (int handle, PSPI_SETTING item, POP_CODE op)
{
	int error;
	
/* TX Write enable for flash */
	/* Set OP code */
	set_op_code(handle, item, &flash_opcode[WREN]);
	/* Start */
	WriteSPIIOCfgReg(handle, REG_SDCR, global_sdcr | 0x0C);
	/* Wait Interrupt */
	if ((error = check_ISR(handle)) != SUCCESS) {
		printf("TX1Interrupt error: %d\n", error);
		exit(1);
	}
	/* Check flash status */
	if (!check_flash_status(handle, FLASH_STATUS_WREN)) {
		printf("Check flash status failed!");
		exit(1);
	}

	/* Set TX DMA */
	if (!SetSpiTxDmaReg(handle,item->dma_test[TX].Length)) {
		printf("ERROR IOCTL TX DMA SETTING!\n");
		exit(1);
	}
	/* Set PP op-code */
	set_op_code(handle, item, op);
	/* Start to test */
	gettimeofday(&item->starttime,0);
		WriteSPIIOCfgReg(handle, REG_SDCR, global_sdcr | 0x0D);
		/* Wait Interrupt */
		if ((error = check_ISR(handle)) != SUCCESS) {
			printf("TX2Interrupt error: %d\n", error);
			exit(1);
		}
		/* Check flash status */
		if (!check_flash_status(handle, FLASH_STATUS_PP)) {
			printf("Check flash status failed!");
			exit(1);
		}
	gettimeofday(&item->endtime,0);
	/* calculate time */
	cal_time(item, TX);		
}

BOOL check_flash_status	(int handle, uint status)
{
	uchar	reg_ssol = 0;
	uchar 	reg_stof1 = 0;
	time_t	start_time, cur_time; 
	int	error;
	
	time(&start_time);
	do { 
		time(&cur_time);
		WriteSPIIOCfgReg(handle, REG_STOF0, OPCODE_RDSR);
		WriteSPIIOCfgReg(handle, REG_STOF1, 0x00);
		ReadSPIIOCfgReg(handle, REG_SPISSOL, &reg_ssol);
		reg_ssol &= 0x0F;
		reg_ssol |= 0x10;
		WriteSPIIOCfgReg(handle, REG_SPISSOL, reg_ssol);
		WriteSPIIOCfgReg(handle, REG_SDCR, global_sdcr | 0x0C);
		/* Wait Interrupt */
		if ((error = check_ISR(handle)) != SUCCESS) {
			printf("0x%x\n", status);
			printf("CInterrupt error: %d\n", error);
			exit(1);
		}		
		ReadSPIIOCfgReg(handle, REG_STOF1, &reg_stof1);
		
		if (reg_stof1 == status)			
			return TRUE;
		//printf("reg_stof1 = 0x%x\n", reg_stof1);
	
	} while ((cur_time - start_time) < 10);	
	
	return FALSE;
	
}

void set_op_code (int handle, PSPI_SETTING item, POP_CODE op)
{
	uchar	temp;
	uint	i;
	uint	op_count =  op->opcode_length + 1;
/* set op-code */
	for (i = 0; i < op_count; i++) 
		WriteSPIIOCfgReg(handle, (REG_STOF0 + i), op->opcode[i]);
/* Select device and set op-code length */
	temp = 0;
	temp |= item->spi_device;		//Slave select
	temp |= (item->spi_ede << 3);	
	temp |= (op->opcode_length << 4);	//OPcode length
	WriteSPIIOCfgReg(handle, REG_SPISSOL, temp);
}

int wait_interrupt_polling (int handle)
{
	time_t	start_time, cur_time; 
	uchar	isr = 0;
	
	time(&start_time);
	do {	
		time(&cur_time);
		ReadSPIIOCfgReg(handle, REG_SPIMISR, &isr);
		if (cur_time - start_time > 20)
		{
			return ERROR_POLLING_TIMEOUT;
		}
	} while (isr == 0);
	if (isr == 0x01)
	{
		WriteSPIIOCfgReg(handle, REG_SPIMISR, isr);
		return SUCCESS;
	}
	else
	{
		WriteSPIIOCfgReg(handle, REG_SPIMISR, isr);
		return ERROR_SPI_TRANSCERIVER;
	}
}

int wait_interrupt (int handle)
{
	memset(nlh, 0, NLMSG_SPACE(MAX_PAYLOAD_SIZE));
	recvmsg(sock_fd, &msg, 0);
	//printf(" Received message payload: %s\n",
	//	NLMSG_DATA(nlh));
	return 0;
}

void cal_time (PSPI_SETTING item, uint direction)
{
	double	temp = 0;
	/* calculate time ms */
	temp = 1000000 * (item->endtime.tv_sec - item->starttime.tv_sec)			
		   + item->endtime.tv_usec - item->starttime.tv_usec;
	temp /=1000;	
	
	item->timeuse[direction] += temp;
}

void generate_flash_data (int handle, PSPI_SETTING item)
{
	int 	i;
	uchar	TempData = 0;
	
	if (item->spi_pattern_type == 0xFF) 
		for(i = 0 ; i < item->spi_data_length ; i++)
		{
			item->dma_test[TX].Buffer[i] = TempData;

			if (TempData == 0xFF)
				TempData = 0;
			else
				TempData++;
		}
	else
		memset(&item->dma_test[TX].Buffer,
			item->spi_pattern_type,
			item->spi_data_length  
		);
	
	SetSpiTxDmaData(handle, &item->dma_test[TX]);
}

void dma_TRx_compare (PSPI_SETTING item)
{
	if (memcmp( &item->dma_test[TX].Buffer,
		    &item->dma_test[RX].Buffer,
		    0x100) != 0)//check
	{
		int i;
		printf("TX & RX data comparison failed!\n");
		printf("TX Buffer -> \n");
		for (i = 0; i < 0x100; i++) {
			printf("%02x ", item->dma_test[TX].Buffer[i]);
			if ((i % 16) == 15)
				printf("\n");
		}
		printf("RX Buffer -> \n");
		for (i = 0; i < 0x100; i++) {
			printf("%02x ", item->dma_test[RX].Buffer[i]);
			if ((i % 16) == 15)
				printf("\n");
		}
		exit(1);
	}

	memset(&item->dma_test[TX].Buffer, 0xFF, 0x100);
	memset(&item->dma_test[RX].Buffer, 0xFF, 0x100);	
}
