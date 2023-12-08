/* Flash OP code */
#define OPCODE_WREN		0x06
#define OPCODE_RDSR		0x05
	#define FLASH_STATUS_WREN	0x02
	#define FLASH_STATUS_ERASE	0x00
	#define FLASH_STATUS_PP		0x00
#define OPCODE_READ		0x03
#define OPCODE_SE		0x20
#define OPCODE_BE		0x52
#define OPCODE_CE		0x60
#define OPCODE_PP		0x02

#define WREN		0
#define RDSR		1
#define SE		2
#define BE		3
#define CE		4

/* Shift op-code */
#define OPCODE_STOF0		0
#define OPCODE_STOF1		8
#define OPCODE_STOF2		16
#define OPCODE_STOF3		24
#define OPCODE_STOF4		OPCODE_STOF0
#define OPCODE_STOF5		OPCODE_STOF1
#define OPCODE_STOF6		OPCODE_STOF2
#define OPCODE_STOF7		OPCODE_STOF3



#define TRUE	1
#define FALSE	0

#define TX	0
#define RX	1
#define TXRX	2

#define INTERRUPT	0
#define POLLING		1


#define	SUCCESS			0
#define ERROR_POLLING_TIMEOUT	-1
#define ERROR_SPI_TRANSCERIVER	-2

typedef unsigned char uchar;
typedef uchar BOOL;

