/* INCLUDE FILE DECLARATIONS */
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <linux/sockios.h>
#include <pthread.h>
#include "ioctl.h"

/* LOCAL VARIABLES DECLARATIONS */

int main(int argc, char* argv[])
{

	char dev[16];
	int  select;
	int  custom_baud;
	int  product;
	int  parameter;
	int  sampling_clock;

	int devfd;

	while ( 1 ) {

		printf( "Please input the port of AX99100. (ex. /dev/ttyF0):" );
		scanf( "%15s", dev );

		if ( memcmp(dev, "/dev/ttyF", 4) != 0 )
			printf( "Wrong input!!\n" );
		else
		{
			printf( "Correct input!!\n" );
			break;
		}
	}

	//memcpy(dev, "/dev/ttyF0", 10);

	devfd = open(dev, O_RDWR);
	
	if ( devfd == -1 ) {
		printf( "Can't open %s\n", dev );
		return 0;
	}

	if ( ioctl( devfd, IOCTL_GET_PRODUCT, &product ) < 0 ) {
		printf( "IOCTL_GET_PRODUCT failed!!!\n" );
		return 0;	
	}

	if ( (product == 1) || (product == 0) ) {
		printf("\nPort %s is AX99100 serial port.\n", dev);
	} else {
		printf("\nPort %s dose not support.\n", dev);
		return 0;
	}

	if ( ioctl(devfd, IOCTL_GET_CUSTOM, &custom_baud ) < 0) {
		printf("IOCTL_GET_CUSTOM failed!!!\n");
		return 0;	
	}

	if ( custom_baud == 0 )
		printf("\nPort %s support standard baud rate\n", dev);
	else
		printf("\nPort %s support custom baud rate:%d\n", dev, custom_baud);

	if ( product == 1 || product == 0 ) { // AX99100
		while (1) {
		  
			printf("\nPlease specify the baud rate of %s.\n", dev);
			printf("0  : Standard baud rate\n");
			printf("1  : 1.1520 M\n");
			printf("2  : 1.8382 M\n");
			printf("3  : 3.6864 M\n");
			printf("4  : 5.0000 M\n");
			printf("5  : 7.8125 M\n");
			printf("6  : 10.417 M\n");
			printf("7  : 12.500 M\n");
			printf("8  : 15.625 M\n");
			printf("9  : 25.000 M\n");
			printf("10 : 25.000 M\n");
			printf("99 : Exit\n");
			printf(":");
			scanf("%d", &select);

			//bit[27:20] = baud base clock
			//bit[19:12] = DLM
			//bit[11:4] = DLL
			//bit[3:0] = sampling rate
			parameter = 0;

			if (select == 1) {
				custom_baud = 1152000;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 4 << 4;	//bit[11:4] = DLL
				sampling_clock = 27; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 2) {	
				custom_baud = 1843200;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 1 << 4;	//bit[11:4] = DLL
				sampling_clock = 68; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 3) {
				custom_baud = 3686400;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 1 << 4;	//bit[11:4] = DLL
				sampling_clock = 34; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 4) {	
				custom_baud = 5000000;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 5 << 4;	//bit[11:4] = DLL
				sampling_clock = 5 ; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 5) {	
				custom_baud = 7812500;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 1 << 4;	//bit[11:4] = DLL
				sampling_clock = 16 ; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 6) {	
				custom_baud = 10416666;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 1 << 4;	//bit[11:4] = DLL
				sampling_clock = 12; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 7) {	
				custom_baud = 12500000;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 10 << 4;	//bit[11:4] = DLL
				sampling_clock = 10; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 8) {	
				custom_baud = 15625000;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 1 << 4;	//bit[11:4] = DLL
				sampling_clock = 8; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 9) {	
				custom_baud = 25000000;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 1 << 4;	//bit[11:4] = DLL
				sampling_clock = 5; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 10) {	
				custom_baud = 25000000;
				parameter |= CLK_125M << 20;	//bit[27:20] = baud base clock
				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 1 << 4;	//bit[11:4] = DLL
				sampling_clock = 5; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 0) {	
				custom_baud = 0;
				parameter |= CLK_1_8382M << 20;	//bit[27:20] = baud base clock
// 				parameter |= 0 << 12;	//bit[19:12] = DLM
				parameter |= 1 << 4;	//bit[11:4] = DLL
				sampling_clock = 0; 	//bit[3:0] = sampling rate
				break;
			} else if (select == 99) {
				return 0;
			}
		}
	}

	if (ioctl(devfd, IOCTL_SET_CUSTOM, custom_baud) < 0) {
		printf("IOCTL_SET_CUSTOM failed!!!\n");
		return 0;	
	}

	if (ioctl(devfd, IOCTL_SET_PARAMETER, parameter) < 0) {
		printf("IOCTL_SET_PARAMETER failed!!!\n");
		return 0;	
	}
	
	if (ioctl(devfd, IOCTL_SET_SAMPLING, sampling_clock) < 0) {
		printf("IOCTL_SET_SAMPLING failed!!!\n");
		return 0;	
	}

	printf("The %s will operating in custom baud rate after open(re-open) it.\n", dev);

exit:

	printf("\n");

	return 0;
}
