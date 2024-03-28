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
#include <linux/types.h>
#include <pthread.h>
#include "ioctl.h"

/* LOCAL VARIABLES DECLARATIONS */

char str_to_char(char* str)
{
	int i;
	int value = 0;
	int count = 128;

	for (i = 0; i <= 14; i++) {
		if (memcmp(&str[i], "1", 1) == 0) {
			value += count;
			count = count >> 1;
		} else if (memcmp(&str[i], "0", 1) == 0) {
			count = count >> 1;
		}	
	}

	return value;
}

int main(int argc, char* argv[])
{

	char dev[16];
	int  select;
	char input_str[8];
	int  return_value;
	int  setup_value;

	char gpio7_value;
	char gpio6_value;
	char gpio5_value;
	char gpio4_value;
	char gpio3_value;
	char gpio2_value;
	char gpio1_value;
	char gpio0_value;

	int devfd;

	while (1) {

		printf("Please input the port of AX99100. (ex. /dev/ttyF0):");
		scanf("%15s", dev);

		if (memcmp(dev, "/dev/ttyF", 9) != 0)
			printf("Wrong input!!\n");
		else
			break;
	}

	devfd = open(dev, O_RDWR);
	
	if (devfd == -1) {
		printf("Can't open %s\n", dev);
		return 0;
	}

	while (1) {

		memset(&input_str, 'X', sizeof(input_str));
		return_value = 0;
		setup_value = 0;

		printf("\nPlease specify the operation of %s.\n", dev);
		printf("0  : Show GPIO status\n");
		printf("1  : Setup GPIO direction (0:Output, 1:Input)\n");
		printf("2  : Setup GPIO output value\n");
		printf("99 : Exit\n");
		printf(":");
		scanf("%d", &select);

		if (select == 99) {
			return 0;
		}

		if (select == 0) {

			if (ioctl(devfd, IOCTL_GPIO_STATUS, &return_value) < 0) {
				printf("IOCTL_GPIO_STATUS failed!!!\n");
				return 0;	
			}

			//return_value BIT[15-8] is direction, BIT[7-0] is value
			gpio7_value = (return_value >> 7) & 0x01;
			gpio6_value = (return_value >> 6) & 0x01;
			gpio5_value = (return_value >> 5) & 0x01;
			gpio4_value = (return_value >> 4) & 0x01;
			gpio3_value = (return_value >> 3) & 0x01;
			gpio2_value = (return_value >> 2) & 0x01;
			gpio1_value = (return_value >> 1) & 0x01;
			gpio0_value = (return_value >> 0) & 0x01;

			printf("\n");			

			if ((return_value >> 15) & 0x01)
				printf("GPIO7: %s	: %d\n", "Input", gpio7_value);
			else
				printf("GPIO7: %s	: %d\n", "Output", gpio7_value);

			if ((return_value >> 14) & 0x01)
				printf("GPIO6: %s	: %d\n", "Input", gpio6_value);
			else
				printf("GPIO6: %s	: %d\n", "Output", gpio6_value);
				
			if ((return_value >> 13) & 0x01)
				printf("GPIO5: %s	: %d\n", "Input", gpio5_value);
			else
				printf("GPIO5: %s	: %d\n", "Output", gpio5_value);

			if ((return_value >> 12) & 0x01)
				printf("GPIO4: %s	: %d\n", "Input", gpio4_value);
			else
				printf("GPIO4: %s	: %d\n", "Output", gpio4_value);

			if ((return_value >> 11) & 0x01) 
				printf("GPIO3: %s	: %d\n", "Input", gpio3_value);
			else
				printf("GPIO3: %s	: %d\n", "Output", gpio3_value);

			if ((return_value >> 10) & 0x01)
				printf("GPIO2: %s	: %d\n", "Input", gpio2_value);
			else
				printf("GPIO2: %s	: %d\n", "Output", gpio2_value);

			if ((return_value >> 9) & 0x01)
				printf("GPIO1: %s	: %d\n", "Input", gpio1_value);
			else
				printf("GPIO1: %s	: %d\n", "Output", gpio1_value);

			if ((return_value >> 8) & 0x01)
				printf("GPIO0: %s	: %d\n", "Input", gpio0_value);
			else
				printf("GPIO0: %s	: %d\n", "Output", gpio0_value);
		}

		if (select == 1) {
			printf("Please input GPIO7-GPIO0 direction you want to setup (ex. 0/0/1/1/0/1/0/1 or 00110101): ");
			scanf("%7s", input_str);

			setup_value = str_to_char(input_str);

			if (ioctl(devfd, IOCTL_GPIO_DIR, setup_value) < 0) {
				printf("IOCTL_GPIO_DIR failed!!!\n");
				return 0;	
			}
		}

		if (select == 2) {
			printf("Please input GPIO5-GPIO0 value you want to output (ex. 0/0/1/1/0/1/0/1 or 00110101): ");
			scanf("%7s", input_str);

			setup_value = str_to_char(input_str);

			if (ioctl(devfd, IOCTL_GPIO_OUTPUT, setup_value) < 0) {
				printf("IOCTL_GPIO_OUTPUT failed!!!\n");
				return 0;	
			}
		}
	}

exit:

	if (close(devfd) != 0)
		printf("Can't close %s\n", dev);

	printf("\n");

	return 0;
}
