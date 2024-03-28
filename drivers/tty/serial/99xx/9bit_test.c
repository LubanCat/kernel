#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include "ioctl.h"

#define TOOL_VERSION "AX99100 Serial Port 9-bit Mode Test Tool: v1.0.0"

int devfd;

static void usage()
{
	system("clear");
	printf("%s\n\n", TOOL_VERSION);
	printf("Usage:9bit_test [dev] [option]\n");
	printf("Example: (9-bit mode: Master, ID: 20)\n");
	printf("\t ./9bit_test /dev/ttyF0 -m 1 -i 20\n");
	printf("-m [Mode]\t1:Master 2:Slave SW 3:Slave HW 4:Data mode\n");
	printf("-i [Address ID]\trange 0-255\n\n");
	printf("Note: Forced 115200, 8N1\n");
}

int
set_interface_attribs (int fd, int speed, int parity)
{
       struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, (speed_t)speed);
        cfsetispeed (&tty, (speed_t)speed);


	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}
void *
rx_function(void * ptr)
{
	int n;
	char buf = '\0';
	while (1) {		
		n = 0;		
   		n = read(devfd, &buf, 1);
		if (n > 0)
			printf("RX: %c (0x%02x)\n", buf, (buf & 0xFF));			
		usleep(100 * 1000);	
	};
}

int
main(int argc, char* argv[])
{
	int s, rx;
	char dev_name[100];
	char tx = '\0';
	char address = '\0';
	unsigned char start_val = 0;	
	struct _slave_mode_config config;	
	pthread_t thread_rx;


	if (argc > 6 || argc < 2) {
		usage();
		return -1;
	}

	printf("%s\n\n", TOOL_VERSION);

	memset(dev_name, 0, 100);

	strncpy(dev_name, argv[1], sizeof(dev_name));
	// parameter
	while (EOF != (s = getopt(argc, argv, "m:i:h"))) {
		switch (s) {
			case 'm':
			{
				unsigned char mode = (atoi(optarg) & 0xFF);
				if(mode < 1 || mode > 4) {
					printf("Please select 1:Master 2:Slave SW 3:Slave HW 4:Data mode\n");
					return -1;
				}
				config.slave_mode = mode;
				break;
			}
			case 'i':
			{
				config.slave_id = (atoi(optarg) & 0xFF);
				break;
			}
			case 'h':
			default:
			{
				usage();
				return -1;
			}
		}
	}	
	// open dev
	devfd = open(dev_name, (O_RDWR| O_NOCTTY));	
	if (devfd < 0)
	{
		printf("error %d opening %s: %s\n", errno, dev_name, strerror (errno));
		return -1;
	}
	set_interface_attribs (devfd, B115200, 0);
	tcflush(devfd,TCIOFLUSH);
	if (ioctl(devfd, IOCTL_SET_SLAVE_MODE, &config) < 0) {
		printf("IOCTL_SET_SLAVE_MODE failed.\n");
		return -1;
	}	

	system("clear");
	printf("%s\n", TOOL_VERSION);
	printf("Mode: %d ID: %d\n", config.slave_mode, config.slave_id);

	rx = pthread_create(&thread_rx, NULL, rx_function, NULL);
	if (rx) {
		printf("ERROR - pthread_create return code: %d\n",rx);
		exit(EXIT_FAILURE);
	}	
	while (1) {
		tx = getchar();
		if (tx == '\n')
			continue;
		if (config.slave_mode == MODE_9BIT_MASTER) {
			start_val = 1;
			if (ioctl(devfd, IOCTL_SET_9BIT_DATA, &start_val) < 0) {
				printf("IOCTL_SET_9BIT_DATA failed.\n");
				return -1;
			}
			address = (config.slave_id & 0xFF);
			write(devfd, &address, 1);
			usleep(100 * 1000);
		}
		start_val = 0;
		if (ioctl(devfd, IOCTL_SET_9BIT_DATA, &start_val) < 0) {
			printf("IOCTL_SET_9BIT_DATA failed.\n");
			return -1;
		}				
		write(devfd, &tx, 1);
		usleep(100 * 1000);
	};

	close(devfd);
	return 0;
}
