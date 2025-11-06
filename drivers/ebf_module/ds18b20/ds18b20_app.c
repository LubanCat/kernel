#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

int main(int argc, char *argv[])
{
    const char *dev = "/dev/ds18b20";

    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    printf("Reading temperature from %s ...\n", dev);

    char buf[64];
    while (1) {
		
        memset(buf, 0, sizeof(buf));

        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        if (n < 0) {
            perror("read");
            sleep(1);
            continue;
        }

        // 去除换行符
        buf[strcspn(buf, "\r\n")] = '\0';
        printf("Temperature: %s °C\n", buf);

        sleep(1); // 每秒读取一次
    }

    close(fd);
    return 0;
}