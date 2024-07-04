#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include "linux/ioctl.h"

int main(int argc, char *argv[]) {
    int fd, ret = 0;
    char *filename;
    unsigned char data;

    if (argc != 2) {
        printf("Error Usage!\r\n");
        return -1;
    }

    filename = argv[1];

    fd = open(filename, O_RDWR);
    if (fd < 0) {
        printf("file %s open failed! err code:%d\r\n", argv[1], fd);
        return -1;
    }

    while (1)
    {
        ret = read(fd, &data, sizeof(data));
        if (ret < 0) { /*数据读取错误或者失效*/
            
        } else {
            if (data) /*读取到数据*/
                printf("key value = %#X\r\n", data);
        }
    }
    
    close(fd);
    return ret;
}