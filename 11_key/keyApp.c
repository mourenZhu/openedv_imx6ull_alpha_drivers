#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"

#define KEY0VALUE 0XF0
#define INVAKEY 0X00

int main(int argc, char *argv[]) {
    int fd, ret;
    char *filename;
    unsigned char keyvalue;

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
        read(fd, &keyvalue, sizeof(keyvalue));
        if (keyvalue == KEY0VALUE) { /*KEY0*/
            printf("KEY0 Press, value = %#X\r\n", keyvalue); /*按下*/
        }
    }
    
    ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed\r\n", argv[1]);
        return -1;
    }
    return 0;
}