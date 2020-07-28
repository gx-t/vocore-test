#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

static void gpio_op(uint8_t* reg_base)
{
    switch(getchar()) {
        case -1:
            return;
        case 0:
            reg_base[0x644] |= 0b100000;
            break;
        case 1:
            reg_base[0x634] |= 0b100000;
            break;
    }
    printf("Access-Control-Allow-Origin: *\r\nContent-type: text/plain, charset=utf-8\r\n\r\n");
    printf("%d\n", !!(reg_base[0x624] & 0b100000));
}

int main()
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(fd == -1)
        return 2;

    uint8_t* reg_base = mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x10000000);
    close(fd);

    if((void*)-1 == reg_base) {
        perror("mmap");
        return 3;
    }

    gpio_op(reg_base);

    munmap((void*)reg_base, 0x1000);
    return 0;
}

