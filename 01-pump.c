#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>

static void pump_status(uint8_t* reg_base)
{
    time_t tt = time(0);
    printf("Access-Control-Allow-Origin: *\r\nContent-type: application/json\r\n\r\n");
    printf("{\"state\":\"%s\", \"time\":%lu}\n", (reg_base[0x624] & 0b100000) ? "Միացված է" : "Անջատված է", tt);
}

static void pump_off(uint8_t* reg_base)
{
    reg_base[0x644] |= 0b100000;
    pump_status(reg_base);
}

static void pump_on(uint8_t* reg_base)
{
    reg_base[0x634] |= 0b100000;
    pump_status(reg_base);
}

static int pump_op(uint8_t* reg_base)
{
    const void(*proc_tbl[])(uint8_t*) = {
        pump_status
            , pump_off
            , pump_on
    };

    int op = getchar();
    if(-1 == op || sizeof(proc_tbl) / sizeof(proc_tbl[0]) <= op)
        return 3;

    proc_tbl[op](reg_base);
    return 0;
}

int main()
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(fd == -1)
        return 1;

    uint8_t* reg_base = mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x10000000);
    close(fd);

    if((void*)-1 == reg_base) {
        perror("mmap");
        return 2;
    }

    int res = pump_op(reg_base);

    munmap((void*)reg_base, 0x1000);
    return res;
}

