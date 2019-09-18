#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <errno.h>

#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <linux/i2c-dev.h>

#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

#define I2C_DEV_NAME        "/dev/i2c-0"
#define MEM_DEV_NAME        "/dev/mem"

#define REG_BASE_ADDR       0x10000000
#define PAGE_SIZE           0x1000

#define BMP180_ADDRESS		0x77
#define LM75_ADDRESS        0x4F

static int g_run = 1;

static void ctrl_c(int sig)
{
    fprintf(stderr, "\nSIGINT (%d)\n", getpid());
    g_run = 0;
}

static int i2c_read_reg(int i2c_fd, uint8_t reg, uint8_t* buff, uint8_t count)
{
    int res = -1;

    if(1 == write(i2c_fd, &reg, 1)) {
        res = read(i2c_fd, buff, count);
    }

    return res;
}

static int bmp180_measure(int i2c_fd, float* tf, float* pf, float* hg, float* hf)
{
    uint8_t buff[32] = {0};

    //read calibration data

    if(22 != i2c_read_reg(i2c_fd, 0xAA, buff, 22)) {
        fprintf(stderr, "BMP180: error reading calibration data\n");
        return -1;
    }

    short ac1, ac2, ac3, b1, b2, mc, md;

    unsigned short ac4, ac5, ac6;
    ac1 = buff[0]  << 8 | buff[1];
    ac2 = buff[2]  << 8 | buff[3];
    ac3 = buff[4]  << 8 | buff[5];
    ac4 = buff[6]  << 8 | buff[7];
    ac5 = buff[8]  << 8 | buff[9];
    ac6 = buff[10] << 8 | buff[11];
    b1  = buff[12] << 8 | buff[13];
    b2  = buff[14] << 8 | buff[15];
    mc  = buff[18] << 8 | buff[19];
    md  = buff[20] << 8 | buff[21];

    //init temp. measurement

    buff[0] = 0xF4;
    buff[1] = 0x2e;
    if(2 != write(i2c_fd, buff, 2)) {
        fprintf(stderr, "BMP180: cannot initiate temperature measurement\n");
        return -1;
    }

    //wait for ADC to complete measurement

    if(0 != usleep(5000)) {
        fprintf(stderr, "BMP180: interrupted by SIGINT\n");
        return -1;
    }

    //read uncompensated temperarure

    if(2 != i2c_read_reg(i2c_fd, 0xF6, buff, 2)) {
        fprintf(stderr, "BMP180: error reading register 0xF6\n");
        return -1;
    }

    long ut = buff[0] << 8 | buff[1];

    //init pres. measurement

    buff[0] = 0xF4;
    buff[1] = 0xF4;
    if(2 != write(i2c_fd, buff, 2)) {
        fprintf(stderr, "BMP180: cannot initiate pressure measurement\n");
        return -1;
    }

    //wait for ADC to complete measurement

    if(0 != usleep(26000)) {
        fprintf(stderr, "BMP180: interrupted by SIGINT\n");
        return -1;
    }

    //read uncompensated pressure

    if(3 != i2c_read_reg(i2c_fd, 0xF6, buff, 3)) {
        fprintf(stderr, "BMP180: error reading register 0xF6\n");
        return -1;
    }
    long up = (buff[0] << 16 | buff[1] << 8 | buff[2]) >> 5;

    //calculate true temperature

    long x1, x2, x3, b3, b5, b6;
    unsigned long b4, b7;
    x1 = (ut - ac6) * ac5 >> 15;
    x2 = (mc << 11) / (x1 + md);
    b5 = x1 + x2;
    float t = (b5 + 8) >> 4;
    t /= 10;

    //calculate true pressure

    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;

    b3 = (((ac1 * 4 + x3) << 3) + 2) >> 2;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = (x1 + x2 + 2) >> 2;
    b4 = (ac4 * (x3 + 32768)) >> 15;
    b7 = (up - b3) * (50000 >> 3);

    long p;
    if(b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += ((x1 + x2 + 3791) >> 4);

    *tf = t;
    *pf = p;

    *hg = *pf * 0.00750062;
    *hf = 44330 * (1 - pow(*pf / 101325, 1.0 / 5.255));
    *pf /= 100.0;

    return 0;
}

static int lm75_measure(int i2c_fd, float* temp)
{
    uint8_t buff[2] = {0};

    if(2 != i2c_read_reg(i2c_fd, 0x00, buff, 2)) {
        fprintf(stderr, "LM75: Cannot Read Sensor Data\n");
        return -1;
    }

    *temp = (float)(((short)(buff[0] << 8) | buff[1]) / 128);
    *temp /= 2;
    return 0;
}

static void fill_rand(uint8_t* pp)
{
    int i = 0x20;
    while(i --)
        *pp ++ = rand() % 0x100;
}

static void write_8bit_val(uint8_t** pp, uint8_t val)
{
    *(*pp)++ = val;
}

static void write_32bit_val(uint8_t** pp, void* val)
{
    uint8_t* pv = (uint8_t*)val;

#if ((__BYTE_ORDER__) == (__ORDER_LITTLE_ENDIAN__))
    *(*pp)++ = *pv++;
    *(*pp)++ = *pv++;
    *(*pp)++ = *pv++;
    *(*pp)++ = *pv;
#else
    pv += 3;
    *(*pp)++ = *pv--;
    *(*pp)++ = *pv--;
    *(*pp)++ = *pv--;
    *(*pp)++ = *pv;
#endif
}

static int udp_measure_fill_buffer(uint8_t buff[0x20])
{
    int res = 0;

    float t = 0;
    float tf = 0;
    float pf = 0;
    float hg = 0;
    float hf = 0;
    uint8_t reg_state = 0;

    int fd = open(I2C_DEV_NAME, O_RDWR);

    if(fd < 0) {
        perror(I2C_DEV_NAME);
        return 3;
    }

    do {
        if(0 > ioctl(fd, I2C_SLAVE, LM75_ADDRESS)) {
            res = 4;
            perror("Set LM73 i2c address");
            break;
        }

        if(lm75_measure(fd, &t)) {
            res = 5;
            break;
        }
        printf("LM75: T=%.1f °C\n", t);

        if(0 > ioctl(fd, I2C_SLAVE, BMP180_ADDRESS)) {
            res = 4;
            perror("Set BMP180 i2c address");
            break;
        }

        if(bmp180_measure(fd, &tf, &pf, &hg, &hf)) {
            res = 5;
            break;
        }
        printf("BMP180: T=%.1f °C, P=%.2f hPa (%.2f mm, %.1f m)\n", tf, pf, hg, hf);

    } while(0);

    close(fd);

    if(res) {
        return res;
    }

    fd = open(MEM_DEV_NAME, O_RDWR | O_SYNC);
    if(fd == -1) {
        perror(MEM_DEV_NAME);
        return 3;
    }

    volatile void* reg_base = mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, REG_BASE_ADDR);
    close(fd);
    if(MAP_FAILED == reg_base) {
        perror("mmap");
        return 4;
    }

    volatile uint8_t* reg_data = (volatile uint8_t*)reg_base;
    reg_data += 0x624;
    reg_state = !!(*reg_data & 0b100000);
    printf("GPIO: %d\n", reg_state);
    munmap((void*)reg_base, PAGE_SIZE);


    fill_rand(buff);

    uint8_t* pp = buff;

    write_8bit_val(&pp, 0); //dev id
    write_8bit_val(&pp, reg_state);
    write_32bit_val(&pp, &t);
    write_32bit_val(&pp, &tf);
    write_32bit_val(&pp, &pf);
    write_32bit_val(&pp, &hg);
    write_32bit_val(&pp, &hf);

    pp = buff + 0x20 - sizeof(uint32_t);
    uint32_t crc32 = 0;
    write_32bit_val(&pp, &crc32);

    return 0;
}

static int bmp180_main()
{
    int res = 0;
    int i2c_fd = open(I2C_DEV_NAME, O_RDWR);

    if(i2c_fd < 0) {
        perror(I2C_DEV_NAME);
        return 3;
    }

    do {
        if(0 > ioctl(i2c_fd, I2C_SLAVE, BMP180_ADDRESS)) {
            res = 4;
            perror("Set BMP180 i2c address");
            break;
        }

        float tf = 0;
        float pf = 0;
        float hg = 0;
        float hf = 0;
        if(bmp180_measure(i2c_fd, &tf, &pf, &hg, &hf)) {
            res = 5;
            break;
        }
        printf("BMP180: T=%.1f °C, P=%.2f hPa (%.2f mm, %.1f m)\n", tf, pf, hg, hf);

    } while(0);

    close(i2c_fd);
    return res;
}

static int lm75_main()
{
    int res = 0;
    int i2c_fd = open(I2C_DEV_NAME, O_RDWR);

    if(i2c_fd < 0) {
        perror(I2C_DEV_NAME);
        return 3;
    }

    do {
        if(0 > ioctl(i2c_fd, I2C_SLAVE, LM75_ADDRESS)) {
            res = 4;
            perror("Set LM73 i2c address");
            break;
        }

        float t = 0;
        if(lm75_measure(i2c_fd, &t)) {
            res = 5;
            break;
        }

        printf("LM75: T=%.1f °C\n", t);
    } while(0);

    close(i2c_fd);
    return res;
}

static int gpio_main()
{
    int fd = open(MEM_DEV_NAME, O_RDWR | O_SYNC);
    if(fd == -1) {
        perror(MEM_DEV_NAME);
        return 5;
    }

    volatile void* reg_base = mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, REG_BASE_ADDR);
    close(fd);
    if(MAP_FAILED == reg_base) {
        perror("mmap");
        return 6;
    }

    volatile uint8_t* reg_data = (volatile uint8_t*)reg_base;
    reg_data += 0x624;
    printf("GPIO: %d\n", !!(*reg_data & 0b100000));
    munmap((void*)reg_base, PAGE_SIZE);
    return 0;
}

static int udp_main(int argc, char* argv[])
{
    int res = 0;
    if(argc != 3)
        return 1;

    argc --;
    argv ++;
    int ss = socket(AF_INET, SOCK_DGRAM, 0);
    if(ss < 0) {
        perror("socket");
        return 3;
    }
    struct timeval tv = {1, 0};
    if(0 > setsockopt(ss, SOL_SOCKET, SO_RCVTIMEO, (struct timeval*)&tv, sizeof(struct timeval))) {
        perror("setsockopt");
        close(ss);
        return 4;
    }

    int port = atoi(argv[1]);
    if(port < 1024 || port > (1 << 16) - 1)
        port = 27727;

    while(g_run) {

        struct sockaddr_in addr = {.sin_family = AF_INET};
        struct hostent* he = gethostbyname(argv[0]);
        if(!he) {
            perror("gethostbyname");
            res = 5;
            break;
        }

        addr.sin_addr.s_addr = *(uint32_t*)he->h_addr_list[0];
        addr.sin_port = htons(port);

        uint8_t buff[32];
        res = udp_measure_fill_buffer(buff);

        socklen_t addr_len = sizeof(addr);
        int len = 0;
        len = sendto(ss, buff, sizeof(buff), 0, (struct sockaddr *)&addr, addr_len);
        if(len != sizeof(buff)) {
            if(EINTR == errno)
                break;
            if(!g_run) {
                perror("sendto");
                res = 6;
            }
            break;
        }

        int cnt = 60;
        while(g_run && cnt) {
            len = recvfrom(ss, buff, sizeof(buff), 0, (struct sockaddr*)&addr, &addr_len);
            if(len < 0) {
                if(EINTR == errno)
                    break;

                if(EAGAIN == errno) {
                    cnt --;
                    continue;
                }

                perror("recvfrom");
                res = 7;
                break;
            }
            if(!len) {
                fprintf(stderr, "PING: %s:%d:\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
                continue;
            }
            if(len == sizeof(buff)) {
                break;
            }
        }
    }
    close(ss);
    return res;
}

int main(int argc, char* argv[])
{
    if(argc < 2) {
        fprintf(stderr, "Usage:\n"
                "\t%s bmp180\n"
                "\t%s lm75\n"
                "\t%s gpio\n"
                "\t%s udp\n",
                *argv,
                *argv,
                *argv,
                *argv);
        return 1;
    }

    argc --;
    argv ++;

    signal(SIGINT, ctrl_c);
    srand(time(0));

    if(!strcmp(*argv, "bmp180"))
        return bmp180_main();
    if(!strcmp(*argv, "lm75"))
        return lm75_main();
    if(!strcmp(*argv, "gpio"))
        return gpio_main();
    if(!strcmp(*argv, "udp"))
        return udp_main(argc, argv);

    fprintf(stderr, "Unknown subcommand: %s\n", *argv);
    return 2;
}

