#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <math.h>

#define BMP180_ADDRESS		0x77
#define LM75_ADDRESS        0x4F

static void ctrl_c(int sig)
{
    signal(SIGINT, ctrl_c);
    close(0);
}

static int i2c_read_reg(int i2c_fd, uint8_t reg, uint8_t* buff, uint8_t count)
{
    int res = -1;

    if(1 == write(i2c_fd, &reg, 1)) {
        res = read(i2c_fd, buff, count);
    }

    return res;
}

static int i2c_dev_func(const char* i2c_dev_name, uint8_t addr, int (*proc)(int i2c_fd, void*), void* ctx)
{
    int i2c_fd = open(i2c_dev_name, O_RDWR);
    if(i2c_fd < 0) {
        perror(i2c_dev_name);
        return 3;
    }

    if(0 > ioctl(i2c_fd, I2C_SLAVE, addr)) {
        close(i2c_fd);
        perror("Set I2C Address");
        return 4;
    }

    int res = proc(i2c_fd, ctx);

    close(i2c_fd);
    return res;
}

static int reg_func(int (*proc)(volatile void*, void*), void* ctx)
{
	static const char* dev_name = "/dev/mem";
	int fd = open(dev_name, O_RDWR | O_SYNC);
	if(fd == -1) {
		perror(dev_name);
		return 5;
	}
	volatile void* reg_base = mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x10000000);
	close(fd);
	if(MAP_FAILED == reg_base) {
		perror("mmap");
        return 6;
	}
    int res = proc(reg_base, ctx);
    munmap((void*)reg_base, 0x1000);
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

static int bmp180_measure_proc(int i2c_fd, void* ctx)
{
    float tf = 0;
    float pf = 0;
    float hg = 0;
    float hf = 0;
    if(bmp180_measure(i2c_fd, &tf, &pf, &hg, &hf))
        return 5;

    printf("BMP180: T=%.1f °C, P=%.2f hPa (%.2f mm, %.1f m)\n", tf, pf, hg, hf);
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

static int lm75_measure_proc(int i2c_fd, void* ctx)
{
    float t = 0;

    if(lm75_measure(i2c_fd, &t))
        return 10;
    printf("LM75: T=%.1f °C\n", t);

    return 0;
}

static int gpio_wait_37_down(volatile void* reg_base, void* ctx)
{
    volatile uint8_t* reg_data = (volatile uint8_t*)reg_base;
    reg_data += 0x624;
    while(*reg_data & 0b100000)
        usleep(100);
    printf("==>> %d\n", !!(*reg_data & 0b100000));
    return 0;
}

static int bmp180_main()
{
    return i2c_dev_func("/dev/i2c-0", BMP180_ADDRESS, bmp180_measure_proc, 0);
}

static int lm75_main()
{
    return i2c_dev_func("/dev/i2c-0", LM75_ADDRESS, lm75_measure_proc, 0);
}

static int gpio_wait_main()
{
    return reg_func(gpio_wait_37_down, 0);
}

int main(int argc, char* argv[])
{
    signal(SIGINT, ctrl_c);
    if(2 != argc) {
        fprintf(stderr, "Usage:\n"
        "\t%s bmp180\n"
        "\t%s lm75\n"
        "\t%s gpio_wait\n",
        *argv,
        *argv,
        *argv);
        return 1;
    }
    argc --;
    argv ++;
    if(!strcmp(*argv, "bmp180"))
        return bmp180_main();
    if(!strcmp(*argv, "lm75"))
        return lm75_main();
    if(!strcmp(*argv, "gpio_wait"))
        return gpio_wait_main();
    fprintf(stderr, "Unknown subcommand: %s\n", *argv);
    return 2;
}

