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

static uint32_t crc32_calc(uint8_t* buff)
{
    static const uint32_t crc_tbl[256] = {
        0x00000000,0x77073096,0xEE0E612C,0x990951BA,0x076DC419,0x706AF48F,0xE963A535,
        0x9E6495A3,0x0EDB8832,0x79DCB8A4,0xE0D5E91E,0x97D2D988,0x09B64C2B,0x7EB17CBD,
        0xE7B82D07,0x90BF1D91,0x1DB71064,0x6AB020F2,0xF3B97148,0x84BE41DE,0x1ADAD47D,
        0x6DDDE4EB,0xF4D4B551,0x83D385C7,0x136C9856,0x646BA8C0,0xFD62F97A,0x8A65C9EC,
        0x14015C4F,0x63066CD9,0xFA0F3D63,0x8D080DF5,0x3B6E20C8,0x4C69105E,0xD56041E4,
        0xA2677172,0x3C03E4D1,0x4B04D447,0xD20D85FD,0xA50AB56B,0x35B5A8FA,0x42B2986C,
        0xDBBBC9D6,0xACBCF940,0x32D86CE3,0x45DF5C75,0xDCD60DCF,0xABD13D59,0x26D930AC,
        0x51DE003A,0xC8D75180,0xBFD06116,0x21B4F4B5,0x56B3C423,0xCFBA9599,0xB8BDA50F,
        0x2802B89E,0x5F058808,0xC60CD9B2,0xB10BE924,0x2F6F7C87,0x58684C11,0xC1611DAB,
        0xB6662D3D,0x76DC4190,0x01DB7106,0x98D220BC,0xEFD5102A,0x71B18589,0x06B6B51F,
        0x9FBFE4A5,0xE8B8D433,0x7807C9A2,0x0F00F934,0x9609A88E,0xE10E9818,0x7F6A0DBB,
        0x086D3D2D,0x91646C97,0xE6635C01,0x6B6B51F4,0x1C6C6162,0x856530D8,0xF262004E,
        0x6C0695ED,0x1B01A57B,0x8208F4C1,0xF50FC457,0x65B0D9C6,0x12B7E950,0x8BBEB8EA,
        0xFCB9887C,0x62DD1DDF,0x15DA2D49,0x8CD37CF3,0xFBD44C65,0x4DB26158,0x3AB551CE,
        0xA3BC0074,0xD4BB30E2,0x4ADFA541,0x3DD895D7,0xA4D1C46D,0xD3D6F4FB,0x4369E96A,
        0x346ED9FC,0xAD678846,0xDA60B8D0,0x44042D73,0x33031DE5,0xAA0A4C5F,0xDD0D7CC9,
        0x5005713C,0x270241AA,0xBE0B1010,0xC90C2086,0x5768B525,0x206F85B3,0xB966D409,
        0xCE61E49F,0x5EDEF90E,0x29D9C998,0xB0D09822,0xC7D7A8B4,0x59B33D17,0x2EB40D81,
        0xB7BD5C3B,0xC0BA6CAD,0xEDB88320,0x9ABFB3B6,0x03B6E20C,0x74B1D29A,0xEAD54739,
        0x9DD277AF,0x04DB2615,0x73DC1683,0xE3630B12,0x94643B84,0x0D6D6A3E,0x7A6A5AA8,
        0xE40ECF0B,0x9309FF9D,0x0A00AE27,0x7D079EB1,0xF00F9344,0x8708A3D2,0x1E01F268,
        0x6906C2FE,0xF762575D,0x806567CB,0x196C3671,0x6E6B06E7,0xFED41B76,0x89D32BE0,
        0x10DA7A5A,0x67DD4ACC,0xF9B9DF6F,0x8EBEEFF9,0x17B7BE43,0x60B08ED5,0xD6D6A3E8,
        0xA1D1937E,0x38D8C2C4,0x4FDFF252,0xD1BB67F1,0xA6BC5767,0x3FB506DD,0x48B2364B,
        0xD80D2BDA,0xAF0A1B4C,0x36034AF6,0x41047A60,0xDF60EFC3,0xA867DF55,0x316E8EEF,
        0x4669BE79,0xCB61B38C,0xBC66831A,0x256FD2A0,0x5268E236,0xCC0C7795,0xBB0B4703,
        0x220216B9,0x5505262F,0xC5BA3BBE,0xB2BD0B28,0x2BB45A92,0x5CB36A04,0xC2D7FFA7,
        0xB5D0CF31,0x2CD99E8B,0x5BDEAE1D,0x9B64C2B0,0xEC63F226,0x756AA39C,0x026D930A,
        0x9C0906A9,0xEB0E363F,0x72076785,0x05005713,0x95BF4A82,0xE2B87A14,0x7BB12BAE,
        0x0CB61B38,0x92D28E9B,0xE5D5BE0D,0x7CDCEFB7,0x0BDBDF21,0x86D3D2D4,0xF1D4E242,
        0x68DDB3F8,0x1FDA836E,0x81BE16CD,0xF6B9265B,0x6FB077E1,0x18B74777,0x88085AE6,
        0xFF0F6A70,0x66063BCA,0x11010B5C,0x8F659EFF,0xF862AE69,0x616BFFD3,0x166CCF45,
        0xA00AE278,0xD70DD2EE,0x4E048354,0x3903B3C2,0xA7672661,0xD06016F7,0x4969474D,
        0x3E6E77DB,0xAED16A4A,0xD9D65ADC,0x40DF0B66,0x37D83BF0,0xA9BCAE53,0xDEBB9EC5,
        0x47B2CF7F,0x30B5FFE9,0xBDBDF21C,0xCABAC28A,0x53B39330,0x24B4A3A6,0xBAD03605,
        0xCDD70693,0x54DE5729,0x23D967BF,0xB3667A2E,0xC4614AB8,0x5D681B02,0x2A6F2B94,
        0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D };

    uint32_t i, crc32 = 0xFFFFFFFF;
    for(i = 0; i < 0x20 - sizeof(crc32); i++) {
        crc32 = (crc32 >> 8) ^ crc_tbl[(crc32 ^ buff[i]) & 0xFF];
    }   
    return ~crc32;
}

//static void dump_buff(uint8_t buff[0x20])
//{       
//    int i = 0;
//    for(; i < 0x20; i ++) {
//        fprintf(stderr, "%02X", buff[i]);
//    }   
//    fprintf(stderr, "\n");
//}       


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

static int i2c_open_dev(int* fd, const char* dev_name)
{
    *fd = open(I2C_DEV_NAME, O_RDWR);

    if(*fd < 0) {
        perror(dev_name);
        return 3;
    }
    return 0;
}

static int i2c_set_addr(int fd, uint8_t addr)
{
    if(0 > ioctl(fd, I2C_SLAVE, addr)) {
        perror("Set i2c address");
        return 4;
    }
    return 0;
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
    int fd;
    
    if((res = i2c_open_dev(&fd, I2C_DEV_NAME)))
        return res;

    do {
        if((res = i2c_set_addr(fd, LM75_ADDRESS)))
            break;

        if(lm75_measure(fd, &t)) {
            res = 5;
            break;
        }
        printf("LM75: T=%.1f °C\n", t);

        if((res = i2c_set_addr(fd, BMP180_ADDRESS)))
            break;

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

    *pp++ = 0; //type
    *pp++ = 0; //dev id
    *pp++ = reg_state;
    write_32bit_val(&pp, &t);
    write_32bit_val(&pp, &tf);
    write_32bit_val(&pp, &pf);
    write_32bit_val(&pp, &hg);
    write_32bit_val(&pp, &hf);

    pp = buff + 0x20 - sizeof(uint32_t);
    uint32_t crc32 = crc32_calc(buff);
    write_32bit_val(&pp, &crc32);

    return 0;
}

static int bmp180_main()
{
    int res = 0;
    int i2c_fd;

    if((res = i2c_open_dev(&i2c_fd, I2C_DEV_NAME)))
        return res;

    do {
        if((res = i2c_set_addr(i2c_fd, BMP180_ADDRESS)))
            break;

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
    int i2c_fd;

    if((res = i2c_open_dev(&i2c_fd, I2C_DEV_NAME)))
        return res;

    do {
        if((res = i2c_set_addr(i2c_fd, LM75_ADDRESS)))
            break;

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
//        dump_buff(buff);

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

