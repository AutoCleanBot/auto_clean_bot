#include "gps_serial.h"
//stty -F /dev/ttyUSB0 查看串口输出波特率
int speed_arr[] = {B460800, B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arr[] = {460800, 230400, 115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};

/**
 * libtty_setopt - config tty device
 * @fd: device handle
 * @speed: baud rate to set
 * @databits: data bits to set 有效数据长度，数据位数
 * @stopbits: stop bits to set 停止位数
 * @parity: parity set 校验位类型
 *
 * The function return 0 if success, or -1 if fail.
 */
int Serial::TTYSetOpt(int fd, int speed, int databits, int stopbits, char parity) {
    struct termios newtio;
    struct termios oldtio;
    unsigned int i;

    bzero(&newtio, sizeof(newtio));
    bzero(&oldtio, sizeof(oldtio));

    if (tcgetattr(fd, &oldtio) != 0) { //tcgetattr 从 fd 获取当前终端属性到 oldtio。
        perror("tcgetattr");
        return -1;
    }
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    /* set tty speed 设置波特率*/
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
        if (speed == name_arr[i]) {
            cfsetispeed(&newtio, speed_arr[i]); //用 cfsetispeed / cfsetospeed 把输入/输出波特率写入 newtio。
            cfsetospeed(&newtio, speed_arr[i]);
        }
    }

    /* set data bits 设置数据位*/
    switch (databits) {
    case 5:
        newtio.c_cflag |= CS5;
        break;
    case 6:
        newtio.c_cflag |= CS6;
        break;
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "unsupported data size\n");
        return -1;
    }

    /* set parity 设置校验位*/
    switch (parity) {
    case 'n':
    case 'N':
        newtio.c_cflag &= ~PARENB; /* Clear parity enable 禁用*/
        newtio.c_iflag &= ~INPCK;  /* Disable input parity check */
        break;
    case 'o':
    case 'O':
        newtio.c_cflag |= (PARODD | PARENB); /* Odd parity instead of even 奇校验*/
        newtio.c_iflag |= INPCK;             /* Enable input parity check */
        break;
    case 'e':
    case 'E':
        newtio.c_cflag |= PARENB;  /* Enable parity */
        newtio.c_cflag &= ~PARODD; /* Even parity instead of odd 偶校验*/
        newtio.c_iflag |= INPCK;   /* Enable input parity check */
        break;
    default:
        fprintf(stderr, "unsupported parity\n");
        return -1;
    }

    /* set stop bits 设置停止位*/
    switch (stopbits) {
    case 1:
        newtio.c_cflag &= ~CSTOPB;
        break;
    case 2:
        newtio.c_cflag |= CSTOPB;
        break;
    default:
        perror("unsupported stop bits\n");
        return -1;
    }

    newtio.c_cc[VTIME] = 0; /* Time-out value (tenths of a second) [!ICANON]. */
    newtio.c_cc[VMIN] = 0;  /* Minimum number of bytes read at once [!ICANON]. */

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
        perror("tcsetattr"); //把 newtio 里的所有配置参数，应用到串口设备 fd 上。
        return -1;
    }
    return 0;
}

/**
 * libtty_open - open tty device
 * @devname: the device name to open
 *
 * In this demo device is opened blocked, you could modify it at will.
 */
int Serial::TTYOpen(const char *devname) {
    int fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY); //调用了 Linux 系统调用 open() 返回文件描述符
    int flags = 0;

    if (fd == -1) {
        perror("open device failed");
        return -1;
    }

    flags = fcntl(fd, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    if (fcntl(fd, F_SETFL, flags) < 0) {
        printf("fcntl failed.\n");
        return -1;
    }

    if (isatty(fd) == 0) {
        printf("not %s device.\n", devname);
        return -1;
    } else
        printf("%s device ok.\n", devname);

    return fd;
}

/**
 * libtty_close - close tty device
 * @fd: the device handle
 *
 */
int Serial::TTYClose(int fd) { return close(fd); }

/**
 * libtty_write - write data
 * @fd: the device handle
 *
 */
int Serial::TTYWrite(int fd, char *buf, int size) {
    int wr = -1;
    if (write(fd, buf, size) > 0)
        wr = 1;
    return wr;
}

/*
void tty_test(int fd)
{
    int nwrite, nread;
    char buf[100];

    memset(buf, 0x32, sizeof(buf));

    while (1) {
        nwrite = write(fd, buf, sizeof(buf));
        printf("wrote %d bytes already.\n", nwrite);
        nread = read(fd, buf, sizeof(buf));
        printf("read %d bytes already.\n", nread);
        sleep(2);
    }
}

int main(int argc, char *argv[])
{
    int fd;
    int ret;

    fd = libtty_open("/dev/ttymxc2");
    if (fd < 0) {
        printf("libtty_open error.\n");
        exit(0);
    }

    ret = libtty_setopt(fd, 9600, 8, 1, 'n');
    if (ret != 0) {
        printf("libtty_setopt error.\n");
        exit(0);
    }

    tty_test(fd);

    ret = libtty_close(fd);
    if (ret != 0) {
        printf("libtty_close error.\n");
        exit(0);
    }
}*/
