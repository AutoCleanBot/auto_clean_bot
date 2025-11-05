#pragma once

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

class Serial {
  public:
    /**
     * tty_setopt - config tty device
     * @fd: device handle
     * @speed: baud rate to set
     * @databits: data bits to set
     * @stopbits: stop bits to set
     * @parity: parity set
     *
     * The function return 0 if success, or -1 if fail.
     */
    static int TTYSetOpt(int fd, int speed, int databits, int stopbits, char parity);

    /**
     * TTYOpen - open tty device
     * @devname: the device name to open
     *
     * In this demo device is opened blocked, you could modify it at will.
     */
    static int TTYOpen(const char *devname);

    /**
     * tty_close - close tty device
     * @fd: the device handle
     *
     */
    static int TTYClose(int fd);

    /**
     * tty_write - write data
     * @fd: the device handle
     *
     */
    static int TTYWrite(int fd, char *buf, int size);
};
