#include "gt_utils.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <string.h>

#define GT911_REG_STATUS 0x814E
#define GT911_REG_POINTS 0x814F
#define GT911_CLEAR 0x00


static int ts_read(GT911* ts, uint16_t reg, uint8_t* buf, int len) {
    uint8_t addr[2] = { reg >> 8, reg & 0xFF };
    if (write(ts->fd, addr, 2) != 2) return -1;
    return read(ts->fd, buf, len);
}

int gt911_init(GT911 *ts, const char *i2c_dev) {
    ts->device_addr = GT911_ADDR;

    ts->fd = open(i2c_dev, O_RDWR);
    if (ts->fd < 0) {
        perror("GT911: cannot open I2C");
        return -1;
    }

    if (ioctl(ts->fd, I2C_SLAVE, ts->device_addr) < 0) {
        perror("GT911: cannot set I2C addr");
        return -1;
    }

    printf("GT911: initialized at I2C addr 0x%X\n", ts->device_addr);
    return 0;
}

int gt911_read_touch(GT911 *ts) {
    uint8_t status;
    if (ts_read(ts, GT911_REG_STATUS, &status, 1) != 1) {
        perror("GT911: status read failed");
        return -1;
    }

    ts->num_points = status & 0x0F;
    if (ts->num_points == 0 || ts->num_points > GT911_MAX_POINTS) {
        return 0;
    }

    uint8_t buf[GT911_MAX_POINTS * 8];
    if (ts_read(ts, GT911_REG_POINTS, buf, ts->num_points * 8) < 0) {
        perror("GT911: points read failed");
        return -1;
    }

    for (int i = 0; i < ts->num_points; i++) {
        int offset = i * 8;
        ts->x[i] = buf[offset + 1] | (buf[offset + 2] << 8);
        ts->y[i] = buf[offset + 3] | (buf[offset + 4] << 8);
    }

 
    uint8_t clear = GT911_CLEAR;
    uint8_t addr[2] = { GT911_REG_STATUS >> 8, GT911_REG_STATUS & 0xFF };
    write(ts->fd, addr, 2);
    write(ts->fd, &clear, 1);

    return ts->num_points;
}

void gt911_close(GT911 *ts) {
    close(ts->fd);
}
