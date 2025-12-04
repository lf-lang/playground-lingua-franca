#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE_EXTENDED 1
#include "capacitive_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <time.h>
#include <gpiod.h>

#define I2C_DEV_PATH "/dev/i2c-1"
static int i2c_fd = -1;

#define GPIO_RST 17
#define GPIO_INT 27

static struct gpiod_chip *gpiochip = NULL;
static struct gpiod_line *line_rst = NULL;
static struct gpiod_line *line_int = NULL;

/* GT911 registers */
#define GT911_PRODUCT_ID       0x8140
#define GT911_VERSION          0x8144
#define GT911_CONFIG_START     0x8047
#define GT911_CONFIG_LEN       186
#define GT911_POINTS_REG       0x814E
#define GT911_STATUS_REG       0x814E
#define GT911_POINT1_REG       0x8150
#define GT911_POINT_REG_LEN    8
#define GT911_CMD_REG          0x8040

int GT911_I2C_Write(uint16_t reg, uint8_t *buf, uint16_t len) {
    if (i2c_fd < 0) return -1;

    uint8_t out[2 + len];
    out[0] = (reg >> 8) & 0xFF;
    out[1] = reg & 0xFF;
    memcpy(out + 2, buf, len);

    ssize_t w = write(i2c_fd, out, 2 + len);
    return (w == (ssize_t)(2 + len)) ? 0 : -1;
}

int GT911_I2C_Read(uint16_t reg, uint8_t *buf, uint16_t len) {
    if (i2c_fd < 0) return -1;

    uint8_t addr[2] = { (reg >> 8), (reg & 0xFF) };
    if (write(i2c_fd, addr, 2) != 2) return -1;

    ssize_t r = read(i2c_fd, buf, len);
    return (r == len) ? 0 : -1;
}

static void msleep(unsigned int ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

void GT911_Platform_ResetSequence(void) {
    gpiochip = gpiod_chip_open_by_name("gpiochip0");
    if (!gpiochip) {
        printf("ERROR: cannot open gpiochip0\n");
        return;
    }

    line_rst = gpiod_chip_get_line(gpiochip, GPIO_RST);
    line_int = gpiod_chip_get_line(gpiochip, GPIO_INT);

    if (!line_rst || !line_int) {
        printf("ERROR: cannot get GPIO lines\n");
        return;
    }

    /* INT = output low */
    gpiod_line_request_output(line_int, "GT911", 0);
    /* RST = output low */
    gpiod_line_request_output(line_rst, "GT911", 0);

    msleep(1);

    /* INT high while RST low */
    gpiod_line_set_value(line_int, 1);
    msleep(1);

    /* Release RST high */
    gpiod_line_set_value(line_rst, 1);
    msleep(5);

    /* INT → input mode */
    gpiod_line_release(line_int);
    gpiod_line_request_input(line_int, "GT911");

    msleep(10);

    printf("GT911 reset (libgpiod) complete.\n");
}

static void read_product_id(char pid[9]) {
    uint8_t buf[4];
    if (GT911_I2C_Read(GT911_PRODUCT_ID, buf, 4) == 0) {
        memcpy(pid, buf, 4);
        pid[4] = 0;
    } else {
        strcpy(pid, "ERR");
    }
}

static uint8_t calc_checksum(uint8_t *buf, int len) {
    uint16_t sum = 0;
    for (int i = 0; i < len; ++i) sum += buf[i];
    return (uint8_t)((~sum) + 1);
}

static GT911_Config_t g_cfg;

int GT911_Init(GT911_Config_t cfg) {
    g_cfg = cfg;

    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) {
        perror("open i2c");
        return -1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, GT911_I2C_ADDR) < 0) {
        perror("ioctl I2C_SLAVE");
        close(i2c_fd);
        i2c_fd = -1;
        return -1;
    }

 
    GT911_Platform_ResetSequence();
   
    char pid[9];
    read_product_id(pid);
    printf("GT911 PID: %s\n", pid);

    uint8_t cfg_buf[GT911_CONFIG_LEN];
    if (GT911_I2C_Read(GT911_CONFIG_START, cfg_buf, GT911_CONFIG_LEN) == 0) {
        
        cfg_buf[GT911_CONFIG_LEN - 2] = calc_checksum(cfg_buf, GT911_CONFIG_LEN - 2);

        
        if (GT911_I2C_Write(GT911_CONFIG_START, cfg_buf, GT911_CONFIG_LEN) == 0) {
            uint8_t cmd0 = 0;
            GT911_I2C_Write(GT911_CMD_REG, &cmd0, 1);
        }
    }

    return 0;
}

int GT911_ReadTouch(TouchCordinate_t coords[5], uint8_t *count) {
    if (!coords || !count) return -1;
    *count = 0;

    uint8_t status = 0;
    if (GT911_I2C_Read(GT911_STATUS_REG, &status, 1) != 0)
        return -1;

    uint8_t points = status & 0x0F;
    if (points == 0) {
        uint8_t zero = 0;
        GT911_I2C_Write(GT911_STATUS_REG, &zero, 1);
        return 0;
    }
    if (points > 5) points = 5;

    uint8_t point_buf[GT911_POINT_REG_LEN * 5];

    if (GT911_I2C_Read(GT911_POINT1_REG, point_buf,
                                GT911_POINT_REG_LEN * points) != 0)
        return -1;

    for (int i = 0; i < points; ++i) {
        uint8_t *p = &point_buf[i * GT911_POINT_REG_LEN];

        uint16_t x = p[1] << 8 | p[0];
        uint16_t y = p[3] << 8 | p[2];
        uint8_t id = p[4];
        uint8_t size = p[5];

        /* Apply transforms */
        uint16_t rx = x, ry = y;
        if (g_cfg.SwitchX2Y) {
            uint16_t t = rx; rx = ry; ry = t;
        }
        if (g_cfg.ReverseX) rx = (g_cfg.X_Resolution > 0) ?
                                (g_cfg.X_Resolution - rx) : rx;
        if (g_cfg.ReverseY) ry = (g_cfg.Y_Resolution > 0) ?
                                (g_cfg.Y_Resolution - ry) : ry;

        coords[i].x = rx;
        coords[i].y = ry;
        coords[i].track_id = id;
        coords[i].size = size;
    }

    *count = points;

    /* Clear status */
    uint8_t zero = 0;
    GT911_I2C_Write(GT911_STATUS_REG, &zero, 1);

    return 0;
}
