#include "temp_utils.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

int fd;
uint32_t period;
struct bme280_dev dev;
struct bme280_settings settings;

BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr,
                                   uint8_t *reg_data,
                                   uint32_t len,
                                   void *intf_ptr)
{
    int fd = *(int *)intf_ptr;

    if (write(fd, &reg_addr, 1) != 1)
        return BME280_E_COMM_FAIL;


    if (read(fd, reg_data, len) != (int)len)
        return BME280_E_COMM_FAIL;

    return BME280_OK;
}

BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr,
                                    const uint8_t *reg_data,
                                    uint32_t len,
                                    void *intf_ptr)
{
    int fd = *(int *)intf_ptr;

    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);

    if (write(fd, buf, len + 1) != (int)(len + 1))
        return BME280_E_COMM_FAIL;

    return BME280_OK;
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    usleep(period);
}


int open_i2c(const char *dev, int addr)
{
    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        perror("open");
        exit(1);
    }
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        perror("ioctl");
        exit(1);
    }
    return fd;
}
