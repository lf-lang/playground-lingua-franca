#include "bme_utils.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <stdio.h>


int i2c_fd;
uint32_t period;
struct bme280_dev dev;
struct bme280_settings settings;

BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                   uint32_t len, void *intf_ptr)
{
    if (write(i2c_fd, &reg_addr, 1) != 1)
        return -1;
    if (read(i2c_fd, reg_data, len) != (int)len)
        return -1;
    return 0;
}

BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                    uint32_t len, void *intf_ptr)
{
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);
    if (write(i2c_fd, buf, len + 1) != (int)(len + 1))
        return -1;
    return 0;
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
    usleep(period);
}
