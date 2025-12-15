#ifndef TEMP_UTILS_H
#define TEMP_UTILS_H

#include "bme280.h"


#define I2C_DEV_PATH "/dev/i2c-1"
#define BME280_I2C_ADDR BME280_I2C_ADDR_SEC

extern int fd;
extern uint32_t period;
extern struct bme280_dev dev;
extern struct bme280_settings settings;

BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr,
                                   uint8_t *reg_data,
                                   uint32_t len,
                                   void *intf_ptr);


BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr,
                                    const uint8_t *reg_data,
                                    uint32_t len,
                                    void *intf_ptr);


void user_delay_us(uint32_t period, void *intf_ptr);


int open_i2c(const char *dev, int addr);

#endif
