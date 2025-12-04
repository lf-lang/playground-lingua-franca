#include "light_utils.h"
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdio.h>

#define BH1750_ADDR 0x23
#define BH1750_MODE 0x10   

int bh1750_init(BH1750* sensor, const char* i2cDevice) {
    sensor->device = (char*)i2cDevice;

    sensor->fd = open(i2cDevice, O_RDWR);
    if (sensor->fd < 0) {
        perror("Failed to open I2C device");
        return -1;
    }

    if (ioctl(sensor->fd, I2C_SLAVE, BH1750_ADDR) < 0) {
        perror("Failed to set I2C address");
        return -1;
    }

    uint8_t cmd = BH1750_MODE;
    if (write(sensor->fd, &cmd, 1) != 1) {
        perror("Failed to write mode");
        return -1;
    }

    usleep(200000);   
    return 0;
}

float bh1750_read_lux(BH1750* sensor) {
    uint8_t data[2];

    if (read(sensor->fd, data, 2) != 2) {
        perror("Failed to read BH1750");
        return -1;
    }

    uint16_t raw = (data[0] << 8) | data[1];
    return raw / 1.2;
}

void bh1750_close(BH1750* sensor) {
    close(sensor->fd);
}
