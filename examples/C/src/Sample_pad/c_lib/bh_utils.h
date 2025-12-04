#ifndef BH_UTILS_H
#define BH_UTILS_H

#include <stdint.h>

typedef struct {
    int fd;            
    char* device;      
} BH1750;

// Initialize the sensor
int bh1750_init(BH1750* sensor, const char* i2cDevice);

// Read lux value
float bh1750_read_lux(BH1750* sensor);

// Close the sensor
void bh1750_close(BH1750* sensor);

#endif
