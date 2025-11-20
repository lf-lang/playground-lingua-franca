#ifndef GT_UTILS_H
#define GT_UTILS_H

#include <stdint.h>

#define GT911_ADDR 0x5D   
#define GT911_MAX_POINTS 5

typedef struct {
    int fd;                   
    uint8_t device_addr;       

    uint8_t num_points;
    uint16_t x[GT911_MAX_POINTS];
    uint16_t y[GT911_MAX_POINTS];
} GT911;

int gt911_init(GT911 *ts, const char *i2c_dev);
int gt911_read_touch(GT911 *ts);
void gt911_close(GT911 *ts);

#endif
