#ifndef GT_UTILS_H
#define GT_UTILS_H

#include <stdint.h>
#include <stdbool.h>


/* ---------------- GT911 I2C ADDRESS ---------------- */
#define GT911_I2C_ADDR 0x14

/* ---------------- MAX TOUCH POINTS ---------------- */
#define GT911_MAX_POINTS 5

/* ---------------- STATUS ENUM ---------------- */
typedef enum {
    GT911_OK = 0,
    GT911_NotResponse = -1
} GT911_Status_t;

/* ---------------- DEVICE STRUCT ---------------- */
typedef struct {
    int fd;                         // I2C file descriptor
    uint8_t device_addr;            // GT911 I2C address

    int num_points;                 // Number of touch points detected
    uint16_t x[GT911_MAX_POINTS];   // X coordinates
    uint16_t y[GT911_MAX_POINTS];   // Y coordinates
} GT911;

/* ---------------- CONFIG STRUCT ---------------- */
typedef struct{
	uint16_t X_Resolution;
	uint16_t Y_Resolution;
	uint8_t Number_Of_Touch_Support;
	bool ReverseX;
	bool ReverseY;
	bool SwitchX2Y;
	bool SoftwareNoiseReduction;

}GT911_Config_t;


/* ---------------- TOUCH POINT STRUCT ---------------- */
typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t track_id;
    uint8_t size;
} TouchCordinate_t;

/* ---------------- PUBLIC API ---------------- */
void gt911_reset_sequence();
int gt911_init(GT911 *ts, const char *i2c_dev);
int gt911_read_touch(GT911 *ts);
void gt911_close(GT911 *ts);

int GT911_Init(GT911_Config_t cfg);
int GT911_ReadTouch(TouchCordinate_t coords[5], uint8_t *count);

#endif
