#ifndef _LED_REMAPPING_H
#define _LED_REMAPPING_H

#include "pico/stdlib.h" // From the Pico-SDK

#include <stdio.h>
#include <stdlib.h>


#define LED_PIN     18
#define NUM_LEDS    256
#define BRIGHTNESS  64
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB


/**
 * @brief Allocates a two-dimensional array mapped to the indices of a serial led array. Allows the serial array to be 
 * accessed in [row][column] format for easier math operations in the animations. 
 * 
 * @param rows The number of rows desired for the remapping: i.e. sqrt(NUM_LEDS)
 * @param cols The number of cols: i.e. also sqrt(NUM_LEDS)
 * 
 */
uint32_t** remap(int rows, int cols);

/**
 * 
 * @brief Frees up the allocated heap memory taken by the led_remap.
 * 
 * @param led_remap The remapped array that was created.
 * @param rows The number of rows in the remapped array.
 * 
 */
void free_remap(uint32_t** led_remap, int rows);

#endif /* _LED_REMAPPING_ */
