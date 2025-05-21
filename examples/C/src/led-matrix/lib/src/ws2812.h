/**
 * MIT License
 * 
 * Copyright (c) 2025 Benjamin Gunnels
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/** 
 * @file ws2812.h
 * 
 * @brief Constant struct and function prototypes for the ws2812 driver.
 * 
 * This contains the struct initializations and functions for the ws2812 driver 
 * for use with the Raspberry Pi Pico microcontroller board. Uses PIO on the board to 
 * control the timing protocol necessary for the ws2812 LED.
 *  
 * Credit to Kevin Thomas @ My Techno Talent 04/11/2023 for inspiration on this code.
 * 
 * @author Benjamin Gunnels
 * @date   05/20/2025
*/

#ifndef _WS2812_H
#define _WS2812_H

#include "pico/stdlib.h"
#include "pins.h"
#include "led_remapping.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

#include <math.h>


/**
 * @brief Structure for WS2812 LED configuration.
 *
 * This structure defines the configuration for controlling WS2812 LEDs.
 */
typedef struct {
    uint8_t pin;
} ws2812_config_t;

/**
 * @brief External declaration of WS2812 LED configuration.
 *
 * This declares an external variable `ws2812_config` of type `ws2812_config_t`,
 * which holds the configuration settings for controlling WS2812 LEDs.
 * The actual definition of this variable is expected to be provided in another 
 * source file.
 */
extern ws2812_config_t ws2812_config;

/**
 * @brief Initialize a PIO state machine to control WS2812 (NeoPixel) LEDs.
 *
 * This function configures one of the RP2040's PIO state machines to run a
 * WS2812-compatible PIO program at a specified frequency. It sets up pin control,
 * data format, and timing, preparing the state machine to transmit LED color data.
 *
 * @param pio     The PIO instance to use (e.g., pio0 or pio1).
 * @param sm      The state machine number (0–3) within the PIO block.
 * @param offset  The program offset in PIO instruction memory (from pio_add_program()).
 * @param pin     The GPIO pin connected to the WS2812 data line.
 * @param freq    The frequency in Hz to run the WS2812 protocol (typically 800,000 Hz).
 * @param rgbw    If true, the state machine is configured to send 32-bit RGBW values.
 *                If false, it sends 24-bit RGB (GRB) values.
 */
void ws2812_program_init(PIO pio, uint sm, uint offset, uint8_t pin, float freq, bool rgbw);


/**
 * @brief Sends an array of colors to the WS2812 Led device.
 *
 * This function sends an array of color data to a WS2812 LED strip using a specified GPIO pin.
 * Makes recurrent calls to ws2812_send_bit to transmit each bit in the color value which is given as a 32 bit integer.
 * 
 * 
 * @param pio            PIO object 
 * @param sm             State machine number used for the PIO block.
 *                       containing the GPIO pin used for WS2812 LED control.
 * @param  colors        An array of size num_leds containing color values corresponding to each pixel in the array.
 *                       The array is single dimensional.  
 * @param  num_leds      The total number of leds in the led array.
 * @return None
 */
// void send_colors(ws2812_config_t *ws2812_config, uint32_t* colors, uint32_t num_leds);
void send_colors(PIO pio, int sm, uint32_t* colors, uint32_t num_leds);


/**
 * @brief Returns a 32 bit integer from an explicit RGB color value.
 *
 * This function makes it convenient to turn explicit R, G, and B values into a 32 bit integer.
 *
 * @param  r    Red color value.
 * @param  g    Green color value.
 * @param  b    Blue color value
 * @param  brightness   Float specified to adjust brightness
 * @return None
 */
uint32_t rgb_color(uint8_t r, uint8_t g, uint8_t b, float brightness);


/**
 *  
 * 
 * Color Animations
 * 
 */

/**
 * @brief Plays a rainbow pattern animation. Scrolls ROYGBIV across the array. Takes an incrementer to move the animation.
 * 
 * @param led_strip One dimensional array used to write values to the LED strip.
 * @param remapping Allows row/column format to be translated to the one dimensional LED array.
 * @param i Counter referring to the current frame. This is used for animation purposes. 
 *             By modding the pointer different patterns can be made per frame.
 * @param brightness Scalar to adjust the brightness of the pixel value.
 * @param rows Number of rows in the LED strip.
 * @param cols Number of columns in the LED strip.
 */
void rainbow(uint32_t *led_strip, uint32_t** remapping, int i, float brightness, int rows, int cols);


/**
 * @brief Uses waves to resemble an aurora borealis.
 * 
 * @param led_strip One dimensional array used to write values to the LED strip.
 * @param remapping Allows row/column format to be translated to the one dimensional LED array.
 * @param i Counter referring to the current frame. This is used for animation purposes. 
 *             By modding the pointer different patterns can be made per frame.
 * @param brightness Scalar to adjust the brightness of the pixel value.
 * @param rows Number of rows in the LED strip.
 * @param cols Number of columns in the LED strip.
 */
void aurora(uint32_t *led_strip, uint32_t** remapping, int i, float brightness, int rows, int cols);

/**
 * @brief Flashes a heart beat animation. The heart grows quickly twice, then shrinks to its resting size for a pause.
 * 
 * @param led_strip One dimensional array used to write values to the LED strip.
 * @param remapping Allows row/column format to be translated to the one dimensional LED array.
 * @param i Counter referring to the current frame. This is used for animation purposes. 
 *             By modding the pointer different patterns can be made per frame.
 * @param brightness Scalar to adjust the brightness of the pixel value.
 * @param rows Number of rows in the LED strip.
 * @param cols Number of columns in the LED strip.
 */
void beating_heart(uint32_t *led_strip, uint32_t** remapping, int i, float brightness, int rows, int cols);

/**
 * @brief Flashes alternate squares of red and green, in a chessboard pattern. 
 * 
 * @param led_strip One dimensional array used to write values to the LED strip.
 * @param remapping Allows row/column format to be translated to the one dimensional LED array.
 * @param i Counter referring to the current frame. This is used for animation purposes. 
 *             By modding the pointer different patterns can be made per frame.
 * @param brightness Scalar to adjust the brightness of the pixel value.
 * @param rows Number of rows in the LED strip.
 * @param cols Number of columns in the LED strip.
 */
void chessboard(uint32_t *led_strip, uint32_t** remapping, int i, float brightness, int rows, int cols);

/**
 * @brief Displays the American Flag.
 * 
 * @param led_strip One dimensional array used to write values to the LED strip.
 * @param remapping Allows row/column format to be translated to the one dimensional LED array.
 * @param i Counter referring to the current frame. This is used for animation purposes. 
 *             By modding the pointer different patterns can be made per frame.
 * @param brightness Scalar to adjust the brightness of the pixel value.
 * @param rows Number of rows in the LED strip.
 * @param cols Number of columns in the LED strip.
 */
void old_glory(uint32_t *led_strip, uint32_t** remapping, int i, float brightness, int rows, int cols);
#endif /* _WS2812_H */
