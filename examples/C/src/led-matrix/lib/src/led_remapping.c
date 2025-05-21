#include <led_remapping.h>

/*
    Remaps the single-dimension LEDs array to a 2D remapped array.
    Provides an abstraction allowing the programmer to use a 2-dimensional mapping for simplicity.
    It counts the current LED using the counter. If the row is even, it maps the row and column to that LED directly.
    If the row is odd, it maps the LEDs backward for that row.
    This accounts for the LED board's serpentine pattern, whereby alternating rows are connected left-to-right, then right-to-left. 

    returns: A dynamically allocated 2D array of CRGB pointers (CRGB**)
*/

// Function to create a 2D mapping (snake pattern) to the LED strip
uint32_t** remap(int rows, int cols) {
    // Allocate the 2D array dynamically (pointer to pointer)
    uint32_t** led_remap = (uint32_t**)malloc(rows * sizeof(uint32_t*));
    for (int i = 0; i < rows; i++) {
        led_remap[i] = (uint32_t*)malloc(cols * sizeof(uint32_t));
    }

    int counter = 0; // Counts the current LED in question serially

    // Create the snake pattern mapping
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            if (row % 2 == 0) {
                // Even rows - left to right
                led_remap[row][col] = counter;
            } else {
                // Odd rows - right to left (snake pattern)
                led_remap[row][cols - 1 - col] = counter;
            }
            counter++;
        }
    }

    return led_remap;
}

// Function to free the dynamically allocated remap
void free_remap(uint32_t** remap, int rows) {
    for (int i = 0; i < rows; i++) {
        free(remap[i]);
    }
    free(remap);
}