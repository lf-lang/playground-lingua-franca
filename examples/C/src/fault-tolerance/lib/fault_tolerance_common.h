#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/**
 * @brief Uniformaly generates a pseudo-random double between min (inclusive) and max (inclusive).
 *
 * @param min The minimum value of the desired range.
 * @param max The maximum value of the desired range.
 * @return A pseudo-random double value within the specified range.
 */
float rand_double_range(float min, float max) {
    if (min > max) {
        // Handle error: min should not be greater than max
        return 0.0f; // Or handle as appropriate
    }
    // Generate a random double between 0 and 1
    float scale = (float)rand() / (float)RAND_MAX;
    // Scale the value to the desired range and add the minimum offset
    return min + scale * (max - min);
}
