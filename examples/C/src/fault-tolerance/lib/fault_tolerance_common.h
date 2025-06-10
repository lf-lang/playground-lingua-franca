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

/**
 * @brief Implements busy wait for the given milliseconds in a floating point number.
 *
 * @param milliseconds Time for busy wait given in milliseconds.
 */
void busy_wait(float milliseconds) {
    struct timespec start, current;
    
    clock_gettime(CLOCK_MONOTONIC, &start);

    unsigned long long elapsed;
    unsigned long long wait_ns = (unsigned long long)(milliseconds * 1000000.0f); // Convert milliseconds to nanoseconds

    do {
        clock_gettime(CLOCK_MONOTONIC, &current);
        elapsed = (current.tv_sec - start.tv_sec) * 1000000000ULL;
        elapsed += current.tv_nsec - start.tv_nsec;
    } while (elapsed < wait_ns);
}