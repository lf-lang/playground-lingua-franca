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

float get_wcet_factor() {
    // Generate a random float between 0.8 and 1.0
    return rand_double_range(0.8f, 1.0f);
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

int random_sleep(int failure_threashold) {
    float factor = get_wcet_factor();

    const static int SUCCESS_SEG_WCET_MSEC = 10;
    const static int FAILED_SEG_PENALTY_MSEC = 1;

    if (rand() < failure_threashold) {
        // Failed case.
        busy_wait(SUCCESS_SEG_WCET_MSEC * factor + FAILED_SEG_PENALTY_MSEC);
        return 1;
    } else {
        // Success case.
        busy_wait(SUCCESS_SEG_WCET_MSEC * factor);
        return 0;
    }
}

int task1_seg1(int failure_threashold) {
    return random_sleep(failure_threashold);
}

int task1_seg2(int failure_threashold) {
    return random_sleep(failure_threashold);
}

int task1_seg3(int failure_threashold) {
    return random_sleep(failure_threashold);
}
