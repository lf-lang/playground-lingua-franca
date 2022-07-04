/**
 * @file
 * @author Edward A. Lee
 * @author Soroush Bateni
 * @author Hou Seng (Steven) Wong
 *
 * @section LICENSE
Copyright (c) 2020, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * @section DESCRIPTION
 * Implementation file for tag functions for Lingua Franca programs.
 */

#include "tag.h"
#include "platform.h"

/**
 * Current time in nanoseconds since January 1, 1970
 * This is not in scope for reactors.
 * This should only ever be accessed while holding the mutex lock.
 */
tag_t current_tag = {.time = 0LL, .microstep = 0};

/**
 * Physical time at the start of the execution.
 * This should only ever be accessed while holding the mutex lock.
 */
instant_t physical_start_time = NEVER;

/**
 * Logical time at the start of execution.
 * This should only ever be accessed while holding the mutex lock.
 */
instant_t start_time = NEVER;

/**
 * Global physical clock offset.
 * Initially set according to the RTI's clock in federated
 * programs.
 */
interval_t _lf_time_physical_clock_offset = 0LL;

/**
 * A measure of calculating the drift between the federate's
 * clock and the RTI's clock
 */
interval_t _lf_global_physical_clock_drift = 0LL;

/**
 * A test offset that is applied to the clock.
 * The clock synchronization algorithm must correct for this offset.
 * This offset is especially useful to test clock synchronization on the
 * same machine.
 */
interval_t _lf_time_test_physical_clock_offset = 0LL;

/**
 * Stores the last reported absolute snapshot of the 
 * physical clock.
 */
instant_t _lf_last_reported_physical_time_ns = 0LL;

/**
 * Records the most recent time reported by the physical clock
 * when accessed by get_physical_time(). This will be an epoch time
 * (number of nanoseconds since Jan. 1, 1970), as reported when
 * you call lf_clock_gettime(CLOCK_REALTIME, ...). This differs from
 * _lf_last_reported_physical_time_ns by _lf_time_physical_clock_offset
 * plus any calculated drift adjustement, which are adjustments made
 * by clock synchronization.
 */
instant_t _lf_last_reported_unadjusted_physical_time_ns = NEVER;

/**
 * Return the current tag, a logical time, microstep pair.
 */
tag_t lf_tag() {
    return current_tag;
}

/**
 * Compare two tags. Return -1 if the first is less than
 * the second, 0 if they are equal, and +1 if the first is
 * greater than the second. A tag is greater than another if
 * its time is greater or if its time is equal and its microstep
 * is greater.
 * @param tag1
 * @param tag2
 * @return -1, 0, or 1 depending on the relation.
 */
int lf_tag_compare(tag_t tag1, tag_t tag2) {
    if (tag1.time < tag2.time) {
        return -1;
    } else if (tag1.time > tag2.time) {
        return 1;
    } else if (tag1.microstep < tag2.microstep) {
        return -1;
    } else if (tag1.microstep > tag2.microstep) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * Delay a tag by the specified time interval to realize the "after" keyword.
 * If either the time interval or the time field of the tag is NEVER,
 * return the unmodified tag.
 * If the time interval is 0LL, add one to the microstep, leave
 * the time field alone, and return the result.
 * Otherwise, add the interval to the time field of the tag and reset
 * the microstep to 0.
 * If the sum overflows, saturate the time value at FOREVER.
 *
 * Note that normally it makes no sense to call this with a negative
 * interval (except NEVER), but this is not checked.
 *
 * @param tag The tag to increment.
 * @param interval The time interval.
 */
tag_t _lf_delay_tag(tag_t tag, interval_t interval) {
    if (tag.time == NEVER || interval == NEVER) return tag;
    tag_t result = tag;
    if (interval == 0LL) {
        // Note that unsigned variables will wrap on overflow.
        // This is probably the only reasonable thing to do with overflowing
        // microsteps.
        result.microstep++;
    } else {
        // Note that overflow in C is undefined for signed variables.
        if (FOREVER - interval < result.time) {
            result.time = FOREVER;
        } else {
            result.time += interval;
        }
        result.microstep = 0;
    }
    return result;
}

/**
 * Return the current physical time in nanoseconds since January 1, 1970,
 * adjusted by the global physical time offset.
 */
instant_t _lf_physical_time() {
    // Get the current clock value
    int result = lf_clock_gettime(&_lf_last_reported_unadjusted_physical_time_ns);

    if (result != 0) {
        lf_print_error("Failed to read the physical clock.");
    }
    
    // Adjust the reported clock with the appropriate offsets
    instant_t adjusted_clock_ns = _lf_last_reported_unadjusted_physical_time_ns
            + _lf_time_physical_clock_offset;

    // Apply the test offset
    adjusted_clock_ns += _lf_time_test_physical_clock_offset;

    // if (_lf_global_physical_clock_drift != 0LL
    //         && _lf_last_clock_sync_instant != 0LL) {
    //     // Apply the calculated drift, if appropriate
    //     interval_t drift = (adjusted_clock_ns - _lf_last_clock_sync_instant) *
    //                        _lf_global_physical_clock_drift;
    //     adjusted_clock_ns += drift;
    //     LF_PRINT_DEBUG("Physical time adjusted for clock drift by %lld.", drift);
    // }
    
    // Check if the clock has progressed since the last reported value
    // This ensures that the clock is monotonic
    if (adjusted_clock_ns > _lf_last_reported_physical_time_ns) {
        _lf_last_reported_physical_time_ns = adjusted_clock_ns;
    }
    
    LF_PRINT_DEBUG("Physical time: %lld. Elapsed: %lld. Offset: %lld",
            _lf_last_reported_physical_time_ns,
            _lf_last_reported_physical_time_ns - start_time,
            _lf_time_physical_clock_offset + _lf_time_test_physical_clock_offset);

    return _lf_last_reported_physical_time_ns;
}

/**
 * An enum for specifying the desired tag when calling "lf_time"
 */
typedef enum _lf_time_type {
    LF_LOGICAL,
    LF_PHYSICAL,
    LF_ELAPSED_LOGICAL,
    LF_ELAPSED_PHYSICAL,
    LF_START
} _lf_time_type;


/**
 * Get the time specified by "type".
 * 
 * Example use cases:
 * - Getting the starting time:
 * _lf_time(LF_START)
 * 
 * - Getting the elapsed physical time:
 * _lf_time(LF_ELAPSED_PHYSICAL)
 * 
 * - Getting the logical time
 * _lf_time(LF_LOGICAL)
 * 
 * @param type A field in an enum specifying the time type. 
 *             See enum "lf_time_type" above.
 * @return The desired time
 */
instant_t _lf_time(_lf_time_type type) {
    switch (type)
    {
    case LF_LOGICAL:
        return current_tag.time;
    case LF_PHYSICAL:
        return _lf_physical_time();
    case LF_ELAPSED_LOGICAL:
        return current_tag.time - start_time;
    case LF_ELAPSED_PHYSICAL:
        return _lf_physical_time() - physical_start_time;
    case LF_START:
        return start_time;
    default:
        return NEVER;
    }
}

/**
 * Return the current logical time in nanoseconds since January 1, 1970.
 */
instant_t lf_time_logical(void) {
    return _lf_time(LF_LOGICAL);
}

/**
 * Return the elapsed logical time in nanoseconds since the start of execution.
 */
interval_t lf_time_logical_elapsed(void) {
    return _lf_time(LF_ELAPSED_LOGICAL);
}


/**
 * Return the current physical time in nanoseconds since January 1, 1970,
 * adjusted by the global physical time offset.
 */
instant_t lf_time_physical(void) {
    return _lf_time(LF_PHYSICAL);
}

/**
 * Return the elapsed physical time in nanoseconds.
 * This is the time returned by get_physical_time() minus the
 * physical start time as measured by get_physical_time() when
 * the program was started.
 */
instant_t lf_time_physical_elapsed(void) {
    return _lf_time(LF_ELAPSED_PHYSICAL);
}


/**
 * Return the physical time of the start of execution in nanoseconds. * 
 * On many platforms, this is the number of nanoseconds
 * since January 1, 1970, but it is actually platform dependent. * 
 * @return A time instant.
 */
instant_t lf_time_start(void) {
    return _lf_time(LF_START);
}

/**
 * Set a fixed offset to the physical clock.
 * After calling this, the value returned by get_physical_time()
 * and get_elpased_physical_time() will have this specified offset
 * added to what it would have returned before the call.
 */
void lf_set_physical_clock_offset(interval_t offset) {
    _lf_time_test_physical_clock_offset += offset;
}

/**
 * Store into the specified buffer a string giving a human-readable
 * rendition of the specified time. The buffer must have length at least
 * equal to LF_TIME_BUFFER_LENGTH. The format is:
 * ```
 *    x weeks, x days, x hours, x minutes, x seconds, x unit
 * ```
 * where each `x` is a string of numbers with commas inserted if needed
 * every three numbers and `unit` is nanoseconds, microseconds, or
 * milliseconds.
 * @param buffer The buffer into which to write the string.
 * @param time The time to write.
 * @return The number of characters written (not counting the null terminator).
 */
size_t lf_readable_time(char* buffer, instant_t time) {
    char* original_buffer = buffer;
    bool lead = false; // Set to true when first clause has been printed.
    if (time > WEEKS(1)) {
        lead = true;
        size_t printed = lf_comma_separated_time(buffer, time / WEEKS(1));
        time = time % WEEKS(1);
        buffer += printed;
        sprintf(buffer, " weeks");
        buffer += 6;
    }
    if (time > DAYS(1)) {
        if (lead == true) {
            sprintf(buffer, ", ");
            buffer += 2;
        }
        lead = true;
        size_t printed = lf_comma_separated_time(buffer, time / DAYS(1));
        time = time % DAYS(1);
        buffer += printed;
        sprintf(buffer, " days");
        buffer += 5;
    }
    if (time > HOURS(1)) {
        if (lead == true) {
            sprintf(buffer, ", ");
            buffer += 2;
        }
        lead = true;
        size_t printed = lf_comma_separated_time(buffer, time / HOURS(1));
        time = time % HOURS(1);
        buffer += printed;
        sprintf(buffer, " hours");
        buffer += 6;
    }
    if (time > MINUTES(1)) {
        if (lead == true) {
            sprintf(buffer, ", ");
            buffer += 2;
        }
        lead = true;
        size_t printed = lf_comma_separated_time(buffer, time / MINUTES(1));
        time = time % MINUTES(1);
        buffer += printed;
        sprintf(buffer, " minutes");
        buffer += 8;
    }
    if (time > SECONDS(1)) {
        if (lead == true) {
            sprintf(buffer, ", ");
            buffer += 2;
        }
        lead = true;
        size_t printed = lf_comma_separated_time(buffer, time / SECONDS(1));
        time = time % SECONDS(1);
        buffer += printed;
        sprintf(buffer, " seconds");
        buffer += 8;
    }
    if (time > (instant_t)0) {
        if (lead == true) {
            sprintf(buffer, ", ");
            buffer += 2;
        }
        const char* units = "nanoseconds";
        if (time % MSEC(1) == (instant_t) 0) {
            units = "milliseconds";
            time = time % MSEC(1);
        } else if (time % USEC(1) == (instant_t) 0) {
            units = "microseconds";
            time = time % USEC(1);
        }
        size_t printed = lf_comma_separated_time(buffer, time);
        buffer += printed;
        sprintf(buffer, " %s", units);
        buffer += strlen(units) + 1;
    } else {
        sprintf(buffer, "0");
    }
    return (buffer - original_buffer);
}

/**
 * Print a non-negative time value in nanoseconds with commas separating thousands
 * into the specified buffer. Ideally, this would use the locale to
 * use periods if appropriate, but I haven't found a sufficiently portable
 * way to do that.
 * @param buffer A buffer long enough to contain a string like "-9,223,372,036,854,775,807".
 * @param time A time value.
 * @return The number of characters written into the buffer (not including
 *  the null terminator).
 */
size_t lf_comma_separated_time(char* buffer, instant_t time) {
    size_t result = 0; // The number of characters printed.
    // If the number is zero, print it and return.
    if (time == (instant_t)0) {
        sprintf(buffer, "0");
        return 1;
    }
    // If the number is negative, print a minus sign.
    if (time < (instant_t)0) {
        sprintf(buffer, "-");
        buffer++;
        result++;
    }
    int count = 0;
    // Assume the time value is no larger than 64 bits.
    instant_t clauses[7];
    while (time > (instant_t)0) {
        clauses[count++] = time;
        time = time/1000;
    }
    // Highest order clause should not be filled with zeros.
    instant_t to_print = clauses[--count] % 1000;
    sprintf(buffer, "%lld", (long long)to_print);
    if (to_print >= 100LL) {
        buffer += 3;
        result += 3;
    } else if (to_print >= 10LL) {
        buffer += 2;
        result += 2;
    } else {
        buffer += 1;
        result += 1;
    }
    while (count-- > 0) {
        to_print = clauses[count] % 1000LL;
        sprintf(buffer, ",%03lld", (long long)to_print);
        buffer += 4;
        result += 4;
    }
    return result;
}

/**
 * For C++ compatibility, take a volatile tag_t and return a non-volatile
 * variant.
 */
#ifdef __cplusplus
tag_t _lf_convert_volatile_tag_to_nonvolatile(tag_t volatile const& vtag) {
    tag_t non_volatile_tag;
    non_volatile_tag.time = vtag.time;
    non_volatile_tag.microstep = vtag.microstep;
    return non_volatile_tag;
}
#else
/**
 * @note This is an undefined behavior in C and should
 *  be used with utmost caution. See Section 6.7.2 of the C99 standard.
 */
tag_t _lf_convert_volatile_tag_to_nonvolatile(tag_t volatile vtag) {
    return vtag;
}
#endif
