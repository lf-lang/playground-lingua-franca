#ifndef CTARGET_TIME
#define CTARGET_TIME

#include "../core/reactor.h"

/**
 * @deprecated version of "lf_time_logical"
 */
DEPRECATED(instant_t get_logical_time(void));

/**
 * @deprecated version of "lf_time_logical_elapsed"
 */
DEPRECATED(interval_t get_elapsed_logical_time(void));

/**
 * @deprecated version of "lf_time_physical"
 */
DEPRECATED(instant_t get_physical_time(void));

/**
 * @deprecated version of "lf_time_physical_elapsed"
 */
DEPRECATED(instant_t get_elapsed_physical_time(void));

/**
 * @deprecated version of "lf_time_start"
 */
DEPRECATED(instant_t get_start_time(void));

/**
 * @deprecated version of 'lf_set_physical_clock_offset'
 */
DEPRECATED(void set_physical_clock_offset(interval_t offset));


#endif // CTARGET_TIME
