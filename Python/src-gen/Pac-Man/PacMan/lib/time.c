#include "../include/ctarget/time.h"

/**
 * @deprecated version of "lf_time_logical"
 */
instant_t get_logical_time() { return lf_time_logical(); }

/**
 * @deprecated version of "lf_time_logical_elapsed"
 */
interval_t get_elapsed_logical_time() { return lf_time_logical_elapsed(); }

/**
 * @deprecated version of "lf_time_physical"
 */
instant_t get_physical_time() { return lf_time_physical(); }


/**
 * @deprecated version of "lf_time_physical_elapsed"
 */
instant_t get_elapsed_physical_time() { return lf_time_physical_elapsed(); }

/**
 * @deprecated version of "lf_time_start"
 */
instant_t get_start_time() { return lf_time_start(); }


/**
 * @deprecated version of 'lf_set_physical_clock_offset'
 */
void set_physical_clock_offset(interval_t offset) {
    lf_set_physical_clock_offset(offset);
}
