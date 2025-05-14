/**
 * @file
 * @author Erling Rennemo Jellum
 * @copyright (c) 2024
 * License: <a href="https://github.com/lf-lang/reactor-c/blob/main/LICENSE.md">BSD 2-clause</a>
 * @brief A higher level API to the clock utilities provided by the platform API.
 *
 * This builds on top of the clocking API of the different platforms and ensures:
 * 1. Monotonicity
 * 2. That clock synchronization offsets are applied and removed when necessary.
 */

#ifndef CLOCK_H
#define CLOCK_H

#include "low_level_platform.h"

/**
 * Block the calling thread until wakeup_time is reached or the thread is
 * interrupted by an asynchronous scheduling. This is used by the single-threaded
 * runtime. Before calling the appropriate function in the platform API, the
 * wakeup_time will be translated into the correct timescale by removing any
 * clock synchronization offset.

 * @return 0 on success or -1 if interrupted.
 */
int lf_clock_interruptable_sleep_until_locked(environment_t* env, instant_t wakeup_time);

/**
 * Retrieve the current physical time from the platform API. This adds any clock synchronization offset
 * and guarantees monotonicity. Specifically, each returned value will be at least one nanosecond larger
 * than any previously returned time.
 * @param now A pointer to the location in which to store the result.
 * @return 0 on success, -1 on failure to read the platform clock.
 */
int lf_clock_gettime(instant_t* now);

#if !defined(LF_SINGLE_THREADED)
/**
 * Block the calling thread on the condition variable until it is
 * signaled or until wakeup_time is reached. Before calling the appropriate
 * function in the platform API, the wakeup_time will be translated into the
 * correct timescale by removing any clock synchronization offset.

 * @return 0 on success, LF_TIMEOUT on timeout, platform-specific error
 * otherwise.
 */
int lf_clock_cond_timedwait(lf_cond_t* cond, instant_t wakeup_time);
#endif

#endif
