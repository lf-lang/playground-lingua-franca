/*************
Copyright (c) 2022, The University of Texas at Dallas. Copyright (c) 2022, The
University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

/**
 * @file scheduler_params.h
 * @author Soroush Bateni <soroush@utdallas.edu>
 * @brief Scheduler parameters.
 *
 * Meant for book-keeping in the threaded schedulers in the reactor C runtime.
 *
 * @copyright Copyright (c) 2022, The University of Texas at Dallas.
 * @copyright Copyright (c) 2022, The University of California at Berkeley.
 */

#ifndef LF_SCHEDULER_PARAMS_H
#define LF_SCHEDULER_PARAMS_H

#ifndef NUMBER_OF_WORKERS // Enable thread-related platform functions
#define NUMBER_OF_WORKERS 1
#endif  // NUMBER_OF_WORKERS

#include "../utils/semaphore.h"
#include "scheduler.h"

extern lf_mutex_t mutex;


/**
 * @brief Paramters used in schedulers of the threaded reactor C runtime.
 *
 * @note Members of this struct are added based on existing schedulers' needs.
 *  These should be expanded to accommodate new schedulers.
 */
typedef struct {
    /**
     * @brief Maximum number of levels for reactions in the program.
     *
     */
    size_t max_reaction_level;

    /**
     * @brief Used by the scheduler to signal the maximum number of worker
     * threads that should be executing work at the same time.
     *
     * Initially, the count is set to 0. Maximum value of count should be
     * `_lf_sched_number_of_workers`.
     *
     * For example, if the scheduler releases the semaphore with a count of 4,
     * no more than 4 worker threads should wake up to process reactions.
     *
     * FIXME: specific comment
     */
    semaphore_t* _lf_sched_semaphore; 

    /**
     * @brief Indicate whether the program should stop
     *
     */
    volatile bool _lf_sched_should_stop;

    /**
     * @brief Hold triggered reactions.
     *
     */
    void* _lf_sched_triggered_reactions;

    /**
     * @brief An array of mutexes.
     *
     * Can be used to avoid race conditions. Schedulers are allowed to
     * initialize as many mutexes as they deem fit.
     */
    lf_mutex_t* _lf_sched_array_of_mutexes;

    /**
     * @brief An array of atomic indexes.
     *
     * Can be used to avoid race conditions. Schedulers are allowed to to use as
     * many indexes as they deem fit.
     */
    volatile int* _lf_sched_indexes;

    /**
     * @brief Hold currently executing reactions.
     */
    void* _lf_sched_executing_reactions;

    /**
     * @brief Hold reactions temporarily.
     */
    void* _lf_sched_transfer_reactions;

    /**
     * @brief Number of workers that this scheduler is managing.
     *
     */
    size_t _lf_sched_number_of_workers;

    /**
     * @brief Number of workers that are idle.
     *
     * Adding to/subtracting from this variable must be done atomically.
     *
     */
    volatile size_t _lf_sched_number_of_idle_workers;

    /**
     * @brief The next level of reactions to execute.
     *
     */
    volatile size_t _lf_sched_next_reaction_level;
} _lf_sched_instance_t;

/**
 * @brief Initialize `instance` using the provided information.
 * 
 * No-op if `instance` is already initialized (i.e., not NULL).
 * This function assumes that mutex is allowed to be recursively locked.
 * 
 * @param instance The `_lf_sched_instance_t` object to initialize.
 * @param number_of_workers  Number of workers in the program.
 * @param params Reference to scheduler parameters in the form of a
 * `sched_params_t`. Can be NULL.
 * @return `true` if initialization was performed. `false` if instance is already
 *  initialized (checked in a thread-safe way).
 */
bool init_sched_instance(
    _lf_sched_instance_t** instance,
    size_t number_of_workers,
    sched_params_t* params) {

    // Check if the instance is already initialized
    lf_mutex_lock(&mutex); // Safeguard against multiple threads calling this 
                           // function.
    if (*instance != NULL) {
        // Already initialized
        lf_mutex_unlock(&mutex);
        return false;
    } else {
        *instance =
            (_lf_sched_instance_t*)calloc(1, sizeof(_lf_sched_instance_t));
    }
    lf_mutex_unlock(&mutex);

    if (params == NULL || params->num_reactions_per_level_size == 0) {
        (*instance)->max_reaction_level = DEFAULT_MAX_REACTION_LEVEL;
    }

    if (params != NULL) {
        if (params->num_reactions_per_level != NULL) {
            (*instance)->max_reaction_level =
                params->num_reactions_per_level_size - 1;
        }
    }

    (*instance)->_lf_sched_semaphore = lf_semaphore_new(0);
    (*instance)->_lf_sched_number_of_workers = number_of_workers;
    (*instance)->_lf_sched_next_reaction_level = 1;

    (*instance)->_lf_sched_should_stop = false;

    return true;
}

#endif // LF_SCHEDULER_PARAMS_H
