/*************
Copyright (c) 2022, The University of Texas at Dallas.
Copyright (c) 2022, The University of California at Berkeley.

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
***************/

/**
 * @file scheduler.h
 * @author Soroush Bateni <soroush@utdallas.edu>
 * @brief Scheduler API for the threaded C runtime.
 * 
 * A scheduler for the threaded runtime of reactor-c should provide an
 * implementation for functions that are defined in this header file.
 * 
 * @copyright Copyright (c) 2022, The University of Texas at Dallas.
 * @copyright Copyright (c) 2022, The University of California at Berkeley.
 */

#ifndef LF_SCHEDULER_H
#define LF_SCHEDULER_H

#include "../reactor.h"

/**
 * @brief Default value that is assumed to be the maximum reaction level in the
 *  program. 
 *
 * Can be overriden by passing the appropriate `parameters` argument to
 * `lf_sched_init`.
 */
#define DEFAULT_MAX_REACTION_LEVEL 100

/**
 * @brief Struct representing the most common scheduler parameters.
 *
 * @param num_reactions_per_level Optional. Default: NULL. An array of
 *  non-negative integers, where each element represents a reaction level
 *  (corresponding to the index), and the value of the element represents the
 *  maximum number of reactions in the program for that level. For example,
 *  num_reactions_per_level = { 2, 3 } indicates that there will be a maximum of
 *  2 reactions in the program with a level of 0, and a maximum of 3 reactions
 *  in the program with a level of 1. Can be NULL.
 * @param num_reactions_per_level_size Optional. The size of the
 * `num_reactions_per_level` array if it is not NULL. If set, it should be the
 * maximum level over all reactions in the program plus 1. If not set,
 * `DEFAULT_MAX_REACTION_LEVEL` will be used.
 */
typedef struct {
    size_t* num_reactions_per_level;
    size_t num_reactions_per_level_size;
} sched_params_t;

/**
 * @brief Initialize the scheduler.
 *
 * This has to be called before other functions of the scheduler can be used.
 * If the scheduler is already initialized, this will be a no-op.
 *
 * @param number_of_workers Indicate how many workers this scheduler will be
 *  managing.
 * @param option Pointer to a `sched_params_t` struct containing additional
 *  scheduler parameters. Can be NULL.
 */
void lf_sched_init(
    size_t number_of_workers, 
    sched_params_t* parameters
);

/**
 * @brief Free the memory used by the scheduler.
 * 
 * This must be called when the scheduler is no longer needed.
 */
void lf_sched_free();

/**
 * @brief Ask the scheduler for one more reaction.
 * 
 * This function blocks until it can return a ready reaction for worker thread
 * 'worker_number' or it is time for the worker thread to stop and exit (where a
 * NULL value would be returned).
 * 
 * @param worker_number For the calling worker thread.
 * @return reaction_t* A reaction for the worker to execute. NULL if the calling
 * worker thread should exit.
 */
reaction_t* lf_sched_get_ready_reaction(int worker_number);

/**
 * @brief Inform the scheduler that worker thread 'worker_number' is done
 * executing the 'done_reaction'.
 * 
 * @param worker_number The worker number for the worker thread that has
 * finished executing 'done_reaction'.
 * @param done_reaction The reaction that is done.
 */
void lf_sched_done_with_reaction(size_t worker_number, reaction_t* done_reaction);


/**
 * @brief Inform the scheduler that worker thread 'worker_number' would like to
 * trigger 'reaction' at the current tag.
 * 
 * If a worker number is not available (e.g., this function is not called by a
 * worker thread), -1 should be passed as the 'worker_number'.
 * 
 * The scheduler will ensure that the same reaction is not triggered twice in
 * the same tag.
 * 
 * @param reaction The reaction to trigger at the current tag.
 * @param worker_number The ID of the worker that is making this call. 0 should be
 *  used if there is only one worker (e.g., when the program is using the
 *  unthreaded C runtime). -1 is used for an anonymous call in a context where a
 *  worker number does not make sense (e.g., the caller is not a worker thread).
 * 
 */
void lf_sched_trigger_reaction(reaction_t* reaction, int worker_number);

#endif // LF_SCHEDULER_H
