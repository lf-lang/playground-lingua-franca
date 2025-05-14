/**
 * @file
 * @author Soroush Bateni
 * @author Edward A. Lee
 * @copyright (c) 2022-2024, The University of Texas at Dallas and The University of California at Berkeley.
 * License: <a href="https://github.com/lf-lang/reactor-c/blob/main/LICENSE.md">BSD 2-clause</a>
 * @brief Scheduler API for the threaded C runtime.
 *
 * A scheduler for the threaded runtime of reactor-c should provide an
 * implementation for functions that are defined in this header file.
 */

#ifndef LF_SCHEDULER_H
#define LF_SCHEDULER_H

#include "lf_types.h"
#include "scheduler_instance.h"

/**
 * @brief Initialize the scheduler.
 *
 * This has to be called before other functions of the scheduler can be used.
 * If the scheduler is already initialized, this will be a no-op.
 *
 * @param env The environment in which we should initialize the scheduler
 * @param number_of_workers Indicate how many workers this scheduler will be
 *  managing.
 * @param option Pointer to a `sched_params_t` struct containing additional
 *  scheduler parameters. Can be NULL.
 */
void lf_sched_init(environment_t* env, size_t number_of_workers, sched_params_t* parameters);

/**
 * @brief Free the memory used by the scheduler.
 * @param scheduler The scheduler
 *
 * This must be called when the scheduler is no longer needed.
 */
void lf_sched_free(lf_scheduler_t* scheduler);

/**
 * @brief Ask the scheduler for one more reaction.
 *
 * This function blocks until it can return a ready reaction for worker thread
 * 'worker_number' or it is time for the worker thread to stop and exit (where a
 * NULL value would be returned).
 * This function assumes that the environment mutex is not locked.
 *
 * @param scheduler The scheduler
 * @param worker_number For the calling worker thread.
 * @return reaction_t* A reaction for the worker to execute. NULL if the calling
 * worker thread should exit.
 */
reaction_t* lf_sched_get_ready_reaction(lf_scheduler_t* scheduler, int worker_number);

/**
 * @brief Inform the scheduler that worker thread 'worker_number' is done
 * executing the 'done_reaction'.
 *
 * @param scheduler The scheduler
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
 * @param scheduler The scheduler
 * @param reaction The reaction to trigger at the current tag.
 * @param worker_number The ID of the worker that is making this call. 0 should be
 *  used if there is only one worker (e.g., when the program is using the
 *  single-threaded C runtime). -1 is used for an anonymous call in a context where a
 *  worker number does not make sense (e.g., the caller is not a worker thread).
 *
 */
void lf_scheduler_trigger_reaction(lf_scheduler_t* scheduler, reaction_t* reaction, int worker_number);

#endif // LF_SCHEDULER_H
