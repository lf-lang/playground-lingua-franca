/* Global Earliest Deadline First (GEDF) non-preemptive scheduler for the
threaded runtime of the C target of Lingua Franca. */

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
 * Global Earliest Deadline First (GEDF) non-preemptive scheduler for the
 * threaded runtime of the C target of Lingua Franca.
 *
 * @author{Soroush Bateni <soroush@utdallas.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */

#ifndef NUMBER_OF_WORKERS
#define NUMBER_OF_WORKERS 1
#endif  // NUMBER_OF_WORKERS

#include <assert.h>

#include "../platform.h"
#include "../utils/pqueue_support.h"
#include "../utils/semaphore.h"
#include "scheduler.h"
#include "scheduler_instance.h"
#include "scheduler_sync_tag_advance.c"

/////////////////// External Variables /////////////////////////
extern lf_mutex_t mutex;

/////////////////// Scheduler Variables and Structs /////////////////////////
_lf_sched_instance_t* _lf_sched_instance;

/////////////////// Scheduler Private API /////////////////////////
/**
 * @brief Insert 'reaction' into _lf_sched_instance->_lf_sched_triggered_reactions
 * at the appropriate level.
 *
 * @param reaction The reaction to insert.
 */
static inline void _lf_sched_insert_reaction(reaction_t* reaction) {
    size_t reaction_level = LEVEL(reaction->index);
    LF_PRINT_DEBUG("Scheduler: Trying to lock the mutex for level %d.",
                reaction_level);
    lf_mutex_lock(
        &_lf_sched_instance->_lf_sched_array_of_mutexes[reaction_level]);
    LF_PRINT_DEBUG("Scheduler: Locked the mutex for level %d.", reaction_level);
    pqueue_insert(((pqueue_t**)_lf_sched_instance
                       ->_lf_sched_triggered_reactions)[reaction_level],
                  (void*)reaction);
    lf_mutex_unlock(
        &_lf_sched_instance->_lf_sched_array_of_mutexes[reaction_level]);
}

/**
 * @brief Distribute any reaction that is ready to execute to idle worker
 * thread(s).
 *
 * @return Number of reactions that were successfully distributed to worker
 * threads.
 */
int _lf_sched_distribute_ready_reactions() {
    pqueue_t* tmp_queue = NULL;
    // Note: All the threads are idle, which means that they are done inserting
    // reactions. Therefore, the reaction queues can be accessed without locking
    // a mutex.
    for (; _lf_sched_instance->_lf_sched_next_reaction_level <=
           _lf_sched_instance->max_reaction_level;
         _lf_sched_instance->_lf_sched_next_reaction_level++) {
        tmp_queue = ((pqueue_t**)_lf_sched_instance->_lf_sched_triggered_reactions)
                        [_lf_sched_instance->_lf_sched_next_reaction_level];
        size_t reactions_to_execute = pqueue_size(tmp_queue);
        if (reactions_to_execute) {
            _lf_sched_instance->_lf_sched_executing_reactions = tmp_queue;
            _lf_sched_instance->_lf_sched_next_reaction_level++;
            return reactions_to_execute;
        }
    }

    return 0;
}

/**
 * @brief If there is work to be done, notify workers individually.
 *
 * This assumes that the caller is not holding any thread mutexes.
 */
void _lf_sched_notify_workers() {
    // Note: All threads are idle. Therefore, there is no need to lock the mutex
    // while accessing the executing queue (which is pointing to one of the
    // reaction queues).
    size_t workers_to_awaken =
        MIN(_lf_sched_instance->_lf_sched_number_of_idle_workers,
            pqueue_size((pqueue_t*)_lf_sched_instance->_lf_sched_executing_reactions));
    LF_PRINT_DEBUG("Scheduler: Notifying %d workers.", workers_to_awaken);
    _lf_sched_instance->_lf_sched_number_of_idle_workers -= workers_to_awaken;
    LF_PRINT_DEBUG("Scheduler: New number of idle workers: %u.",
                _lf_sched_instance->_lf_sched_number_of_idle_workers);
    if (workers_to_awaken > 1) {
        // Notify all the workers except the worker thread that has called this
        // function.
        lf_semaphore_release(_lf_sched_instance->_lf_sched_semaphore,
                             (workers_to_awaken - 1));
    }
}

/**
 * @brief Signal all worker threads that it is time to stop.
 *
 */
void _lf_sched_signal_stop() {
    _lf_sched_instance->_lf_sched_should_stop = true;
    lf_semaphore_release(_lf_sched_instance->_lf_sched_semaphore,
                         (_lf_sched_instance->_lf_sched_number_of_workers - 1));
}

/**
 * @brief Advance tag or distribute reactions to worker threads.
 *
 * Advance tag if there are no reactions on the reaction queue. If
 * there are such reactions, distribute them to worker threads.
 *
 * This function assumes the caller does not hold the 'mutex' lock.
 */
void _lf_sched_try_advance_tag_and_distribute() {
    // Executing queue must be empty when this is called.
    assert(pqueue_size((pqueue_t*)_lf_sched_instance->_lf_sched_executing_reactions) == 0);

    // Loop until it's time to stop or work has been distributed
    while (true) {
        if (_lf_sched_instance->_lf_sched_next_reaction_level ==
            (_lf_sched_instance->max_reaction_level + 1)) {
            _lf_sched_instance->_lf_sched_next_reaction_level = 0;
            lf_mutex_lock(&mutex);
            // Nothing more happening at this tag.
            LF_PRINT_DEBUG("Scheduler: Advancing tag.");
            // This worker thread will take charge of advancing tag.
            if (_lf_sched_advance_tag_locked()) {
                LF_PRINT_DEBUG("Scheduler: Reached stop tag.");
                _lf_sched_signal_stop();
                lf_mutex_unlock(&mutex);
                break;
            }
            lf_mutex_unlock(&mutex);
        }

        if (_lf_sched_distribute_ready_reactions() > 0) {
            _lf_sched_notify_workers();
            break;
        }
    }
}

/**
 * @brief Wait until the scheduler assigns work.
 *
 * If the calling worker thread is the last to become idle, it will call on the
 * scheduler to distribute work. Otherwise, it will wait on
 * '_lf_sched_instance->_lf_sched_semaphore'.
 *
 * @param worker_number The worker number of the worker thread asking for work
 * to be assigned to it.
 */
void _lf_sched_wait_for_work(size_t worker_number) {
    // Increment the number of idle workers by 1 and check if this is the last
    // worker thread to become idle.
    if (lf_atomic_add_fetch(&_lf_sched_instance->_lf_sched_number_of_idle_workers,
                            1) ==
        _lf_sched_instance->_lf_sched_number_of_workers) {
        // Last thread to go idle
        LF_PRINT_DEBUG("Scheduler: Worker %d is the last idle thread.",
                    worker_number);
        // Call on the scheduler to distribute work or advance tag.
        _lf_sched_try_advance_tag_and_distribute();
    } else {
        // Not the last thread to become idle.
        // Wait for work to be released.
        LF_PRINT_DEBUG(
            "Scheduler: Worker %d is trying to acquire the scheduling "
            "semaphore.",
            worker_number);
        lf_semaphore_acquire(_lf_sched_instance->_lf_sched_semaphore);
        LF_PRINT_DEBUG("Scheduler: Worker %d acquired the scheduling semaphore.",
                    worker_number);
    }
}

///////////////////// Scheduler Init and Destroy API /////////////////////////
/**
 * @brief Initialize the scheduler.
 *
 * This has to be called before other functions of the scheduler can be used.
 * If the scheduler is already initialized, this will be a no-op.
 *
 * @param number_of_workers Indicate how many workers this scheduler will be
 *  managing.
 * @param option Pointer to a `sched_params_t` struct containing additional
 *  scheduler parameters.
 */
void lf_sched_init(
    size_t number_of_workers, 
    sched_params_t* params
) {
    LF_PRINT_DEBUG("Scheduler: Initializing with %d workers", number_of_workers);
    if(!init_sched_instance(&_lf_sched_instance, number_of_workers, params)) {
        // Already initialized
        return;
    }

    _lf_sched_instance->_lf_sched_triggered_reactions = calloc(
        (_lf_sched_instance->max_reaction_level + 1),
        sizeof(pqueue_t*));

    _lf_sched_instance->_lf_sched_array_of_mutexes = (lf_mutex_t*)calloc(
        (_lf_sched_instance->max_reaction_level + 1), sizeof(lf_mutex_t));

    size_t queue_size = INITIAL_REACT_QUEUE_SIZE;
    for (size_t i = 0; i <= _lf_sched_instance->max_reaction_level; i++) {
        if (params != NULL) {
            if (params->num_reactions_per_level != NULL) {
                queue_size = params->num_reactions_per_level[i];
            }
        }
        // Initialize the reaction queues
        ((pqueue_t**)_lf_sched_instance->_lf_sched_triggered_reactions)[i] =
            pqueue_init(queue_size, in_reverse_order, get_reaction_index,
                        get_reaction_position, set_reaction_position,
                        reaction_matches, print_reaction);
        // Initialize the mutexes for the reaction queues
        lf_mutex_init(&_lf_sched_instance->_lf_sched_array_of_mutexes[i]);
    }

    _lf_sched_instance->_lf_sched_executing_reactions =
        ((pqueue_t**)_lf_sched_instance->_lf_sched_triggered_reactions)[0];
}

/**
 * @brief Free the memory used by the scheduler.
 *
 * This must be called when the scheduler is no longer needed.
 */
void lf_sched_free() {
    // for (size_t j = 0; j <= _lf_sched_instance->max_reaction_level; j++) {
    //     pqueue_free(_lf_sched_instance->_lf_sched_triggered_reactions[j]);
    //     FIXME: This is causing weird memory errors.
    // }
    pqueue_free((pqueue_t*)_lf_sched_instance->_lf_sched_executing_reactions);
    lf_semaphore_destroy(_lf_sched_instance->_lf_sched_semaphore);
}

///////////////////// Scheduler Worker API (public) /////////////////////////
/**
 * @brief Ask the scheduler for one more reaction.
 *
 * This function blocks until it can return a ready reaction for worker thread
 * 'worker_number' or it is time for the worker thread to stop and exit (where a
 * NULL value would be returned).
 *
 * @param worker_number
 * @return reaction_t* A reaction for the worker to execute. NULL if the calling
 * worker thread should exit.
 */
reaction_t* lf_sched_get_ready_reaction(int worker_number) {
    // Iterate until the stop_tag is reached or reaction queue is empty
    while (!_lf_sched_instance->_lf_sched_should_stop) {
        // Need to lock the mutex for the current level
        size_t current_level =
            _lf_sched_instance->_lf_sched_next_reaction_level - 1;
        LF_PRINT_DEBUG(
            "Scheduler: Worker %d trying to lock the mutex for level %d.",
            worker_number, current_level);
        lf_mutex_lock(
            &_lf_sched_instance->_lf_sched_array_of_mutexes[current_level]);
        LF_PRINT_DEBUG("Scheduler: Worker %d locked the mutex for level %d.",
                    worker_number, current_level);
        reaction_t* reaction_to_return = (reaction_t*)pqueue_pop(
            (pqueue_t*)_lf_sched_instance->_lf_sched_executing_reactions);
        lf_mutex_unlock(
            &_lf_sched_instance->_lf_sched_array_of_mutexes[current_level]);

        if (reaction_to_return != NULL) {
            // Got a reaction
            return reaction_to_return;
        }

        LF_PRINT_DEBUG("Worker %d is out of ready reactions.", worker_number);

        // Ask the scheduler for more work and wait
        tracepoint_worker_wait_starts(worker_number);
        _lf_sched_wait_for_work(worker_number);
        tracepoint_worker_wait_ends(worker_number);
    }

    // It's time for the worker thread to stop and exit.
    return NULL;
}

/**
 * @brief Inform the scheduler that worker thread 'worker_number' is done
 * executing the 'done_reaction'.
 *
 * @param worker_number The worker number for the worker thread that has
 * finished executing 'done_reaction'.
 * @param done_reaction The reaction that is done.
 */
void lf_sched_done_with_reaction(size_t worker_number,
                                 reaction_t* done_reaction) {
    if (!lf_bool_compare_and_swap(&done_reaction->status, queued, inactive)) {
        lf_print_error_and_exit("Unexpected reaction status: %d. Expected %d.",
                             done_reaction->status, queued);
    }
}

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
 * @param worker_number The ID of the worker that is making this call. 0 should
 * be used if there is only one worker (e.g., when the program is using the
 *  unthreaded C runtime). -1 is used for an anonymous call in a context where a
 *  worker number does not make sense (e.g., the caller is not a worker thread).
 */
void lf_sched_trigger_reaction(reaction_t* reaction, int worker_number) {
    if (reaction == NULL || !lf_bool_compare_and_swap(&reaction->status, inactive, queued)) {
        return;
    }
    LF_PRINT_DEBUG("Scheduler: Enqueing reaction %s, which has level %lld.",
            reaction->name, LEVEL(reaction->index));
    _lf_sched_insert_reaction(reaction);
}
