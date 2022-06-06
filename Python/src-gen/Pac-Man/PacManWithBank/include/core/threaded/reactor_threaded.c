/* Runtime infrastructure for the threaded version of the C target of Lingua Franca. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

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

/** Runtime infrastructure for the threaded version of the C target of Lingua Franca.
 *  
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Soroush Bateni <soroush@utdallas.edu>}
 */

#ifndef NUMBER_OF_WORKERS
#define NUMBER_OF_WORKERS 1
#endif // NUMBER_OF_WORKERS

#include "../reactor_common.c"
#include "../platform.h"
#include "scheduler.h"
#include <signal.h>


/**
 * The maximum amount of time a worker thread should stall
 * before checking the reaction queue again.
 * This is not currently used.
 */
#define MAX_STALL_INTERVAL MSEC(1)

/**
 * Unless the "fast" option is given, an LF program will wait until
 * physical time matches logical time before handling an event with
 * a given logical time. The amount of time is less than this given
 * threshold, then no wait will occur. The purpose of this is
 * to prevent unnecessary delays caused by simply setting up and
 * performing the wait.
 */
#define MIN_WAIT_TIME USEC(10)

/*
 * A struct representing a barrier in threaded 
 * Lingua Franca programs that can prevent advancement 
 * of tag if
 * 1- Number of requestors is larger than 0
 * 2- Value of horizon is not (FOREVER, 0)
 */
typedef struct _lf_tag_advancement_barrier {
    int requestors; // Used to indicate the number of
                    // requestors that have asked
                    // for a barrier to be raised
                    // on tag.
    tag_t horizon;  // If semaphore is larger than 0
                    // then the runtime should not
                    // advance its tag beyond the
                    // horizon.
} _lf_tag_advancement_barrier;


/**
 * Create a global tag barrier and
 * initialize the barrier's semaphore to 0 and its horizon to FOREVER_TAG.
 */
_lf_tag_advancement_barrier _lf_global_tag_advancement_barrier = {0, FOREVER_TAG_INITIALIZER};

// The one and only mutex lock.
lf_mutex_t mutex;

// Condition variables used for notification between threads.
lf_cond_t event_q_changed;
// A condition variable that notifies threads whenever the number
// of requestors on the tag barrier reaches zero.
lf_cond_t global_tag_barrier_requestors_reached_zero;

/**
 * Enqueue network input control reactions that determine if the trigger for a
 * given network input port is going to be present at the current logical time
 * or absent.
 */
void enqueue_network_input_control_reactions();

/**
 * Enqueue network output control reactions that will send a PORT_ABSENT
 * message to downstream federates if a given network output port is not present.
 */
void enqueue_network_output_control_reactions();

/**
 * Raise a barrier to prevent the current tag from advancing to or
 * beyond the value of the future_tag argument, if possible.
 * If the current tag is already at or beyond future_tag, then
 * prevent any further advances. This function will increment the
 * total number of pending barrier requests. For each call to this
 * function, there should always be a subsequent call to
 * _lf_decrement_global_tag_barrier_locked()
 * to release the barrier.
 * 
 * If there is already a barrier raised at a tag later than future_tag, this
 * function will change the barrier to future_tag or the current tag, whichever
 * is larger. If the existing barrier is earlier 
 * than future_tag, this function will not change the barrier. If there are
 * no existing barriers and future_tag is in the past relative to the 
 * current tag, this function will raise a barrier to the current tag.
 * 
 * This function assumes the mutex lock is already held, thus, it will not
 * acquire it itself.
 * 
 * @note This function is only useful in threaded applications to facilitate
 *  certain non-blocking functionalities such as receiving timed messages
 *  over the network or handling stop in a federated execution.
 * 
 * @param future_tag A desired tag for the barrier. This function will guarantee
 * that current logical time will not go past future_tag if it is in the future.
 * If future_tag is in the past (or equals to current logical time), the runtime
 * will freeze advancement of logical time.
 */
void _lf_increment_global_tag_barrier_already_locked(tag_t future_tag) {
    // Check if future_tag is after stop tag.
    // This will only occur when a federate receives a timed message with 
    // a tag that is after the stop tag
    if (_lf_is_tag_after_stop_tag(future_tag)) {
        lf_print_warning("Attempting to raise a barrier after the stop tag.");
        future_tag = stop_tag;
    }
    tag_t current_tag = lf_tag();
    // Check to see if future_tag is actually in the future.
    if (lf_tag_compare(future_tag, current_tag) > 0) {
        // Future tag is actually in the future.
        // See whether it is smaller than any pre-existing barrier.
        if (lf_tag_compare(future_tag, _lf_global_tag_advancement_barrier.horizon) < 0) {
            // The future tag is smaller than the current horizon of the barrier.
            // Therefore, we should prevent logical time from reaching the
            // future tag.
            _lf_global_tag_advancement_barrier.horizon = future_tag;
            LF_PRINT_DEBUG("Raised barrier at elapsed tag (%lld, %u).",
                        _lf_global_tag_advancement_barrier.horizon.time - start_time,
                        _lf_global_tag_advancement_barrier.horizon.microstep);
        } 
    } else {
            // The future_tag is not in the future.

            // One possibility is that the incoming message has violated the STP offset.
            // Another possibility is that the message is coming from a zero-delay loop,
            // and control reactions are waiting.

            // Prevent logical time from advancing further so that the measure of
            // STP violation properly reflects the amount of time (logical or physical)
            // that has elapsed after the incoming message would have violated the STP offset.
            _lf_global_tag_advancement_barrier.horizon = current_tag;
            _lf_global_tag_advancement_barrier.horizon.microstep++;
            LF_PRINT_DEBUG("Raised barrier at elapsed tag (%lld, %u).",
                        _lf_global_tag_advancement_barrier.horizon.time - start_time,
                        _lf_global_tag_advancement_barrier.horizon.microstep);
    }
    // Increment the number of requestors
    _lf_global_tag_advancement_barrier.requestors++;
}

/**
 * Raise a barrier to prevent the current tag from advancing to or
 * beyond the value of the future_tag argument, if possible.
 * If the current tag is already at or beyond future_tag, then
 * prevent any further advances. This function will increment the
 * total number of pending barrier requests. For each call to this
 * function, there should always be a subsequent call to
 * _lf_decrement_global_tag_barrier_locked()
 * to release the barrier.
 * 
 * If there is already a barrier raised at a tag later than future_tag, this
 * function will change the barrier to future_tag or the current tag, whichever
 * is larger. If the existing barrier is earlier 
 * than future_tag, this function will not change the barrier. If there are
 * no existing barriers and future_tag is in the past relative to the 
 * current tag, this function will raise a barrier to the current tag.
 * 
 * This function acquires the mutex lock .
 * 
 * @note This function is only useful in threaded applications to facilitate
 *  certain non-blocking functionalities such as receiving timed messages
 *  over the network or handling stop in a federated execution.
 * 
 * @param future_tag A desired tag for the barrier. This function will guarantee
 * that current tag will not go past future_tag if it is in the future.
 * If future_tag is in the past (or equals to current tag), the runtime
 * will freeze advancement of tag.
 */
void _lf_increment_global_tag_barrier(tag_t future_tag) {
    lf_mutex_lock(&mutex);
    _lf_increment_global_tag_barrier_already_locked(future_tag);
    lf_mutex_unlock(&mutex);
}

/**
 * Decrement the total number of pending barrier requests for the global tag barrier.
 * If the total number of requests reaches zero, this function resets the
 * tag barrier to FOREVER_TAG and notifies all threads that are waiting
 * on the barrier that the number of requests has reached zero.
 * 
 * This function assumes that the caller already holds the mutex lock.
 * 
 * @note This function is only useful in threaded applications to facilitate
 *  certain non-blocking functionalities such as receiving timed messages
 *  over the network or handling stop in the federated execution.
 */
void _lf_decrement_global_tag_barrier_locked() {
    // Decrement the number of requestors for the tag barrier.
    _lf_global_tag_advancement_barrier.requestors--;
    // Check to see if the semaphore is negative, which indicates that
    // a mismatched call was placed for this function.
    if (_lf_global_tag_advancement_barrier.requestors < 0) {
        lf_print_error_and_exit("Mismatched use of _lf_increment_global_tag_barrier()"
                " and  _lf_decrement_global_tag_barrier_locked().");
    } else if (_lf_global_tag_advancement_barrier.requestors == 0) {
        // When the semaphore reaches zero, reset the horizon to forever.
        _lf_global_tag_advancement_barrier.horizon = FOREVER_TAG;
        // Notify waiting threads that the semaphore has reached zero.
        lf_cond_broadcast(&global_tag_barrier_requestors_reached_zero);
    }
    LF_PRINT_DEBUG("Barrier is at tag (%lld, %u).",
                 _lf_global_tag_advancement_barrier.horizon.time,
                 _lf_global_tag_advancement_barrier.horizon.microstep);
}

/**
 * If the proposed_tag is greater than or equal to a barrier tag that has been
 * set by a call to _lf_increment_global_tag_barrier or
 * _lf_increment_global_tag_barrier_already_locked, and if there are requestors
 * still pending on that barrier, then wait until all requestors have been
 * satisfied. This is used in federated execution when an incoming timed
 * message has been partially read so that we know its tag, but the rest of
 * message has not yet been read and hence the event has not yet appeared
 * on the event queue.  To prevent tardiness, this function blocks the
 * advancement of time until to the proposed tag until the message has
 * been put onto the event queue.
 *
 * If the prposed_tag is greater than the stop tag, then use the stop tag instead.
 * 
 * This function assumes the mutex is already locked.
 * Thus, it unlocks the mutex while it's waiting to allow
 * the tag barrier to change.
 * 
 * @param proposed_tag The tag that the runtime wants to advance to.
 * @return 0 if no wait was needed and 1 if a wait actually occurred.
 */
int _lf_wait_on_global_tag_barrier(tag_t proposed_tag) {
    // Check the most common case first.
    if (_lf_global_tag_advancement_barrier.requestors == 0) return 0;
    
    // Do not wait for tags after the stop tag
    if (_lf_is_tag_after_stop_tag(proposed_tag)) {
        proposed_tag = stop_tag;
    }
    // Do not wait forever
    if (proposed_tag.time == FOREVER) {
        lf_print_warning("Global tag barrier should not handle FOREVER proposed tags.");
        return 0;
    }
    int result = 0;
    // Wait until the global barrier semaphore on logical time is zero
    // and the proposed_time is larger than or equal to the horizon.
    while (_lf_global_tag_advancement_barrier.requestors > 0
            && lf_tag_compare(proposed_tag, _lf_global_tag_advancement_barrier.horizon) >= 0
    ) {
        result = 1;
        LF_PRINT_LOG("Waiting on barrier for tag (%lld, %u).", proposed_tag.time - start_time, proposed_tag.microstep);
        // Wait until no requestor remains for the barrier on logical time
        lf_cond_wait(&global_tag_barrier_requestors_reached_zero, &mutex);
        
        // The stop tag may have changed during the wait.
        if (_lf_is_tag_after_stop_tag(proposed_tag)) {
            proposed_tag = stop_tag;
        }
    }
    return result;
}

/**
 * Schedule the specified trigger at current_tag.time plus the offset of the
 * specified trigger plus the delay.
 * See reactor.h for documentation.
 */
trigger_handle_t _lf_schedule_token(void* action, interval_t extra_delay, lf_token_t* token) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    lf_mutex_lock(&mutex);
    int return_value = _lf_schedule(trigger, extra_delay, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    lf_cond_broadcast(&event_q_changed);
    lf_mutex_unlock(&mutex);
    return return_value;
}

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value.
 * See reactor.h for documentation.
 */
trigger_handle_t _lf_schedule_copy(void* action, interval_t offset, void* value, size_t length) {
    if (value == NULL) {
        return _lf_schedule_token(action, offset, NULL);
    }
    trigger_t* trigger = _lf_action_to_trigger(action);

    if (trigger == NULL || trigger->token == NULL || trigger->token->element_size <= 0) {
        lf_print_error("schedule: Invalid trigger or element size.");
        return -1;
    }
    lf_mutex_lock(&mutex);
    // Initialize token with an array size of length and a reference count of 0.
    lf_token_t* token = _lf_initialize_token(trigger->token, length);
    // Copy the value into the newly allocated memory.
    memcpy(token->value, value, token->element_size * length);
    // The schedule function will increment the reference count.
    trigger_handle_t result = _lf_schedule(trigger, offset, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    lf_cond_signal(&event_q_changed);
    lf_mutex_unlock(&mutex);
    return result;
}

/**
 * Variant of schedule_token that creates a token to carry the specified value.
 * See reactor.h for documentation.
 */
trigger_handle_t _lf_schedule_value(void* action, interval_t extra_delay, void* value, size_t length) {
    trigger_t* trigger = _lf_action_to_trigger(action);

    lf_mutex_lock(&mutex);
    lf_token_t* token = create_token(trigger->element_size);
    token->value = value;
    token->length = length;
    int return_value = _lf_schedule(trigger, extra_delay, token);
    // Notify the main thread in case it is waiting for physical time to elapse.
    lf_cond_signal(&event_q_changed);
    lf_mutex_unlock(&mutex);
    return return_value;
}

/*
 * Mark the given is_present field as true. This is_present field
 * will later be cleaned up by _lf_start_time_step.
 * This assumes that the mutex is not held.
 * @param is_present_field A pointer to the is_present field that
 * must be set.
 */
void _lf_set_present(bool* is_present_field) {
    int ipfas = lf_atomic_fetch_add(&_lf_is_present_fields_abbreviated_size, 1);
    if (ipfas < _lf_is_present_fields_size) {
        _lf_is_present_fields_abbreviated[ipfas] = is_present_field;
    }
    *is_present_field = true;
}

/** 
 * Synchronize the start with other federates via the RTI.
 * This assumes that a connection to the RTI is already made 
 * and _fed.socket_TCP_RTI is valid. It then sends the current logical
 * time to the RTI and waits for the RTI to respond with a specified
 * time. It starts a thread to listen for messages from the RTI.
 * It then waits for physical time to match the specified time,
 * sets current logical time to the time returned by the RTI,
 * and then returns. If --fast was specified, then this does
 * not wait for physical time to match the logical start time
 * returned by the RTI.
 */
void synchronize_with_other_federates();

/**
 * Wait until physical time matches or exceeds the specified logical time,
 * unless -fast is given.
 *
 * If an event is put on the event queue during the wait, then the wait is
 * interrupted and this function returns false. It also returns false if the
 * timeout time is reached before the wait has completed.
 * 
 * The mutex lock is assumed to be held by the calling thread.
 * Note this this could return true even if the a new event
 * was placed on the queue if that event time matches or exceeds
 * the specified time.
 *
 * @param logical_time_ns Logical time to wait until physical time matches it.
 * @param return_if_interrupted If this is false, then wait_util will wait
 *  until physical time matches the logical time regardless of whether new
 *  events get put on the event queue. This is useful, for example, for
 *  synchronizing the start of the program.
 * 
 * @return Return false if the wait is interrupted either because of an event
 *  queue signal or if the wait time was interrupted early by reaching
 *  the stop time, if one was specified. Return true if the full wait time
 *  was reached.
 */
bool wait_until(instant_t logical_time_ns, lf_cond_t* condition) {
    LF_PRINT_DEBUG("-------- Waiting until physical time matches logical time %lld", logical_time_ns);
    bool return_value = true;
    interval_t wait_until_time_ns = logical_time_ns;
#ifdef FEDERATED_DECENTRALIZED // Only apply the STA if coordination is decentralized
    // Apply the STA to the logical time
    // Prevent an overflow
    if (wait_until_time_ns < FOREVER - _lf_fed_STA_offset) {
        // If wait_time is not forever
        LF_PRINT_DEBUG("Adding STA %lld to wait until time %lld.",
                _lf_fed_STA_offset,
                wait_until_time_ns - start_time);
        wait_until_time_ns += _lf_fed_STA_offset;
    }
#endif
    if (!fast) {
        // Get physical time as adjusted by clock synchronization offset.
        instant_t current_physical_time = lf_time_physical();
        // We want to wait until that adjusted time matches the logical time.
        interval_t ns_to_wait = wait_until_time_ns - current_physical_time;
        // We should not wait if that adjusted time is already ahead
        // of logical time.
        if (ns_to_wait < MIN_WAIT_TIME) {
            LF_PRINT_DEBUG("Wait time %lld is less than MIN_WAIT_TIME %lld. Skipping wait.",
                ns_to_wait, MIN_WAIT_TIME);
            return return_value;
        }

        // We will use lf_cond_timedwait, which takes as an argument the absolute
        // time to wait until. However, that will not include the offset that we
        // have calculated with clock synchronization. So we need to instead ensure
        // that the time it waits is ns_to_wait.
        // We need the current clock value as obtained using CLOCK_REALTIME because
        // that is what lf_cond_timedwait will use.
        // The above call to setPhysicalTime() set the
        // _lf_last_reported_unadjusted_physical_time_ns to the CLOCK_REALTIME value
        // unadjusted by clock synchronization.
        // Note that if ns_to_wait is large enough, then the following addition could
        // overflow. This could happen, for example, if wait_until_time_ns == FOREVER.
        instant_t unadjusted_wait_until_time_ns = FOREVER;
        if (FOREVER - _lf_last_reported_unadjusted_physical_time_ns > ns_to_wait) {
            unadjusted_wait_until_time_ns = _lf_last_reported_unadjusted_physical_time_ns + ns_to_wait;
        }
        LF_PRINT_DEBUG("-------- Clock offset is %lld ns.", current_physical_time - _lf_last_reported_unadjusted_physical_time_ns);
        LF_PRINT_DEBUG("-------- Waiting %lld ns for physical time to match logical time %llu.", ns_to_wait, 
                logical_time_ns - start_time);

        // lf_cond_timedwait returns 0 if it is awakened before the timeout.
        // Hence, we want to run it repeatedly until either it returns non-zero or the
        // current physical time matches or exceeds the logical time.
        if (lf_cond_timedwait(condition, &mutex, unadjusted_wait_until_time_ns) != LF_TIMEOUT) {
            LF_PRINT_DEBUG("-------- wait_until interrupted before timeout.");

            // Wait did not time out, which means that there
            // may have been an asynchronous call to lf_schedule().
            // Continue waiting.
            // Do not adjust current_tag.time here. If there was an asynchronous
            // call to lf_schedule(), it will have put an event on the event queue,
            // and current_tag.time will be set to that time when that event is pulled.
            return_value = false;
        } else {
            // Reached timeout.
            // FIXME: move this to Mac-specific platform implementation
            // Unfortunately, at least on Macs, pthread_cond_timedwait appears
            // to be implemented incorrectly and it returns well short of the target
            // time.  Check for this condition and wait again if necessary.
            interval_t ns_to_wait = wait_until_time_ns - lf_time_physical();
            // We should not wait if that adjusted time is already ahead
            // of logical time.
            if (ns_to_wait < MIN_WAIT_TIME) {
                return true;
            }
            LF_PRINT_DEBUG("-------- lf_cond_timedwait claims to have timed out, "
                    "but it did not reach the target time. Waiting again.");
            return wait_until(wait_until_time_ns, condition);
        }

        LF_PRINT_DEBUG("-------- Returned from wait, having waited %lld ns.", lf_time_physical() - current_physical_time);
    }
    return return_value;
}

/**
 * Return the tag of the next event on the event queue.
 * If the event queue is empty then return either FOREVER_TAG
 * or, is a stop_time (timeout time) has been set, the stop time.
 */
tag_t get_next_event_tag() {
    // Peek at the earliest event in the event queue.
    event_t* event = (event_t*)pqueue_peek(event_q);
    tag_t next_tag = FOREVER_TAG;
    if (event != NULL) {
        // There is an event in the event queue.
        if (event->time < current_tag.time) {
            lf_print_error_and_exit("get_next_event_tag(): Earliest event on the event queue (%lld) is "
                                  "earlier than the current time (%lld).",
                                  event->time - start_time,
                                  current_tag.time - start_time);
        }

        next_tag.time = event->time;
        if (next_tag.time == current_tag.time) {
        	LF_PRINT_DEBUG("Earliest event matches current time. Incrementing microstep. Event is dummy: %d.",
        			event->is_dummy);
            next_tag.microstep =  lf_tag().microstep + 1;
        } else {
            next_tag.microstep = 0;
        }
    }

    // If a timeout tag was given, adjust the next_tag from the
    // event tag to that timeout tag.
    if (_lf_is_tag_after_stop_tag(next_tag)) {
        next_tag = stop_tag;
    }
    LF_PRINT_LOG("Earliest event on the event queue (or stop time if empty) is (%lld, %u). Event queue has size %d.",
            next_tag.time - start_time, next_tag.microstep, pqueue_size(event_q));
    return next_tag;
}

#ifdef FEDERATED_CENTRALIZED
// The following is defined in federate.c and used in the following function.
tag_t _lf_send_next_event_tag(tag_t tag, bool wait_for_reply);
#endif

/**
 * In a federated execution with centralized coordination, this function returns
 * a tag that is less than or equal to the specified tag when, as far
 * as the federation is concerned, it is safe to commit to advancing
 * to the returned tag. That is, all incoming network messages with
 * tags less than the returned tag have been received.
 * In unfederated execution or in federated execution with decentralized
 * control, this function returns the specified tag immediately.
 *
 * @param tag The tag to which to advance.
 * @param wait_for_reply If true, wait for the RTI to respond.
 * @return The tag to which it is safe to advance.
 */
tag_t send_next_event_tag(tag_t tag, bool wait_for_reply) {
#ifdef FEDERATED_CENTRALIZED
    return _lf_send_next_event_tag(tag, wait_for_reply);
#else
    return tag;
#endif
}

/**
 * If there is at least one event in the event queue, then wait until
 * physical time matches or exceeds the time of the least tag on the event
 * queue; pop the next event(s) from the event queue that all have the same tag;
 * extract from those events the reactions that are to be invoked at this
 * logical time and insert them into the reaction queue. The event queue is
 * sorted by time tag.
 *
 * If there is no event in the queue and the keepalive command-line option was
 * not given, and this is not a federated execution with centralized coordination,
 * set the stop tag to the current tag.
 * If keepalive was given, then wait for either lf_request_stop()
 * to be called or an event appears in the event queue and then return.
 *
 * Every time tag is advanced, it is checked against stop tag and if they are
 * equal, shutdown reactions are triggered.
 *
 * This does not acquire the mutex lock. It assumes the lock is already held.
 */
void _lf_next_locked() {
#ifdef MODAL_REACTORS
    // Perform mode transitions
    _lf_handle_mode_changes();
#endif

    // Previous logical time is complete.
    tag_t next_tag = get_next_event_tag();

#ifdef FEDERATED_CENTRALIZED
    // In case this is in a federation with centralized coordination, notify 
    // the RTI of the next earliest tag at which this federate might produce 
    // an event. This function may block until it is safe to advance the current 
    // tag to the next tag. Specifically, it blocks if there are upstream 
    // federates. If an action triggers during that wait, it will unblock
    // and return with a time (typically) less than the next_time.
    tag_t grant_tag = send_next_event_tag(next_tag, true); // true means this blocks.
    if (lf_tag_compare(grant_tag, next_tag) < 0) {
        // RTI has granted tag advance to an earlier tag or the wait
        // for the RTI response was interrupted by a local physical action with
        // a tag earlier than requested.
        // Continue executing. The event queue may have changed.
        return;
    }
    // Granted tag is greater than or equal to next event tag that we sent to the RTI.
    // Since send_next_event_tag releases the mutex lock internally, we need to check
    // again for what the next tag is (e.g., the stop time could have changed).
    next_tag = get_next_event_tag();
    
    // FIXME: Do starvation analysis for centralized coordination.
    // Specifically, if the event queue is empty on *all* federates, this
    // can become known to the RTI which can then stop execution.
    // Hence, it will no longer be necessary to force keepalive to be true
    // for all federated execution. With centralized coordination, we could
    // allow keepalive to be either true or false and could get the same
    // behavior with centralized coordination as with unfederated execution.

#else  // not FEDERATED_CENTRALIZED
    if (pqueue_peek(event_q) == NULL && !keepalive_specified) {
        // There is no event on the event queue and keepalive is false.
        // No event in the queue
        // keepalive is not set so we should stop.
        // Note that federated programs with decentralized coordination always have
        // keepalive = true
        _lf_set_stop_tag((tag_t){.time=current_tag.time,.microstep=current_tag.microstep+1});

        // Stop tag has changed. Need to check next_tag again.
        next_tag = get_next_event_tag();
    }
#endif

    // Wait for physical time to advance to the next event time (or stop time).
    // This can be interrupted if a physical action triggers (e.g., a message
    // arrives from an upstream federate or a local physical action triggers).
    LF_PRINT_LOG("Waiting until elapsed time %lld.", (next_tag.time - start_time));
    while (!wait_until(next_tag.time, &event_q_changed)) {
        LF_PRINT_DEBUG("_lf_next_locked(): Wait until time interrupted.");
        // Sleep was interrupted.  Check for a new next_event.
        // The interruption could also have been due to a call to lf_request_stop().
        next_tag = get_next_event_tag();

        // If this (possibly new) next tag is past the stop time, return.
        if (_lf_is_tag_after_stop_tag(next_tag)) {
            return;
        }
    }
    // A wait occurs even if wait_until() returns true, which means that the
    // tag on the head of the event queue may have changed.
    next_tag = get_next_event_tag();

    // If this (possibly new) next tag is past the stop time, return.
    if (_lf_is_tag_after_stop_tag(next_tag)) { // lf_tag_compare(tag, stop_tag) > 0
        return;
    }

    LF_PRINT_DEBUG("Physical time is ahead of next tag time by %lld. This should be small unless -fast is used.",
                lf_time_physical() - next_tag.time);
    
#ifdef FEDERATED
    // In federated execution (at least under decentralized coordination),
    // it is possible that an incoming message has been partially read,
    // enough to see its tag. To prevent it from becoming tardy, the thread
    // that is reading the message has set a barrier to prevent logical time
    // from exceeding the timestamp of the message. It will remove that barrier
    // once the complete message has been read. Here, we wait for that barrier
    // to be removed, if appropriate.
    if(_lf_wait_on_global_tag_barrier(next_tag)) {
        // A wait actually occurred, so the next_tag may have changed again.
        next_tag = get_next_event_tag();
    }
#endif // FEDERATED

    // If the first event in the event queue has a tag greater than or equal to the
    // stop time, and the current_tag matches the stop tag (meaning that we have already
    // executed microstep 0 at the timeout time), then we are done. The above code prevents the next_tag
    // from exceeding the stop_tag, so we have to do further checks if
    // they are equal.
    if (lf_tag_compare(next_tag, stop_tag) >= 0 && lf_tag_compare(current_tag, stop_tag) >= 0) {
        // If we pop anything further off the event queue with this same time or larger,
        // then it will be assigned a tag larger than the stop tag.
        return;
    }

    // Invoke code that must execute before starting a new logical time round,
    // such as initializing outputs to be absent.
    _lf_start_time_step();
        
    // At this point, finally, we have an event to process.
    // Advance current time to match that of the first event on the queue.
    _lf_advance_logical_time(next_tag.time);

    if (lf_tag_compare(current_tag, stop_tag) >= 0) {
        // Pop shutdown events
        LF_PRINT_DEBUG("Scheduling shutdown reactions.");
        _lf_trigger_shutdown_reactions();
    }

    // Pop all events from event_q with timestamp equal to current_tag.time,
    // extract all the reactions triggered by these events, and
    // stick them into the reaction queue.
    _lf_pop_events();
}

/**
 * Request a stop to execution as soon as possible.
 * In a non-federated execution, this will occur
 * at the conclusion of the current logical time.
 * In a federated execution, it will likely occur at
 * a later logical time determined by the RTI so that
 * all federates stop at the same logical time.
 */
void lf_request_stop() {
    lf_mutex_lock(&mutex);
    // Check if already at the previous stop tag.
    if (lf_tag_compare(current_tag, stop_tag) >= 0) {
        // If so, ignore the stop request since the program
        // is already stopping at the current tag.
        lf_mutex_unlock(&mutex);
        return;
    }
#ifdef FEDERATED
    _lf_fd_send_stop_request_to_rti();
    // Do not set stop_requested
    // since the RTI might grant a
    // later stop tag than the current
    // tag. The _lf_fd_send_request_stop_to_rti() 
    // will raise a barrier at the current
    // logical time.
#else
    // In a non-federated program, the stop_tag will be the next microstep
    _lf_set_stop_tag((tag_t) {.time = current_tag.time, .microstep = current_tag.microstep+1});
    // We signal instead of broadcast under the assumption that only
    // one worker thread can call wait_until at a given time because
    // the call to wait_until is protected by a mutex lock
    lf_cond_signal(&event_q_changed);
#endif
    lf_mutex_unlock(&mutex);
}

/**
 * Trigger 'reaction'.
 * 
 * @param reaction The reaction.
 * @param worker_number The ID of the worker that is making this call. 0 should be
 *  used if there is only one worker (e.g., when the program is using the
 *  unthreaded C runtime). -1 is used for an anonymous call in a context where a
 *  worker number does not make sense (e.g., the caller is not a worker thread).
 */
void _lf_trigger_reaction(reaction_t* reaction, int worker_number) {
#ifdef MODAL_REACTORS
        // Check if reaction is disabled by mode inactivity
        if (_lf_mode_is_active(reaction->mode)) {
#endif
    lf_sched_trigger_reaction(reaction, worker_number);
#ifdef MODAL_REACTORS
        } else { // Suppress reaction by preventing entering reaction queue
            LF_PRINT_DEBUG("Suppressing downstream reaction %s due inactivity of mode %s.",
            		reaction->name, reaction->mode->name);
        }
#endif
}

/**
 * Perform the necessary operations before tag (0,0) can be processed.
 * 
 * This includes injecting any reactions triggered at (0,0), initializing timers,
 * and for the federated execution, waiting for a proper coordinated start.
 *
 * This assumes the mutex lock is held by the caller.
 */
void _lf_initialize_start_tag() {

    // Add reactions invoked at tag (0,0) (including startup reactions) to the reaction queue
    _lf_trigger_startup_reactions(); 

#ifdef FEDERATED
    // Reset status fields before talking to the RTI to set network port
    // statuses to unknown
    reset_status_fields_on_input_port_triggers();

    // Get a start_time from the RTI
    synchronize_with_other_federates(); // Resets start_time in federated execution according to the RTI.
    current_tag = (tag_t){.time = start_time, .microstep = 0u};
#endif

    _lf_initialize_timers();

    // If the stop_tag is (0,0), also insert the shutdown
    // reactions. This can only happen if the timeout time
    // was set to 0.
    if (lf_tag_compare(current_tag, stop_tag) >= 0) {
        _lf_trigger_shutdown_reactions();
    }

#ifdef FEDERATED
    // Call wait_until if federated. This is required because the startup procedure
    // in synchronize_with_other_federates() can decide on a new start_time that is 
    // larger than the current physical time.
    // Therefore, if --fast was not specified, wait until physical time matches
    // or exceeds the start time. Microstep is ignored.  
    // This wait_until() is deliberately called after most precursor operations
    // for tag (0,0) are performed (e.g., injecting startup reactions, etc.).
    // This has two benefits: First, the startup overheads will reduce 
    // the required waiting time. Second, this call releases the mutex lock and allows
    // other threads (specifically, federate threads that handle incoming p2p messages 
    // from other federates) to hold the lock and possibly raise a tag barrier. This is 
    // especially useful if an STA is set properly because the federate will get
    // a chance to process incoming messages while utilizing the STA.
    LF_PRINT_LOG("Waiting for start time %lld plus STA %lld.",
            start_time, _lf_fed_STA_offset);
    // Ignore interrupts to this wait. We don't want to start executing until
    // physical time matches or exceeds the logical start time.
    while (!wait_until(start_time, &event_q_changed)) {}
    LF_PRINT_DEBUG("Done waiting for start time %lld.", start_time);
    LF_PRINT_DEBUG("Physical time is ahead of current time by %lld. This should be small.",
            lf_time_physical() - start_time);

    // Reinitialize the physical start time to match the start_time.
    // Otherwise, reports of lf_time_physical() are not very meaningful
    // w.r.t. logical time.
    physical_start_time = start_time;

    // Each federate executes the start tag (which is the current
    // tag). Inform the RTI of this if needed.
    send_next_event_tag(current_tag, true);

    // Depending on RTI's answer, if any, enqueue network control reactions,
    // which will selectively block reactions that depend on network input ports
    // until they receive further instructions (to unblock) from the RTI or the
    // upstream federates.
    enqueue_network_control_reactions();
#endif

#ifdef FEDERATED_DECENTRALIZED
    // In federated execution (at least under decentralized coordination),
    // it is possible that an incoming message has been partially read at (0,0),
    // enough to see its tag. To prevent it from becoming tardy, the thread
    // that is reading the message has set a barrier to prevent logical time
    // from exceeding the timestamp of the message. It will remove that barrier
    // once the complete message has been read. Here, we wait for that barrier
    // to be removed, if appropriate before proceeding to executing tag (0,0).
    _lf_wait_on_global_tag_barrier((tag_t){.time=start_time,.microstep=0});
#endif // FEDERATED_DECENTRALIZED
    
    // Set the following boolean so that other thread(s), including federated threads,
    // know that the execution has started
    _lf_execution_started = true;
}

/** For logging and debugging, each worker thread is numbered. */
int worker_thread_count = 0;

/**
 * Handle deadline violation for 'reaction'. 
 * The mutex should NOT be locked when this function is called. It might acquire
 * the mutex when scheduling the reactions that are triggered as a result of
 * executing the deadline violation handler on the 'reaction', if it exists.
 *
 * @return true if a deadline violation occurred. false otherwise.
 */
bool _lf_worker_handle_deadline_violation_for_reaction(int worker_number, reaction_t* reaction) {
    bool violation_occurred = false;
    // If the reaction has a deadline, compare to current physical time
    // and invoke the deadline violation reaction instead of the reaction function
    // if a violation has occurred. Note that the violation reaction will be invoked
    // at most once per logical time value. If the violation reaction triggers the
    // same reaction at the current time value, even if at a future superdense time,
    // then the reaction will be invoked and the violation reaction will not be invoked again.
    if (reaction->deadline > 0LL) {
        // Get the current physical time.
        instant_t physical_time = lf_time_physical();
        // Check for deadline violation.
        if (physical_time > current_tag.time + reaction->deadline) {
            // Deadline violation has occurred.
            violation_occurred = true;
            // Invoke the local handler, if there is one.
            reaction_function_t handler = reaction->deadline_violation_handler;
            if (handler != NULL) {
                LF_PRINT_LOG("Worker %d: Deadline violation. Invoking deadline handler.",
                        worker_number);
                (*handler)(reaction->self);

                // If the reaction produced outputs, put the resulting
                // triggered reactions into the queue or execute them directly if possible.
                schedule_output_reactions(reaction, worker_number);
                // Remove the reaction from the executing queue.
            }
        }
    }
    return violation_occurred;
}

/**
 * Handle STP violation for 'reaction'. 
 * The mutex should NOT be locked when this function is called. It might acquire
 * the mutex when scheduling the reactions that are triggered as a result of
 * executing the STP violation handler on the 'reaction', if it exists.
 *
 * @return true if an STP violation occurred. false otherwise.
 */
bool _lf_worker_handle_STP_violation_for_reaction(int worker_number, reaction_t* reaction) {
    bool violation_occurred = false;
    // If the reaction violates the STP offset,
    // an input trigger to this reaction has been triggered at a later
    // logical time than originally anticipated. In this case, a special
    // STP handler will be invoked.             
    // FIXME: Note that the STP handler will be invoked
    // at most once per logical time value. If the STP handler triggers the
    // same reaction at the current time value, even if at a future superdense time,
    // then the reaction will be invoked and the STP handler will not be invoked again.
    // However, inputs ports to a federate reactor are network port types so this possibly should
    // be disallowed.
    // @note The STP handler and the deadline handler are not mutually exclusive.
    //  In other words, both can be invoked for a reaction if it is triggered late
    //  in logical time (STP offset is violated) and also misses the constraint on 
    //  physical time (deadline).
    // @note In absence of an STP handler, the is_STP_violated will be passed down the reaction
    //  chain until it is dealt with in a downstream STP handler.
    if (reaction->is_STP_violated == true) {
        reaction_function_t handler = reaction->STP_handler;
        LF_PRINT_LOG("STP violation detected.");
        // Invoke the STP handler if there is one.
        if (handler != NULL) {
            LF_PRINT_LOG("Worker %d: Invoking tardiness handler.", worker_number);
            // There is a violation
            violation_occurred = true;
            (*handler)(reaction->self);
            
            // If the reaction produced outputs, put the resulting
            // triggered reactions into the queue or execute them directly if possible.
            schedule_output_reactions(reaction, worker_number);
            
            // Reset the is_STP_violated because it has been dealt with
            reaction->is_STP_violated = false;
        }
    }
    return violation_occurred;
}

/**
 * Handle violations for 'reaction'. Currently limited to deadline violations
 * and STP violations. 
 * The mutex should NOT be locked when this function is called. It might acquire
 * the mutex when scheduling the reactions that are triggered as a result of
 * executing the deadline or STP violation handler(s) on the 'reaction', if they
 * exist.
 *
 * @return true if a violation occurred. false otherwise.
 */
bool _lf_worker_handle_violations(int worker_number, reaction_t* reaction) {
    bool violation = false;
    
    violation = _lf_worker_handle_deadline_violation_for_reaction(worker_number, reaction) ||
                    _lf_worker_handle_STP_violation_for_reaction(worker_number, reaction);
    return violation;
}

/**
 * Invoke 'reaction' and schedule any resulting triggered reaction(s) on the
 * reaction queue. 
 * The mutex should NOT be locked when this function is called. It might acquire
 * the mutex when scheduling the reactions that are triggered as a result of
 * executing 'reaction'.
 */
void _lf_worker_invoke_reaction(int worker_number, reaction_t* reaction) {
    LF_PRINT_LOG("Worker %d: Invoking reaction %s at elapsed tag (%lld, %d).",
            worker_number,
            reaction->name,
            current_tag.time - start_time,
            current_tag.microstep);
    _lf_invoke_reaction(reaction, worker_number);

    // If the reaction produced outputs, put the resulting triggered
    // reactions into the queue or execute them immediately.
    schedule_output_reactions(reaction, worker_number);
}

/**
 * The main looping logic of each LF worker thread.
 * This function assumes the caller holds the mutex lock.
 * 
 * @param worker_number The number assigned to this worker thread
 */
void _lf_worker_do_work(int worker_number) {
    // Keep track of whether we have decremented the idle thread count.
    // Obtain a reaction from the scheduler that is ready to execute
    // (i.e., it is not blocked by concurrently executing reactions
    // that it depends on).
    // lf_print_snapshot(); // This is quite verbose (but very useful in debugging reaction deadlocks).
    reaction_t* current_reaction_to_execute = NULL;
    while ((current_reaction_to_execute = 
            lf_sched_get_ready_reaction(worker_number)) 
            != NULL) {
        // Got a reaction that is ready to run.
        LF_PRINT_DEBUG("Worker %d: Got from scheduler reaction %s: "
                "level: %lld, is control reaction: %d, chain ID: %llu, and deadline %lld.", 
                worker_number,
                current_reaction_to_execute->name,
                LEVEL(current_reaction_to_execute->index),
                current_reaction_to_execute->is_a_control_reaction,
                current_reaction_to_execute->chain_id,
                current_reaction_to_execute->deadline);

        bool violation = _lf_worker_handle_violations(
            worker_number, 
            current_reaction_to_execute
        );

        if (!violation) {
            // Invoke the reaction function.
            _lf_worker_invoke_reaction(worker_number, current_reaction_to_execute);
        }

        LF_PRINT_DEBUG("Worker %d: Done with reaction %s.",
                worker_number, current_reaction_to_execute->name);

        lf_sched_done_with_reaction(worker_number, current_reaction_to_execute);
    }
}

/**
 * Worker thread for the thread pool.
 * This acquires the mutex lock and releases it to wait for time to
 * elapse or for asynchronous events and also releases it to execute reactions.
 */
void* worker(void* arg) {
    lf_mutex_lock(&mutex);
    int worker_number = worker_thread_count++;
    LF_PRINT_LOG("Worker thread %d started.", worker_number);
    lf_mutex_unlock(&mutex);

    _lf_worker_do_work(worker_number);

    lf_mutex_lock(&mutex);

    // This thread is exiting, so don't count it anymore.
    worker_thread_count--;

    if (worker_thread_count == 0) {
        // The last worker thread to exit will inform the RTI if needed.
        // Notify the RTI that there will be no more events (if centralized coord).
        // False argument means don't wait for a reply.
        send_next_event_tag(FOREVER_TAG, false);
    }

    lf_cond_signal(&event_q_changed);

    LF_PRINT_DEBUG("Worker %d: Stop requested. Exiting.", worker_number);
    lf_mutex_unlock(&mutex);
    // timeout has been requested.
    return NULL;
}

/**
 * If DEBUG logging is enabled, prints the status of the event queue,
 * the reaction queue, and the executing queue.
 */
void lf_print_snapshot() {
    if(LOG_LEVEL > LOG_LEVEL_LOG) {
        LF_PRINT_DEBUG(">>> START Snapshot");
        LF_PRINT_DEBUG("Pending:");
        // pqueue_dump(reaction_q, print_reaction); FIXME: reaction_q is not
        // accessible here
        LF_PRINT_DEBUG("Event queue size: %d. Contents:",
                        pqueue_size(event_q));
        pqueue_dump(event_q, print_reaction); 
        LF_PRINT_DEBUG(">>> END Snapshot");
    }
}

// Array of thread IDs (to be dynamically allocated).
lf_thread_t* _lf_thread_ids;

// Start threads in the thread pool.
void start_threads() {
    LF_PRINT_LOG("Starting %u worker threads.", _lf_number_of_workers);
    _lf_thread_ids = (lf_thread_t*)malloc(_lf_number_of_workers * sizeof(lf_thread_t));
    for (unsigned int i = 0; i < _lf_number_of_workers; i++) {
        lf_thread_create(&_lf_thread_ids[i], worker, NULL);
    }
}

/**
 * @brief Determine the number of workers.
 * 
 */
void determine_number_of_workers(void) {
    // If _lf_number_of_workers is 0, it means that it was not provided on
    // the command-line using the --workers argument.
    if (_lf_number_of_workers == 0u) {
        #if !defined(NUMBER_OF_WORKERS) || NUMBER_OF_WORKERS == 0
        // Use the number of cores on the host machine.
        _lf_number_of_workers = lf_available_cores();

        // If reaction graph breadth is available. Cap number of workers
        #if defined(LF_REACTION_GRAPH_BREADTH)
        if (LF_REACTION_GRAPH_BREADTH < _lf_number_of_workers) {
            _lf_number_of_workers = LF_REACTION_GRAPH_BREADTH;
        }
        #endif

        #else
        // Use the provided number of workers by the user
        _lf_number_of_workers = NUMBER_OF_WORKERS;
        #endif
    }

    #if defined(WORKERS_NEEDED_FOR_FEDERATE)
    // Add the required number of workers needed for the proper function of
    // federated execution
    _lf_number_of_workers += WORKERS_NEEDED_FOR_FEDERATE;
    #endif
}

/**
 * The main loop of the LF program.
 * 
 * An unambiguous function name that can be called
 * by external libraries.
 * 
 * Note: In target languages that use the C core library,
 * there should be an unambiguous way to execute the LF
 * program's main function that will not conflict with
 * other main functions that might get resolved and linked
 * at compile time.
 */
int lf_reactor_c_main(int argc, char* argv[]) {
    // Invoke the function that optionally provides default command-line options.
    _lf_set_default_command_line_options();
    
    // Initialize the one and only mutex to be recursive, meaning that it is OK
    // for the same thread to lock and unlock the mutex even if it already holds
    // the lock.
    // FIXME: This is dangerous. The docs say this: "It is advised that an
    // application should not use a PTHREAD_MUTEX_RECURSIVE mutex with
    // condition variables because the implicit unlock performed for a
    // pthread_cond_wait() or pthread_cond_timedwait() may not actually
    // release the mutex (if it had been locked multiple times).
    // If this happens, no other thread can satisfy the condition
    // of the predicate.”  This seems like a bug in the implementation of
    // pthreads. Maybe it has been fixed?
    // The one and only mutex lock.
    lf_mutex_init(&mutex);

    // Initialize condition variables used for notification between threads.
    lf_cond_init(&event_q_changed);
    lf_cond_init(&global_tag_barrier_requestors_reached_zero);

    if (atexit(termination) != 0) {
        lf_print_warning("Failed to register termination function!");
    }
    // The above handles only "normal" termination (via a call to exit).
    // As a consequence, we need to also trap ctrl-C, which issues a SIGINT,
    // and cause it to call exit.
    signal(SIGINT, exit);
#ifdef SIGPIPE
    // Ignore SIGPIPE errors, which terminate the entire application if
    // socket write() fails because the reader has closed the socket.
    // Instead, cause an EPIPE error to be set when write() fails.
    signal(SIGPIPE, SIG_IGN);
#endif // SIGPIPE

    if (process_args(default_argc, default_argv)
            && process_args(argc, argv)) {
        
        determine_number_of_workers();

        lf_mutex_lock(&mutex);
        initialize(); // Sets start_time

        lf_print("---- Using %d workers.", _lf_number_of_workers);
        
        // Initialize the scheduler
        lf_sched_init(
            (size_t)_lf_number_of_workers, 
            NULL);

        // Call the following function only once, rather than per worker thread (although 
        // it can be probably called in that manner as well).
        _lf_initialize_start_tag();

        start_threads();

        lf_mutex_unlock(&mutex);
        LF_PRINT_DEBUG("Waiting for worker threads to exit.");

        // Wait for the worker threads to exit.
        void* worker_thread_exit_status = NULL;
        LF_PRINT_DEBUG("Number of threads: %d.", _lf_number_of_workers);
        int ret = 0;
        for (int i = 0; i < _lf_number_of_workers; i++) {
        	int failure = lf_thread_join(_lf_thread_ids[i], &worker_thread_exit_status);
        	if (failure) {
        		lf_print_error("Failed to join thread listening for incoming messages: %s", strerror(failure));
        	}
        	if (worker_thread_exit_status != NULL) {
                lf_print_error("---- Worker %d reports error code %p", worker_thread_exit_status);
                ret = 1;
        	}
        }

        if (ret == 0) {
            LF_PRINT_LOG("---- All worker threads exited successfully.");
        }
        
        lf_sched_free();
        free(_lf_thread_ids);
        return ret;
    } else {
        return -1;
    }
}
