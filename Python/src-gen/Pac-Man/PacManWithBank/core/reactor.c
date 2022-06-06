/* Runtime infrastructure for the non-threaded version of the C target of Lingua Franca. */

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

/** Runtime infrastructure for the non-threaded version of the C target
 *  of Lingua Franca.
 *  
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Soroush Bateni <soroush@utdallas.edu>}
 */

#include "reactor_common.c"
#include "platform.h"
#include <signal.h> // To trap ctrl-c and invoke termination().
//#include <assert.h>

/**
 * @brief Queue of triggered reactions at the current tag.
 * 
 */
pqueue_t* reaction_q;

/**
 * Unless the "fast" option is given, an LF program will wait until
 * physical time matches logical time before handling an event with
 * a given logical time. The amount of time is less than this given
 * threshold, then no wait will occur. The purpose of this is
 * to prevent unnecessary delays caused by simply setting up and
 * performing the wait.
 */
#define MIN_WAIT_TIME NSEC(10)

/**
 * Schedule the specified trigger at current_tag.time plus the offset of the
 * specified trigger plus the delay.
 * See reactor.h for documentation.
 */
trigger_handle_t _lf_schedule_token(void* action, interval_t extra_delay, lf_token_t* token) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    return _lf_schedule(trigger, extra_delay, token);
}

/**
 * Variant of schedule_token that creates a token to carry the specified value.
 * See reactor.h for documentation.
 */
trigger_handle_t _lf_schedule_value(void* action, interval_t extra_delay, void* value, size_t length) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    lf_token_t* token = create_token(trigger->element_size);
    token->value = value;
    token->length = length;
    return _lf_schedule_token(action, extra_delay, token);
}

/**
 * Schedule an action to occur with the specified value and time offset
 * with a copy of the specified value.
 * See reactor.h for documentation.
 */
trigger_handle_t _lf_schedule_copy(void* action, interval_t offset, void* value, size_t length) {
    trigger_t* trigger = _lf_action_to_trigger(action);
    if (value == NULL) {
        return _lf_schedule_token(action, offset, NULL);
    }
    if (trigger == NULL || trigger->token == NULL || trigger->token->element_size <= 0) {
        lf_print_error("schedule: Invalid trigger or element size.");
        return -1;
    }
    LF_PRINT_DEBUG("schedule_copy: Allocating memory for payload (token value): %p.", trigger);
    // Initialize token with an array size of length and a reference count of 0.
    lf_token_t* token = _lf_initialize_token(trigger->token, length);
    // Copy the value into the newly allocated memory.
    memcpy(token->value, value, token->element_size * length);
    // The schedule function will increment the reference count.
    return _lf_schedule_token(action, offset, token);
}

/**
 * Mark the given is_present field as true. This is_present field
 * will later be cleaned up by _lf_start_time_step.
 * This assumes that the mutex is not held.
 * @param is_present_field A pointer to the is_present field that
 * must be set.
 */
void _lf_set_present(bool* is_present_field) {
    if (_lf_is_present_fields_abbreviated_size < _lf_is_present_fields_size) {
        _lf_is_present_fields_abbreviated[_lf_is_present_fields_abbreviated_size]
            = is_present_field;
    }
    _lf_is_present_fields_abbreviated_size++;
    *is_present_field = true;
}

/**
 * Advance logical time to the lesser of the specified time or the
 * timeout time, if a timeout time has been given. If the -fast command-line option
 * was not given, then wait until physical time matches or exceeds the start time of
 * execution plus the current_tag.time plus the specified logical time.  If this is not
 * interrupted, then advance current_tag.time by the specified logical_delay.
 * Return 0 if time advanced to the time of the event and -1 if the wait
 * was interrupted or if the timeout time was reached.
 */ 
int wait_until(instant_t logical_time_ns) {
    int return_value = 0;
    if (!fast) {
        LF_PRINT_LOG("Waiting for elapsed logical time %lld.", logical_time_ns - start_time);
        interval_t ns_to_wait = logical_time_ns - lf_time_physical();
    
        if (ns_to_wait < MIN_WAIT_TIME) {
            LF_PRINT_DEBUG("Wait time %lld is less than MIN_WAIT_TIME %lld. Skipping wait.",
                ns_to_wait, MIN_WAIT_TIME);
            return return_value;
        }

        return_value = lf_nanosleep(ns_to_wait);
    }
    return return_value;
}

void print_snapshot() {
    if(LOG_LEVEL > LOG_LEVEL_LOG) {
        LF_PRINT_DEBUG(">>> START Snapshot");
        pqueue_dump(reaction_q, reaction_q->prt);
        LF_PRINT_DEBUG(">>> END Snapshot");
    }
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
    if (!_lf_mode_is_active(reaction->mode)) {
        LF_PRINT_DEBUG("Suppressing downstream reaction %s due inactivity of mode %s.", reaction->name, reaction->mode->name);
        return; // Suppress reaction by preventing entering reaction queue
    }
#endif
    // Do not enqueue this reaction twice.
    if (reaction->status == inactive) {
        LF_PRINT_DEBUG("Enqueing downstream reaction %s, which has level %lld.",
        		reaction->name, reaction->index & 0xffffLL);
        reaction->status = queued;
        pqueue_insert(reaction_q, reaction);
    }
}

/**
 * Execute all the reactions in the reaction queue at the current tag.
 * 
 * @return Returns 1 if the execution should continue and 0 if the execution
 *  should stop.
 */
int _lf_do_step(void) {
    // Invoke reactions.
    while(pqueue_size(reaction_q) > 0) {
        // print_snapshot();
        reaction_t* reaction = (reaction_t*)pqueue_pop(reaction_q);
        reaction->status = running;
        
        LF_PRINT_LOG("Invoking reaction %s at elapsed logical tag (%lld, %d).",
        		reaction->name,
                current_tag.time - start_time, current_tag.microstep);

        bool violation = false;

        // FIXME: These comments look outdated. We may need to update them.
        // If the reaction has a deadline, compare to current physical time
        // and invoke the deadline violation reaction instead of the reaction function
        // if a violation has occurred. Note that the violation reaction will be invoked
        // at most once per logical time value. If the violation reaction triggers the
        // same reaction at the current time value, even if at a future superdense time,
        // then the reaction will be invoked and the violation reaction will not be invoked again.
        if (reaction->deadline > 0LL) {
            // Get the current physical time.
            instant_t physical_time = lf_time_physical();
            // FIXME: These comments look outdated. We may need to update them.
            // Check for deadline violation.
            // There are currently two distinct deadline mechanisms:
            // local deadlines are defined with the reaction;
            // container deadlines are defined in the container.
            // They can have different deadlines, so we have to check both.
            // Handle the local deadline first.
            if (physical_time > current_tag.time + reaction->deadline) {
                LF_PRINT_LOG("Deadline violation. Invoking deadline handler.");
                // Deadline violation has occurred.
                violation = true;
                // Invoke the local handler, if there is one.
                reaction_function_t handler = reaction->deadline_violation_handler;
                if (handler != NULL) {
                    (*handler)(reaction->self);
                    // If the reaction produced outputs, put the resulting
                    // triggered reactions into the queue.
                    schedule_output_reactions(reaction, 0);
                }
            }
        }
        
        if (!violation) {
            // Invoke the reaction function.
            _lf_invoke_reaction(reaction, 0);   // 0 indicates unthreaded.

            // If the reaction produced outputs, put the resulting triggered
            // reactions into the queue.
            schedule_output_reactions(reaction, 0);
        }
        // There cannot be any subsequent events that trigger this reaction at the
        //  current tag, so it is safe to conclude that it is now inactive.
        reaction->status = inactive;
    }
    
#ifdef MODAL_REACTORS
    // At the end of the step, perform mode transitions
    _lf_handle_mode_changes();
#endif

    // No more reactions should be blocked at this point.
    //assert(pqueue_size(blocked_q) == 0);

    if (lf_tag_compare(current_tag, stop_tag) >= 0) {
        return 0;
    }

    return 1;
}

// Wait until physical time matches or exceeds the time of the least tag
// on the event queue. If there is no event in the queue, return 0.
// After this wait, advance current_tag.time to match
// this tag. Then pop the next event(s) from the
// event queue that all have the same tag, and extract from those events
// the reactions that are to be invoked at this logical time.
// Sort those reactions by index (determined by a topological sort)
// and then execute the reactions in order. Each reaction may produce
// outputs, which places additional reactions into the index-ordered
// priority queue. All of those will also be executed in order of indices.
// If the -timeout option has been given on the command line, then return
// 0 when the logical time duration matches the specified duration.
// Also return 0 if there are no more events in the queue and
// the keepalive command-line option has not been given.
// Otherwise, return 1.
int next(void) {
    event_t* event = (event_t*)pqueue_peek(event_q);
    //pqueue_dump(event_q, event_q->prt);
    // If there is no next event and -keepalive has been specified
    // on the command line, then we will wait the maximum time possible.
    // FIXME: is LLONG_MAX different from FOREVER?
    tag_t next_tag = { .time = LLONG_MAX, .microstep = UINT_MAX};
    if (event == NULL) {
        // No event in the queue.
        if (!keepalive_specified) { // FIXME: validator should issue a warning for unthreaded implementation
                                    // schedule is not thread-safe
            _lf_set_stop_tag((tag_t){.time=current_tag.time,.microstep=current_tag.microstep+1});
        }
    } else {
        next_tag.time = event->time;
        // Deduce the microstep
        if (next_tag.time == current_tag.time) {
            next_tag.microstep = lf_tag().microstep + 1;
        } else {
            next_tag.microstep = 0;
        }
    }
    
    if (_lf_is_tag_after_stop_tag(next_tag)) {
        // Cannot process events after the stop tag.
        next_tag = stop_tag;
    }

    LF_PRINT_LOG("Next event (elapsed) time is %lld.", next_tag.time - start_time);
    // Wait until physical time >= event.time.
    // The wait_until function will advance current_tag.time.
    if (wait_until(next_tag.time) != 0) {
        LF_PRINT_DEBUG("***** wait_until was interrupted.");
        // Sleep was interrupted.
        // FIXME: It is unclear what would cause this to occur in this unthreaded
        // runtime since lf_schedule() is not thread safe here and should not
        // be called asynchronously. Perhaps in some runtime such as for a
        // PRET machine this will be supported, so here we handle this as
        // if an asynchronous call to schedule has occurred. In that case,
        // we should return 1 to let the runtime loop around to see what
        // is on the event queue.
        return 1;
    }

    // At this point, finally, we have an event to process.
    // Advance current time to match that of the first event on the queue.
    _lf_advance_logical_time(next_tag.time);

    if (lf_tag_compare(current_tag, stop_tag) >= 0) {        
        _lf_trigger_shutdown_reactions();
    }

    // Invoke code that must execute before starting a new logical time round,
    // such as initializing outputs to be absent.
    _lf_start_time_step();
    
    // Pop all events from event_q with timestamp equal to current_tag.time,
    // extract all the reactions triggered by these events, and
    // stick them into the reaction queue.
    _lf_pop_events();

    return _lf_do_step();
}

/**
 * Stop execution at the conclusion of the next microstep.
 */
void request_stop() {
	tag_t new_stop_tag;
	new_stop_tag.time = current_tag.time;
	new_stop_tag.microstep = current_tag.microstep + 1;
	_lf_set_stop_tag(new_stop_tag);
}

/**
 * Return false.
 * @param reaction The reaction.
 */
bool _lf_is_blocked_by_executing_reaction(void) {
    return false;
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

    LF_PRINT_DEBUG("Processing command line arguments.");
    if (process_args(default_argc, default_argv)
            && process_args(argc, argv)) {
        LF_PRINT_DEBUG("Processed command line arguments.");
        LF_PRINT_DEBUG("Registering the termination function.");
        if (atexit(termination) != 0) {
            lf_print_warning("Failed to register termination function!");
        }
        // The above handles only "normal" termination (via a call to exit).
        // As a consequence, we need to also trap ctrl-C, which issues a SIGINT,
        // and cause it to call exit.
        signal(SIGINT, exit);

        LF_PRINT_DEBUG("Initializing.");
        initialize(); // Sets start_time.

        // Reaction queue ordered first by deadline, then by level.
        // The index of the reaction holds the deadline in the 48 most significant bits,
        // the level in the 16 least significant bits.
        reaction_q = pqueue_init(INITIAL_REACT_QUEUE_SIZE, in_reverse_order, get_reaction_index,
                get_reaction_position, set_reaction_position, reaction_matches, print_reaction);
                
        current_tag = (tag_t){.time = start_time, .microstep = 0u};
        _lf_execution_started = true;
        _lf_trigger_startup_reactions();
        _lf_initialize_timers(); 
        // If the stop_tag is (0,0), also insert the shutdown
        // reactions. This can only happen if the timeout time
        // was set to 0.
        if (lf_tag_compare(current_tag, stop_tag) >= 0) {
            _lf_trigger_shutdown_reactions(); // _lf_trigger_shutdown_reactions();
        }
        LF_PRINT_DEBUG("Running the program's main loop.");
        // Handle reactions triggered at time (T,m).
        if (_lf_do_step()) {
            while (next() != 0);
        }
        // pqueue_free(reaction_q); FIXME: This might be causing weird memory errors
        return 0;
    } else {
        return -1;
    }
}
