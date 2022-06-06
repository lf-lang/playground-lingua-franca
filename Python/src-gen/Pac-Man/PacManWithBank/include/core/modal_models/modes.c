/*************
Copyright (c) 2021, Kiel University.

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
 * Runtime infrastructure for modes in the C target of Lingua Franca.
 * This file contains functions that handle the processing of mode transitions.
 * It works together with other changes across the runtime implementation.
 *
 * Any mode related code will only work in the presence of the MODAL_REACTORS
 * definition.
 *
 * This file is intended for direct include in reactor_common.c.
 *
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 * @author{Soroush Bateni <soroush@utdallas.edu}
 */
#ifdef MODAL_REACTORS

#include "modes.h"
#include "../reactor.h"

// Forward declaration of functions and variables supplied by reactor_common.c
void _lf_trigger_reaction(reaction_t* reaction, int worker_number);
event_t* _lf_create_dummy_events(trigger_t* trigger, instant_t time, event_t* next, microstep_t offset);
extern reaction_t** _lf_startup_reactions;
extern int _lf_startup_reactions_size;
extern pqueue_t* event_q;

/**
 * Return true if the given mode is active.
 * This includes checking all enclosing modes.
 * If any of those is inactive, then so is this one.
 *
 * @param mode The mode instance to check.
 */
bool _lf_mode_is_active(reactor_mode_t* mode) {
    /*
     * This code could be optimized by introducing a cached activity indicator
     * in all mode states. But for now: no premature optimization.
     */
    if (mode != NULL) {
        //LF_PRINT_DEBUG("Checking mode state of %s", mode->name);
        reactor_mode_state_t* state = mode->state;
        while (state != NULL) {
            // If this or any parent mode is inactive, return inactive
            if (state->active_mode != mode) {
                //LF_PRINT_DEBUG(" => Mode is inactive");
                return false;
            }
            mode = state->parent_mode;
            if (mode != NULL) {
                state = mode->state;
            } else {
                state = NULL;
            }
        }
        //LF_PRINT_DEBUG(" => Mode is active");
    }
    return true;
}

/**
 * Trigger startup reactions in modal reactors that should be triggered.
 * A startup reaction should be triggered if it is in an active mode and
 * either it has never been triggered before or it has not been triggered
 * since a reset transition was taken into the mode.
 * Startup reactions are triggered upon first entry and reentry with reset.
 */
void _lf_handle_mode_startup_reactions() {
    // Handle startup reactions in modal reactors
    for (int i = 0; i < _lf_startup_reactions_size; i++) {
        if (_lf_startup_reactions[i]->mode != NULL) {
            if(_lf_startup_reactions[i]->status == inactive
                    && _lf_mode_is_active(_lf_startup_reactions[i]->mode)
                    && _lf_startup_reactions[i]->mode->should_trigger_startup == true
            ) {
                _lf_trigger_reaction(_lf_startup_reactions[i], -1);
            }
        }
    }

    // Reset the should_trigger_startup flag for startup reactions that are
    // going to be triggered in the current tag.
    for (int i = 0; i < _lf_startup_reactions_size; i++) {
        if (_lf_startup_reactions[i]->mode != NULL) {
            if (_lf_mode_is_active(_lf_startup_reactions[i]->mode)) {
                _lf_startup_reactions[i]->mode->should_trigger_startup = false;
            }
        }
    }
}

// Linked list element for suspended events in inactive modes
typedef struct _lf_suspended_event {
    struct _lf_suspended_event* next;
    event_t* event;
} _lf_suspended_event_t;
_lf_suspended_event_t* _lf_suspended_events_head = NULL; // Start of linked collection of suspended events (managed automatically!)
int _lf_suspended_events_num = 0; // Number of suspended events (managed automatically!)
_lf_suspended_event_t* _lf_unsused_suspended_events_head = NULL; // Internal collection of reusable list elements (managed automatically!)

/**
 * Save the given event as suspended.
 */
void _lf_add_suspended_event(event_t* event) {
    _lf_suspended_event_t* new_suspended_event;
    if (_lf_unsused_suspended_events_head != NULL) {
        new_suspended_event = _lf_unsused_suspended_events_head;
        _lf_unsused_suspended_events_head = _lf_unsused_suspended_events_head->next;
    } else {
        new_suspended_event = (_lf_suspended_event_t*) malloc(sizeof(_lf_suspended_event_t));
    }

    new_suspended_event->event = event;
    new_suspended_event->next = _lf_suspended_events_head; // prepend
    _lf_suspended_events_num++;

    _lf_suspended_events_head = new_suspended_event;
}

/**
 * Remove the given node from the list of suspended events and
 * Returns the next element in the list.
 */
_lf_suspended_event_t* _lf_remove_suspended_event(_lf_suspended_event_t* event) {
    _lf_suspended_event_t* next = event->next;

    // Clear content
    event->event = NULL;
    event->next = NULL;
    _lf_suspended_events_num--;

    // Store for recycling
    if (_lf_unsused_suspended_events_head == NULL) {
        _lf_unsused_suspended_events_head = event;
    } else {
        event->next = _lf_unsused_suspended_events_head;
        _lf_unsused_suspended_events_head = event;
    }

    if (_lf_suspended_events_head == event) {
        _lf_suspended_events_head = next; // Adjust head
    } else {
        _lf_suspended_event_t* predecessor = _lf_suspended_events_head;
        while(predecessor->next != event && predecessor != NULL) {
                predecessor = predecessor->next;
        }
        if (predecessor != NULL) {
                predecessor->next = next; // Remove from linked list
        }
    }

    return next;
}

/**
 * Perform transitions in all modal reactors.
 *
 * @param state An array of mode state of modal reactor instance, which must be ordered hierarchically, where
 *      an enclosing mode must come before the inner mode.
 * @param num_states The number of mode state.
 * @param reset_data A list of initial values for reactor state variables.
 * @param reset_data_size
 * @param timer_triggers Array of pointers to timer triggers.
 * @param timer_triggers_size
 */
void _lf_process_mode_changes(
    reactor_mode_state_t* states[],
    int num_states,
    mode_state_variable_reset_data_t reset_data[],
    int reset_data_size,
    trigger_t* timer_triggers[],
    int timer_triggers_size
) {
    bool transition = false; // any mode change in this step

    // Detect mode changes (top down for hierarchical reset)
    for (int i = 0; i < num_states; i++) {
        reactor_mode_state_t* state = states[i];
        if (state != NULL) {
            // Hierarchical reset: if this mode has parent that is entered in
            // this step with a reset this reactor has to enter its initial mode
            if (state->parent_mode != NULL
                    && state->parent_mode->state != NULL
                    && state->parent_mode->state->next_mode == state->parent_mode
                    && state->parent_mode->state->mode_change == reset_transition
            ){
                // Reset to initial state.
                state->next_mode = state->initial_mode;
                // Enter with reset, to cascade it further down.
                state->mode_change = reset_transition;
                LF_PRINT_DEBUG("Modes: Hierarchical mode reset to %s when entering %s.",
                        state->initial_mode->name, state->parent_mode->name);
            }

            // Handle effect of entering next mode
            if (state->next_mode != NULL) {
                LF_PRINT_DEBUG("Modes: Transition to %s.", state->next_mode->name);
                transition = true;

                if (state->mode_change == reset_transition) {
                    // Reset state variables
                    for (int i = 0; i < reset_data_size; i++) {
                        mode_state_variable_reset_data_t data = reset_data[i];
                        if (data.mode == state->next_mode) {
                            LF_PRINT_DEBUG("Modes: Reseting state variable.");
                            memcpy(data.target, data.source, data.size);
                        }
                    }
                }

                // Reset/Reactivate previously suspended events of next state
                _lf_suspended_event_t* suspended_event = _lf_suspended_events_head;
                while(suspended_event != NULL) {
                    event_t* event = suspended_event->event;
                    if (event != NULL && event->trigger != NULL && event->trigger->mode == state->next_mode) {
                        if (state->mode_change == reset_transition) { // Reset transition
                            if (event->trigger->is_timer) { // Only reset timers
                                trigger_t* timer = event->trigger;

                                LF_PRINT_DEBUG("Modes: Re-enqueuing reset timer.");
                                // Reschedule the timer with no additional delay.
                                // This will take care of super dense time when offset is 0.
                                _lf_schedule(timer, event->trigger->offset, NULL);
                            }
                            // No further processing; drops all events upon reset (timer event was recreated by schedule and original can be removed here)
                        } else if (state->next_mode != state->active_mode && event->trigger != NULL) { // History transition to a different mode
                            // Remaining time that the event would have been waiting before mode was left
                            instant_t local_remaining_delay = event->time - (state->next_mode->deactivation_time != 0 ? state->next_mode->deactivation_time : lf_time_start());
                            tag_t current_logical_tag = lf_tag();

                            // Reschedule event with original local delay
                            LF_PRINT_DEBUG("Modes: Re-enqueuing event with a suspended delay of %d (previous TTH: %u, Mode suspended at: %u).", local_remaining_delay, event->time, state->next_mode->deactivation_time);
                            tag_t schedule_tag = {.time = current_logical_tag.time + local_remaining_delay, .microstep = (local_remaining_delay == 0 ? current_logical_tag.microstep + 1 : 0)};
                            _lf_schedule_at_tag(event->trigger, schedule_tag, event->token);

                            if (event->next != NULL) {
                                // The event has more events stacked up in super dense time, attach them to the newly created event.
                                if (event->trigger->last->next == NULL) {
                                    event->trigger->last->next = event->next;
                                } else {
                                    lf_print_error("Modes: Cannot attach events stacked up in super dense to the just unsuspended root event.");
                                }
                            }
                        }
                        // A fresh event was created by schedule, hence, recycle old one
                        _lf_recycle_event(event);

                        // Remove suspended event and continue
                        suspended_event = _lf_remove_suspended_event(suspended_event);
                    } else {
                        suspended_event = suspended_event->next;
                    }
                }

                if (state->mode_change == reset_transition) { // Reset transition
                    // Handle timers that have a period of 0. These timers will only trigger
                    // once and will not be on the event_q after their initial triggering.
                    // Therefore, the logic above cannot handle these timers. We need
                    // to trigger these timers manually if there is a reset transition.
                    for (int i = 0; i < timer_triggers_size; i++) {
                        trigger_t* timer = timer_triggers[i];
                        if (timer->period == 0 && timer->mode == state->next_mode) {
                            _lf_schedule(timer, timer->offset, NULL);
                        }
                    }
                }
            }
        }
    }

    // Handle leaving active mode in all states
    if (transition) {
        bool should_trigger_startup_reactions_at_next_microstep = false;
        // Set new active mode and clear mode change flags
        for (int i = 0; i < num_states; i++) {
            reactor_mode_state_t* state = states[i];
            if (state != NULL && state->next_mode != NULL) {
                // Save time when mode was left to handle suspended events in the future
                state->active_mode->deactivation_time = lf_time_logical();

                // Apply transition
                state->active_mode = state->next_mode;
                if (state->mode_change == reset_transition
                        || state->active_mode->should_trigger_startup) {
                    state->active_mode->should_trigger_startup = true;
                    should_trigger_startup_reactions_at_next_microstep = true;
                }

                state->next_mode = NULL;
                state->mode_change = no_transition;
            }
        }

        // Retract all events from the event queue that are associated with now inactive modes
        if (event_q != NULL) {
            size_t q_size = pqueue_size(event_q);
            if (q_size > 0) {
                event_t** delayed_removal = (event_t**) calloc(q_size, sizeof(event_t*));
                size_t delayed_removal_count = 0;

                // Find events
                for (int i = 0; i < q_size; i++) {
                    event_t* event = (event_t*)event_q->d[i + 1]; // internal queue data structure omits index 0
                    if (event != NULL && event->trigger != NULL && !_lf_mode_is_active(event->trigger->mode)) {
                        delayed_removal[delayed_removal_count++] = event;
                        // This will store the event including possibly those chained up in super dense time
                        _lf_add_suspended_event(event);
                    }
                }

                // Events are removed delayed in order to allow linear iteration over the queue
                LF_PRINT_DEBUG("Modes: Pulling %d events from the event queue to suspend them. %d events are now suspended.", delayed_removal_count, _lf_suspended_events_num);
                for (int i = 0; i < delayed_removal_count; i++) {
                    pqueue_remove(event_q, delayed_removal[i]);
                }

                free(delayed_removal);
            }
        }

        if (should_trigger_startup_reactions_at_next_microstep) {
            // Insert a dummy event in the event queue for the next microstep to make
            // sure startup reactions (if any) can be triggered as soon as possible.
            pqueue_insert(event_q, _lf_create_dummy_events(NULL, current_tag.time, NULL, 1));
        }
    }
}

/**
 * Release internal data structures for modes.
 * - Frees all suspended events.
 */
void _lf_terminate_modal_reactors() {
    _lf_suspended_event_t* suspended_event = _lf_suspended_events_head;
    while(suspended_event != NULL) {
        _lf_recycle_event(suspended_event->event);
        _lf_suspended_event_t* next = suspended_event->next;
        free(suspended_event);
        suspended_event = next;
    }
    _lf_suspended_events_head = NULL;
    _lf_suspended_events_num = 0;

    // Also free suspended_event elements stored for recycling
    suspended_event = _lf_unsused_suspended_events_head;
    while(suspended_event != NULL) {
        _lf_suspended_event_t* next = suspended_event->next;
        free(suspended_event);
        suspended_event = next;
    }
    _lf_unsused_suspended_events_head = NULL;
}
#endif
