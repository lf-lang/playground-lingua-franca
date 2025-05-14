/**
 * @file
 * @author Edward A. Lee
 * @author Hou Seng (Steven) Wong
 * @author Soroush Bateni
 * @copyright (c) 2020-2024, The University of California at Berkeley.
 * License: <a href="https://github.com/lf-lang/reactor-c/blob/main/LICENSE.md">BSD 2-clause</a>
 *
 * @brief API functions for scheduling actions.
 *
 * Most of these functions take a `void*` pointer to an action, which will be internally cast to
 * a `lf_action_base_t*` pointer. The cast could be done by macros in reaction_macros.h, but unlike
 * the macros defined there, it is common for `lf_schedule` functions to be invoked outside of reaction
 * bodies.  This means that users writing code in separate library files are responsible for ensuring that
 * the `void*` pointer is indeed a valid `lf_action_base_t*` pointer before passing it to `lf_schedule`.
 * The compiler will not check this.
 */

#ifndef API_H
#define API_H

#include "lf_types.h"
#include "tag.h"

/**
 * @brief Schedule an action to occur with the specified time offset with no payload (no value conveyed).
 *
 * The later tag will depend on whether the action is logical or physical. If it is logical,
 * the time of the event will be the current logical time of the environment associated with
 * the action plus the minimum delay of the action plus the extra delay. If that time is equal
 * to the current time, then the tag will be one microstep beyond the current tag.
 * If the action is physical, the time will be the current physical time plus the extra delay,
 * and the microstep will be zero.
 *
 * See lf_schedule_token(), which this uses, for details.
 *
 * @param action The action to be triggered (a pointer to an `lf_action_base_t`).
 * @param offset The time offset over and above the minimum delay of the action.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
trigger_handle_t lf_schedule(void* action, interval_t offset);

/**
 * @brief Schedule the specified action with an integer value at a later logical time.
 *
 * The later tag will depend on whether the action is logical or physical. If it is logical,
 * the time of the event will be the current logical time of the environment associated with
 * the action plus the minimum delay of the action plus the extra delay. If that time is equal
 * to the current time, then the tag will be one microstep beyond the current tag.
 * If the action is physical, the time will be the current physical time plus the extra delay,
 * and the microstep will be zero.
 *
 * This wraps a copy of the integer value in a token. See lf_schedule_token() for more details.
 *
 * @param action The action to be triggered (a pointer to an `lf_action_base_t`).
 * @param extra_delay Extra offset of the event release above that in the action.
 * @param value The value to send.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
trigger_handle_t lf_schedule_int(void* action, interval_t extra_delay, int value);

/**
 * @brief Schedule the specified action at a later tag with the specified token as a payload.
 *
 * The later tag will depend on whether the action is logical or physical. If it is logical,
 * the time of the event will be the current logical time of the environment associated with
 * the action plus the minimum delay of the action plus the extra delay. If that time is equal
 * to the current time, then the tag will be one microstep beyond the current tag.
 * If the action is physical, the time will be the current physical time plus the extra delay,
 * and the microstep will be zero.
 *
 * For a logical action:
 *
 * A logical action has a minimum delay (default is zero) and a minimum spacing, which also
 * defaults to zero. The logical time at which this scheduled event will trigger is the current time
 * of the environment associated with the action plus the offset plus the delay argument given to
 * this function. If, however, that time is not greater than a prior
 * triggering of this logical action by at least the minimum spacing, then the
 * one of two things can happen depending on the policy specified
 * for the action. If the action's policy is DROP (default), then the
 * action is simply dropped and the memory pointed to by value argument
 * is freed. If the policy is DEFER, then the time will be increased
 * to equal the time of the most recent triggering plus the minimum spacing.
 *
 * For the above, "current time" means the logical time of the
 * reaction that is calling this function. Logical actions should
 * always be scheduled within a reaction invocation, never asynchronously
 * from the outside. FIXME: This needs to be checked.
 *
 * physical action: A physical action has all the same parameters
 * as a logical action, but its timestamp will be the larger of the
 * current physical time and the time it would be assigned if it
 * were a logical action.
 *
 * There are three conditions under which this function will not
 * actually put an event on the event queue and decrement the reference count
 * of the token (if there is one), which could result in the payload being
 * freed. In all three cases, this function returns 0. Otherwise,
 * it returns a handle to the scheduled trigger, which is an integer
 * greater than 0.
 *
 * The first condition is that stop() has been called and the time offset
 * of this event is greater than zero.
 * The second condition is that the logical time of the event
 * is greater that the stop time (timeout) that is specified in the target
 * properties or on the command line.
 * The third condition is that the trigger argument is null.
 *
 * @param action The action to be triggered (a pointer to an `lf_action_base_t`).
 * @param extra_delay Extra offset of the event release above that in the action.
 * @param token The token to carry the payload or null for no payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
trigger_handle_t lf_schedule_token(void* action, interval_t extra_delay, lf_token_t* token);

/**
 * @brief Schedule an action to occur with the specified value and time offset with a
 * copy of the specified value.
 *
 * If the value is non-null, then it will be copied
 * into newly allocated memory under the assumption that its size is given in
 * the trigger's token object's element_size field multiplied by the specified
 * length.
 *
 * The later tag will depend on whether the action is logical or physical. If it is logical,
 * the time of the event will be the current logical time of the environment associated with
 * the action plus the minimum delay of the action plus the extra delay. If that time is equal
 * to the current time, then the tag will be one microstep beyond the current tag.
 * If the action is physical, the time will be the current physical time plus the extra delay,
 * and the microstep will be zero.
 *
 * See lf_schedule_token(), which this uses, for details.
 *
 * @param action The action to be triggered (a pointer to an `lf_action_base_t`).
 * @param offset The time offset over and above that in the action.
 * @param value A pointer to the value to copy.
 * @param length The length, if an array, 1 if a scalar, and 0 if value is NULL.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for
 *  error.
 */
trigger_handle_t lf_schedule_copy(void* action, interval_t offset, void* value, size_t length);

/**
 * @brief Variant of lf_schedule_token that creates a token to carry the specified value.
 *
 * The value is required to be malloc'd memory with a size equal to the
 * element_size of the specified action times the length parameter.
 *
 * See lf_schedule_token(), which this uses, for details.
 *
 * @param action The action to be triggered (a pointer to an `lf_action_base_t`).
 * @param extra_delay Extra offset of the event release above that in the
 *  action.
 * @param value Dynamically allocated memory containing the value to send.
 * @param length The length of the array, if it is an array, or 1 for a scalar
 *  and 0 for no payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for
 *  error.
 */
trigger_handle_t lf_schedule_value(void* action, interval_t extra_delay, void* value, int length);

/**
 * @brief Schedule the specified trigger to execute in the specified environment with given delay and token.
 *
 * This is the most flexible version of the schedule functions and is used in the implementation
 * of many of the others. End users would rarely use it.
 *
 * This will schedule the specified trigger at env->current_tag.time plus the offset of the
 * specified trigger plus the delay. The value is required to be either
 * NULL or a pointer to a token wrapping the payload. The token carries
 * a reference count, and when the reference count decrements to 0,
 * the will be freed. Hence, it is essential that the payload be in
 * memory allocated using malloc.
 *
 * There are several conditions under which this function will not
 * actually put an event on the event queue and decrement the reference count
 * of the token (if there is one), which could result in the payload being
 * freed. In all cases, this function returns 0. Otherwise,
 * it returns a handle to the scheduled trigger, which is an integer
 * greater than 0.
 *
 * The first condition is that a stop has been requested and the trigger
 * offset plus the extra delay is greater than zero.
 * The second condition is that the trigger offset plus the extra delay
 * is greater that the requested stop time (timeout).
 * A third condition is that the trigger argument is null.
 * Also, an event might not be scheduled if the trigger is an action
 * with a `min_spacing` parameter.  See the documentation.

 * @param env The environment in which to schedule the event.
 * @param trigger The action or timer to be triggered.
 * @param delay Offset of the event release.
 * @param token The token payload.
 * @return A handle to the event, or 0 if no event was scheduled, or -1 for error.
 */
trigger_handle_t lf_schedule_trigger(environment_t* env, trigger_t* trigger, interval_t delay, lf_token_t* token);

/**
 * @brief Check the deadline of the currently executing reaction against the
 * current physical time.
 *
 * If the deadline has passed, invoke the deadline
 * handler (if invoke_deadline_handler parameter is set true) and return true.
 * Otherwise, return false.
 *
 * This function is intended to be used within a reaction that has been invoked without a deadline
 * violation, but that wishes to check whether the deadline gets violated _during_ the execution of
 * the reaction. This can be used, for example, to implement a timeout mechanism that bounds the
 * execution time of a reaction, for example to realize an "anytime" computation.
 *
 * @param self The self struct of the reactor.
 * @param invoke_deadline_handler When this is set true, also invoke deadline
 *  handler if the deadline has passed.
 * @return True if the specified deadline has passed and false otherwise.
 */
bool lf_check_deadline(void* self, bool invoke_deadline_handler);

#endif // API_H
