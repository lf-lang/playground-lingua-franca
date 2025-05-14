/**
 * @file
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @author Chris Gill
 * @author Mehrdad Niknami
 * @copyright (c) 2020-2024, The University of California at Berkeley.
 * License: <a href="https://github.com/lf-lang/reactor-c/blob/main/LICENSE.md">BSD 2-clause</a>
 * @brief Definitions for the C target of Lingua Franca shared by threaded and unthreaded versions.
 *
 * This header file defines functions that programmers use in the body of reactions for reading and
 * writing inputs and outputs and scheduling future events. Other functions that might be useful to
 * application programmers are also defined here.
 *
 * Many of these functions have macro wrappers defined in reaction_macros.h.
 */

#ifndef REACTOR_H
#define REACTOR_H

#include "lf_types.h"
#include "modes.h" // Modal model support
#include "port.h"
#include "tag.h"   // Time-related functions.
#include "clock.h" // Time-related functions.
#include "tracepoint.h"
#include "util.h"

/**
 * @brief Macro to suppress warnings about unused variables.
 */
#define SUPPRESS_UNUSED_WARNING(x) (void)(x)

//////////////////////  Function Declarations  //////////////////////

/**
 * @brief Return true if the provided tag is after stop tag.
 * @param env Environment in which we are executing.
 * @param tag The tag to check against stop tag
 */
bool lf_is_tag_after_stop_tag(environment_t* env, tag_t tag);

/**
 * @brief Mark the given port's is_present field as true.
 * @param port A pointer to the port struct as an `lf_port_base_t*`.
 */
void lf_set_present(lf_port_base_t* port);

/**
 * @brief Set the stop tag if it is less than the stop tag of the specified environment.
 * @note In threaded programs, the environment's mutex must be locked before calling this function.
 */
void lf_set_stop_tag(environment_t* env, tag_t tag);

#ifdef FEDERATED_DECENTRALIZED

/**
 * @brief Return the global STP offset on advancement of logical time for federated execution.
 */
interval_t lf_get_stp_offset(void);

/**
 * @brief Set the global STP offset on advancement of logical time for federated execution.
 * @param offset A positive time value to be applied as the STP offset.
 */
void lf_set_stp_offset(interval_t offset);

#endif // FEDERATED_DECENTRALIZED

/**
 * @brief Print a snapshot of the priority queues used during execution (for debugging).
 *
 * This function implementation will be empty if the NDEBUG macro is defined; that macro
 * is normally defined for release builds.
 * @param env The environment in which we are executing.
 */
void lf_print_snapshot(environment_t* env);

/**
 * @brief Request a stop to execution as soon as possible.
 *
 * In a non-federated execution with only a single enclave, this will occur
 * one microstep later than the current tag. In a federated execution or when
 * there is more than one enclave, it will likely occur at a later tag determined
 * by the RTI so that all federates and enclaves stop at the same tag.
 */
void lf_request_stop(void);

/**
 * @brief Allocate memory and record on the specified allocation record (a self struct).
 *
 * This will allocate memory using calloc (so the allocated memory is zeroed out)
 * and record the allocated memory on the specified self struct so that
 * it will be freed when calling {@link free_reactor(self_base_t)}.
 *
 * @param count The number of items of size 'size' to accomodate.
 * @param size The size of each item.
 * @param head Pointer to the head of a list on which to record
 *  the allocation, or NULL to not record it (an `allocation_record_t**`),
 * @return A pointer to the allocated memory.
 */
void* lf_allocate(size_t count, size_t size, struct allocation_record_t** head);

/**
 * @brief Allocate memory for a new runtime instance of a reactor.
 *
 * This records the reactor on the list of reactors to be freed at
 * termination of the program. If you plan to free the reactor before
 * termination of the program, use
 * {@link lf_allocate(size_t, size_t, allocation_record_t**)}
 * with a null last argument instead.
 *
 * @param size The size of the self struct, obtained with sizeof().
 */
self_base_t* lf_new_reactor(size_t size);

/**
 * @brief Free all the reactors that are allocated with {@link #lf_new_reactor(size_t)}.
 */
void lf_free_all_reactors(void);

/**
 * @brief Free the specified reactor.
 *
 * This will free the memory recorded on the allocations list of the specified reactor
 * and then free the specified self struct.
 * @param self The self struct of the reactor.
 */
void lf_free_reactor(self_base_t* self);

/**
 * @brief Return the instance name of the reactor.
 *
 * The instance name is the name of given to the instance created by the `new` operator in LF.
 * If the instance is in a bank, then the name will have a suffix of the form `[bank_index]`.
 *
 * @param self The self struct of the reactor.
 */
const char* lf_reactor_name(self_base_t* self);

/**
 * @brief Return the full name of the reactor.
 *
 * The fully qualified name of a reactor is the instance name of the reactor concatenated with the names of all
 * of its parents, separated by dots. If the reactor or any of its parents is a bank, then the name
 * will have a suffix of the form `[bank_index]`.
 *
 * @param self The self struct of the reactor.
 */
const char* lf_reactor_full_name(self_base_t* self);

#endif /* REACTOR_H */
/** @} */
