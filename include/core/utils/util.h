/**
 * @file
 * @author Edward A. Lee
 * @author Soroush Bateni
 *
 * @section LICENSE
Copyright (c) 2020, The University of California at Berkeley.

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

 * @section DESCRIPTION
 * Header file for utility types and functions for Lingua Franca programs.
 */

#ifndef UTIL_H
#define UTIL_H

#include <stdarg.h> // Defines va_list
#include <stdbool.h>
#include <stdint.h> // Defines int64_t

#include "logging_macros.h"

/**
 * Holds generic statistical data
 */
typedef struct lf_stat_ll {
  int64_t average;
  int64_t standard_deviation;
  int64_t variance;
  int64_t max;
} lf_stat_ll;

/**
 * A handy macro that can concatenate three strings.
 * Useful in the LF_PRINT_DEBUG macro and lf_print_error
 * functions that want to concatenate a "DEBUG: " or
 * "ERROR: " to the beginning of the message and a
 * new line format \n at the end.
 */
#define CONCATENATE_THREE_STRINGS(__string1, __string2, __string3) __string1 __string2 __string3

/**
 * Macro for extracting the level from the index of a reaction.
 * A reaction that has no upstream reactions has level 0.
 * Other reactions have a level that is the length of the longest
 * upstream chain to a reaction with level 0 (inclusive).
 * This is used, along with the deadline, to sort reactions
 * in the reaction queue. It ensures that reactions that are
 * upstream in the dependence graph execute before reactions
 * that are downstream.
 */
#define LF_LEVEL(index) (index & 0xffffLL)

/** Utility for finding the maximum of two values. */
#ifndef LF_MAX
#define LF_MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif

/** Utility for finding the minimum of two values. */
#ifndef LF_MIN
#define LF_MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#endif

/**
 * The ID of this federate. For a non-federated execution, this will
 * be -1.  For a federated execution, it will be assigned when the generated function
 * _lf_initialize_trigger_objects() is called.
 * @see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CGenerator.xtend.
 */
extern uint16_t _lf_my_fed_id;

/**
 * Return the federate ID or -1 if this program is not part of a federation.
 */
uint16_t lf_fed_id(void);

/**
 * varargs alternative of "lf_print"
 */
void lf_vprint(const char* format, va_list args) ATTRIBUTE_FORMAT_PRINTF(1, 0);

/**
 * varargs alternative of "lf_print_log"
 */
void lf_vprint_log(const char* format, va_list args) ATTRIBUTE_FORMAT_PRINTF(1, 0);

/**
 * varargs alternative of "lf_print_debug"
 */
void lf_vprint_debug(const char* format, va_list args) ATTRIBUTE_FORMAT_PRINTF(1, 0);

/**
 * Print the error defined by the errno variable with the
 * specified message as a prefix, then exit with error code 1.
 * @param msg The prefix to the message.
 */
void error(const char* msg);

/**
 * varargs alternative of "lf_print_error"
 */
void lf_vprint_error(const char* format, va_list args) ATTRIBUTE_FORMAT_PRINTF(1, 0);

/**
 * varargs alternative of "lf_print_warning"
 */
void lf_vprint_warning(const char* format, va_list args) ATTRIBUTE_FORMAT_PRINTF(1, 0);

/**
 * varargs alternative of "lf_print_error_and_exit"
 */
void lf_vprint_error_and_exit(const char* format, va_list args) ATTRIBUTE_FORMAT_PRINTF(1, 0);

/**
 * Initialize mutex with error checking.
 * This is optimized away if the NDEBUG flag is defined.
 * @param mutex Pointer to the mutex to initialize.
 */
#define LF_MUTEX_INIT(mutex) LF_ASSERTN(lf_mutex_init(mutex), "Mutex init failed.")

/**
 * Lock mutex with error checking.
 * This is optimized away if the NDEBUG flag is defined.
 * @param mutex Pointer to the mutex to lock.
 */
#define LF_MUTEX_LOCK(mutex) LF_ASSERTN(lf_mutex_lock(mutex), "Mutex lock failed.")

/**
 * Unlock mutex with error checking.
 * This is optimized away if the NDEBUG flag is defined.
 * @param mutex Pointer to the mutex to unlock.
 */
#define LF_MUTEX_UNLOCK(mutex) LF_ASSERTN(lf_mutex_unlock(mutex), "Mutex unlock failed.")

/**
 * Initialize condition variable with error checking.
 * This is optimized away if the NDEBUG flag is defined.
 * @param cond Pointer to the condition variable to initialize.
 * @param mutex Pointer to the mutex to associate with the condition variable.
 */
#define LF_COND_INIT(cond, mutex) LF_ASSERTN(lf_cond_init(cond, mutex), "Condition variable init failed.")

/**
 * Signal a condition variable with error checking.
 * This is optimized away if the NDEBUG flag is defined.
 * @param cond Pointer to the condition variable.
 */
#define LF_COND_SIGNAL(cond) LF_ASSERTN(lf_cond_signal(cond), "Condition variable signal failed.")

/**
 * Broadcast a condition variable with error checking.
 * This is optimized away if the NDEBUG flag is defined.
 * @param cond Pointer to the condition variable.
 */
#define LF_COND_BROADCAST(cond) LF_ASSERTN(lf_cond_broadcast(cond), "Condition variable broadcast failed.")

/**
 * Wait on a condition variable with error checking.
 * This is optimized away if the NDEBUG flag is defined.
 * @param cond Pointer to the condition variable.
 */
#define LF_COND_WAIT(cond) LF_ASSERTN(lf_cond_wait(cond), "Condition variable wait failed.")

/**
 * Enter critical section with error checking.
 * This is optimized away if the NDEBUG flag is defined.
 * @param env Pointer to the environment.
 */
#define LF_CRITICAL_SECTION_ENTER(env) LF_ASSERT(!lf_critical_section_enter(env), "Could not enter critical section")

/**
 * Exit critical section with error checking.
 * This is optimized away if the NDEBUG flag is defined.
 * @param env Pointer to the environment.
 */
#define LF_CRITICAL_SECTION_EXIT(env) LF_ASSERT(!lf_critical_section_exit(env), "Could not exit critical section")

#endif /* UTIL_H */
