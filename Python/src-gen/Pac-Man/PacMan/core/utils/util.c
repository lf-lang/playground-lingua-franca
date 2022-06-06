/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
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
 * Utility functions for managing output the user, error and warning
 * messages, logging, and debug messages. Outputs are filtered based on
 * whether a
 */

#include "util.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>     // Defines memcpy()
#include <stdarg.h>     // Defines va_list
#include <time.h>       // Defines nanosleep()

#ifndef NUMBER_OF_FEDERATES
#define NUMBER_OF_FEDERATES 1
#endif

/** Number of nanoseconds to sleep before retrying a socket read. */
#define SOCKET_READ_RETRY_INTERVAL 1000000

/**
 * The ID of this federate. For a non-federated execution, this will
 * be -1.  For a federated execution, it will be assigned when the generated function
 * _lf_initialize_trigger_objects() is called.
 * @see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CGenerator.xtend.
 */
int _lf_my_fed_id = -1;

/**
 * If non-null, this function will be used instead of the printf to
 * print messages.
 */
print_message_function_t* print_message_function = NULL;

/** The level of messages to redirect to print_message_function. */
int print_message_level = -1;

/**
 * Return the federate ID or -1 if this program is not part of a federation.
 */
int lf_fed_id() {
	return _lf_my_fed_id;
}

/**
 * Internal implementation of the next few reporting functions.
 */
void _lf_message_print(
		int is_error, const char* prefix, const char* format, va_list args, int log_level
) {
	// The logging level may be set either by a LOG_LEVEL #define
	// (which is code generated based on the logging target property)
	// or by a register_print_function() call. Honor both. If neither
	// has been set, then assume LOG_LEVEL_INFO. If both have been set,
	// then honor the maximum.
	int print_level = -1;
#ifdef LOG_LEVEL
	print_level = LOG_LEVEL;
#endif
	if (print_level < print_message_level) {
		print_level = print_message_level;
	}
	if (print_level < 0) {
		// Neither has been set.
		print_level = LOG_LEVEL_INFO;
	}
	if (log_level <= print_level) {
		// Rather than calling printf() multiple times, we need to call it just
		// once because this function is invoked by multiple threads.
		// If we make multiple calls to printf(), then the results could be
		// interleaved between threads.
		// vprintf() is a version that takes an arg list rather than multiple args.
		size_t length = strlen(prefix) + strlen(format) + 32;
		char* message = (char*) malloc(length + 1);
		if (_lf_my_fed_id < 0) {
			snprintf(message, length, "%s%s\n",
					prefix, format);
		} else {
			snprintf(message, length, "Federate %d: %s%s\n",
					_lf_my_fed_id, prefix, format);
		}
		if (print_message_function == NULL) {
			if (is_error) {
				vfprintf(stderr, message, args);
			} else {
				vfprintf(stdout, message, args);
			}
		} else {
			(*print_message_function)(message, args);
		}
		free(message);
	}
}

/**
 * Report an informational message on stdout with
 * a newline appended at the end.
 * If this execution is federated, then
 * the message will be prefaced by "Federate n: ",
 * where n is the federate ID.
 * The arguments are just like printf().
 */
void lf_print(const char* format, ...) {
	va_list args;
    va_start (args, format);
    lf_vprint(format, args);
	va_end (args);
}

/**
 * varargs alternative of "lf_print"
 */
void lf_vprint(const char* format, va_list args) {
    _lf_message_print(0, "", format, args, LOG_LEVEL_INFO);
}

/**
 * Report an log message on stdout with the prefix
 * "LOG: " and a newline appended
 * at the end. If this execution is federated, then
 * the message will be prefaced by "Federate n: ",
 * where n is the federate ID.
 * The arguments are just like printf().
 */
void lf_print_log(const char* format, ...) {
	va_list args;
    va_start (args, format);
	lf_vprint_log(format, args);
	va_end (args);
}

/**
 * varargs alternative of "lf_print_log"
 */
void lf_vprint_log(const char* format, va_list args) {
    _lf_message_print(0, "LOG: ", format, args, LOG_LEVEL_LOG);
}


/**
 * Report an debug message on stdout with the prefix
 * "DEBUG: " and a newline appended
 * at the end. If this execution is federated, then
 * the message will be prefaced by "Federate n: ",
 * where n is the federate ID.
 * The arguments are just like printf().
 */
void lf_print_debug(const char* format, ...) {
    va_list args;
    va_start (args, format);
    lf_vprint_debug(format, args);
    va_end (args);
}

/**
 * varargs alternative of "lf_print_debug"
 */
void lf_vprint_debug(const char* format, va_list args) {
    _lf_message_print(0, "DEBUG: ", format, args, LOG_LEVEL_DEBUG);
}

/**
 * Report an error with the prefix "ERROR: " and a newline appended
 * at the end.  The arguments are just like printf().
 */
void lf_print_error(const char* format, ...) {
    va_list args;
    va_start (args, format);
    lf_vprint_error(format, args);
    va_end (args);
}

/**
 * varargs alternative of "lf_print_error"
 */
void lf_vprint_error(const char* format, va_list args) {
    _lf_message_print(1, "ERROR: ", format, args, LOG_LEVEL_ERROR);
}

/**
 * Report a warning with the prefix "WARNING: " and a newline appended
 * at the end.  The arguments are just like printf().
 */
void lf_print_warning(const char* format, ...) {
    va_list args;
    va_start (args, format);
    lf_vprint_warning(format, args);
    va_end (args);
}

/**
 * varargs alternative of "lf_print_warning"
 */
void lf_vprint_warning(const char* format, va_list args) {
    _lf_message_print(1, "WARNING: ", format, args, LOG_LEVEL_WARNING);
}

/**
 * Report an error with the prefix "ERROR: " and a newline appended
 * at the end, then exit with the failure code EXIT_FAILURE.
 * The arguments are just like printf().
 */
void lf_print_error_and_exit(const char* format, ...) {
    va_list args;
    va_start (args, format);
    lf_vprint_error_and_exit(format, args);
    va_end (args);
    exit(EXIT_FAILURE);
}

/**
 * varargs alternative of "lf_print_error_and_exit"
 */
void lf_vprint_error_and_exit(const char* format, va_list args) {
    _lf_message_print(1, "FATAL ERROR: ", format, args, LOG_LEVEL_ERROR);
}

/**
 * Register a function to display messages. After calling this,
 * all messages passed to the above print functions will be
 * printed using the specified function rather than printf
 * if their log level is greater than the specified level.
 * The level should be one of LOG_LEVEL_ERROR, LOG_LEVEL_WARNING,
 * LOG_LEVEL_INFO, LOG_LEVEL_LOG, or LOG_LEVEL_DEBUG.
 *
 * @param function The print message function or NULL to revert
 *  to using printf.
 * @param log_level The level of messages to redirect.
 */
void lf_register_print_function(print_message_function_t* function, int log_level) {
    print_message_function = function;
    print_message_level = log_level;
}

