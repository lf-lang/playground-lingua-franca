/**
 * @file
 * @author Hou Seng (Steven) Wong (housengw@berkeley.edu)
 *
 * @section LICENSE
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

 * @section DESCRIPTION
 * Target-specific runtime functions for the C target language.
 * This API layer can be used in conjunction with:
 *     target C;
 * 
 * Note for target language developers. This is one way of developing a target language where 
 * the C core runtime is adopted. This file is a translation layer that implements Lingua Franca 
 * APIs which interact with the internal _lf_SET and _lf_schedule APIs. This file can act as a 
 * template for future runtime developement for target languages.
 * For source generation, see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CCppGenerator.xtend.
 */


#ifndef CTARGET_UTIL
#define CTARGET_UTIL
#include "../core/reactor.h"

//////////////////////////////////////////////////////////////
/////////////  Util Functions

/**
 * Return the federate ID or -1 if this program is not part of a federation.
 */
int lf_fed_id(void);

/**
 * Report an informational message on stdout with
 * a newline appended at the end.
 * If this execution is federated, then
 * the message will be prefaced by "Federate n: ",
 * where n is the federate ID.
 * The arguments are just like printf().
 */
void lf_print(const char* format, ...);
DEPRECATED(void info_print(const char* format, ...));

/**
 * Report an log message on stdout with the prefix
 * "LOG: " and a newline appended
 * at the end. If this execution is federated, then
 * the message will be prefaced by "Federate n: ",
 * where n is the federate ID.
 * The arguments are just like printf().
 */
void lf_print_log(const char* format, ...);
DEPRECATED(void log_print(const char* format, ...));

/**
 * Report an debug message on stdout with the prefix
 * "DEBUG: " and a newline appended
 * at the end. If this execution is federated, then
 * the message will be prefaced by "Federate n: ",
 * where n is the federate ID.
 * The arguments are just like printf().
 */
void lf_print_debug(const char* format, ...);
DEPRECATED(void debug_print(const char* format, ...));

/**
 * Report an error with the prefix "ERROR: " and a newline appended
 * at the end.  The arguments are just like printf().
 */
void lf_print_error(const char* format, ...);
DEPRECATED(void error_print(const char* format, ...));

/**
 * Report a warning with the prefix "WARNING: " and a newline appended
 * at the end.  The arguments are just like printf().
 */
void lf_print_warning(const char* format, ...);
DEPRECATED(void warning_print(const char* format, ...));

/**
 * Report an error with the prefix "ERROR: " and a newline appended
 * at the end, then exit with the failure code EXIT_FAILURE.
 * The arguments are just like printf().
 */
void lf_print_error_and_exit(const char* format, ...);
DEPRECATED(void error_print_and_exit(const char* format, ...));


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
void lf_register_print_function(print_message_function_t* function, int log_level);
DEPRECATED(void register_print_function(print_message_function_t* function, int log_level));

/**
 * A macro used to print useful debug information. It can be enabled
 * by setting the target property 'logging' to 'DEBUG' or
 * by defining LOG_LEVEL to 2 in the top-level preamble.
 * The input to this macro is exactly like printf: (format, ...).
 * "DEBUG: " is prepended to the beginning of the message
 * and a newline is appended to the end of the message.
 *
 * @note This macro is non-empty even if LOG_LEVEL is not defined in
 * user-code. This is to ensure that the compiler will still parse
 * the predicate inside (...) to prevent LF_PRINT_DEBUG statements
 * to fall out of sync with the rest of the code. This should have
 * a negligible impact on performance if compiler optimization
 * (e.g., -O2 for gcc) is used as long as the arguments passed to
 * it do not themselves incur significant overhead to evaluate.
 * 
 * @deprecated
 */
#define DEBUG_PRINT(format, ...) \
            do { if(LOG_LEVEL >= LOG_LEVEL_DEBUG) { \
                    debug_print(format, ##__VA_ARGS__); \
                } } while (0)

/**
 * A macro used to print useful logging information. It can be enabled
 * by setting the target property 'logging' to 'LOG' or
 * by defining LOG_LEVEL to LOG_LEVEL_LOG or
 * LOG_LEVEL_DEBUG in the top-level preamble.
 * The input to this macro is exactly like printf: (format, ...).
 * "LOG: " is prepended to the beginning of the message
 * and a newline is appended to the end of the message.
 *
 * @note This macro is non-empty even if LOG_LEVEL is not defined in
 * user-code. This is to ensure that the compiler will still parse
 * the predicate inside (...) to prevent LF_PRINT_LOG statements
 * to fall out of sync with the rest of the code. This should have
 * a negligible impact on performance if compiler optimization
 * (e.g., -O2 for gcc) is used as long as the arguments passed to
 * it do not themselves incur significant overhead to evaluate.
 * 
 * @deprecated
 */
#define LOG_PRINT(format, ...) \
            do { if(LOG_LEVEL >= LOG_LEVEL_LOG) { \
                    log_print(format, ##__VA_ARGS__); \
                } } while (0)

#endif // CTARGET_UTIL
