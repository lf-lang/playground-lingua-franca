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
 * @deprecated version of "lf_print"
 */
DEPRECATED(void info_print(const char* format, ...));

/**
 * @deprecated version of "lf_print_log"
 */
DEPRECATED(void log_print(const char* format, ...));

/**
 * @deprecated version of "lf_print_debug"
 */
DEPRECATED(void debug_print(const char* format, ...));

/**
 * @deprecated version of "lf_print_error"
 */
DEPRECATED(void error_print(const char* format, ...));

/**
 * @deprecated version of "lf_print_warning"
 */
DEPRECATED(void warning_print(const char* format, ...));

/**
 * @deprecated version of "lf_print_error_and_exit"
 */
DEPRECATED(void error_print_and_exit(const char* format, ...));


/**
 * @deprecated version of "lf_register_print_function"
 */
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


/**
 * @deprecated version of "lf_get_stp_offset"
 */
DEPRECATED(interval_t get_stp_offset(void));

/**
 * @deprecated version of "lf_set_stp_offset"
 */
DEPRECATED(void set_stp_offset(interval_t offset));

/**
 * @deprecated version of "lf_print_snapshot"
 */
DEPRECATED(void print_snapshot(void));

/**
 * @deprecated version of "lf_request_stop"
 */
DEPRECATED(void request_stop(void));

#endif // CTARGET_UTIL
