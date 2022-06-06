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

//////////////////////////////////////////////////////////////
/////////////  Util Functions
/**
 * This file gives definition of deprecated util functions in the C target. 
 * For definition of up-to-date util functions, please refer to util.c in
 * the core files folder.
 */
#include "../include/ctarget/util.h"
#include <stdarg.h>   // Defines va_list
#include "stdlib.h"

/**
 * @deprecated version of "lf_print"
 */
void info_print(const char* format, ...) {
    va_list args;
    va_start (args, format);
    lf_vprint(format, args);
	va_end (args);
}

/**
 * @deprecated version of "lf_print_log"
 */
void log_print(const char* format, ...) {
    va_list args;
    va_start (args, format);
	lf_vprint_log(format, args);
	va_end (args);
}

/**
 * @deprecated version of "lf_print_debug"
 */
void debug_print(const char* format, ...) {
    va_list args;
    va_start (args, format);
    lf_vprint_debug(format, args);
    va_end (args);
}

/**
 * @deprecated version of "lf_print_error"
 */
void error_print(const char* format, ...) {
    va_list args;
    va_start (args, format);
    lf_vprint_error(format, args);
    va_end (args);
}

/**
 * @deprecated version of "lf_print_warning"
 */
void warning_print(const char* format, ...) {
    va_list args;
    va_start (args, format);
    lf_vprint_warning(format, args);
    va_end (args);
}

/**
 * @deprecated version of "lf_print_error_and_exit"
 */
void error_print_and_exit(const char* format, ...) {
    va_list args;
    va_start (args, format);
    lf_vprint_error_and_exit(format, args);
    va_end (args);
    exit(EXIT_FAILURE);
}

/**
 * @deprecated version of "lf_register_print_function"
 */
void register_print_function(print_message_function_t* function, int log_level) {
    lf_register_print_function(function, log_level);
}

/**
 * @deprecated version of "lf_get_stp_offset"
 */
interval_t get_stp_offset(void) {
    return lf_get_stp_offset();
}

/**
 * @deprecated version of "lf_set_stp_offset"
 */
void set_stp_offset(interval_t offset) {
    lf_set_stp_offset(offset);
}

/**
 * @deprecated version of "lf_print_snapshot"
 */
void print_snapshot(void) {
    lf_print_snapshot();
}


/**
 * @deprecated version of "lf_request_stop"
 */
void request_stop(void) {
    lf_request_stop();
}
