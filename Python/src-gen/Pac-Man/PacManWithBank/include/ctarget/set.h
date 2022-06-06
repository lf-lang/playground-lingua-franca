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

#ifndef CTARGET_SET
#define CTARGET_SET

//////////////////////////////////////////////////////////////
/////////////  SET Functions (to produce an output)

// NOTE: According to the "Swallowing the Semicolon" section on this page:
//    https://gcc.gnu.org/onlinedocs/gcc-3.0.1/cpp_3.html
// the following macros should use an odd do-while construct to avoid
// problems with if ... else statements that do not use braces around the
// two branches.

/**
 * Set the specified output (or input of a contained reactor)
 * to the specified value.
 *
 * If the value argument is a primitive type such as int,
 * double, etc. as well as the built-in types bool and string, 
 * the value is copied and therefore the variable carrying the
 * value can be subsequently modified without changing the output.
 * This also applies to structs with a type defined by a typedef
 * so that the type designating string does not end in '*'.
 * 
 * If the value argument is a pointer
 * to memory that the calling reaction has dynamically allocated,
 * the memory will be automatically freed once all downstream
 * reactions no longer need the value.
 * If 'lf_set_destructor' is called on 'out', then that destructor
 * will be used to free 'value'. 
 * Otherwise, the default void free(void*) function is used.
 * 
 * @param out The output port (by name) or input of a contained
 *  reactor in form input_name.port_name.
 * @param value The value to insert into the self struct.
 */
#define lf_set(out, val) _LF_SET(out, val)
#define SET(out, val) \
do { \
        _Pragma ("Warning \"'SET' is deprecated. Use 'lf_set' instead.\""); \
        _LF_SET(out, val); \
} while (0)

/**
 * Version of lf_set for output types given as 'type[]' where you
 * want to send a previously dynamically allocated array.
 *
 * The deallocation is delegated to downstream reactors, which
 * automatically deallocate when the reference count drops to zero.
 * It also sets the corresponding _is_present variable in the self
 * struct to true (which causes the object message to be sent).
 * @param out The output port (by name).
 * @param val The array to send (a pointer to the first element).
 * @param length The length of the array to send.
 * @see lf_token_t
 */
#define SET_ARRAY(out, val, elem_size, length) \
do { \
        _Pragma ("Warning \"'SET_ARRAY' is deprecated.\""); \
        _LF_SET_ARRAY(out, val, length); \
} while (0)

/**
 * Version of lf_set() for output types given as 'type*' that
 * allocates a new object of the type of the specified output port.
 *
 * This macro dynamically allocates enough memory to contain one
 * instance of the output datatype and sets the variable named
 * by the argument to point to the newly allocated memory.
 * The user code can then populate it with whatever value it
 * wishes to send.
 *
 * This macro also sets the corresponding _is_present variable in the self
 * struct to true (which causes the object message to be sent),
 * @param out The output port (by name).
 */
#define SET_NEW(out) \
do { \
        _Pragma ("Warning \"'SET_NEW' is deprecated.\""); \
        _LF_SET_NEW(out); \
} while (0)

/**
 * Version of lf_set() for output types given as 'type[]'.
 *
 * This allocates a new array of the specified length,
 * sets the corresponding _is_present variable in the self struct to true
 * (which causes the array message to be sent), and sets the variable
 * given by the first argument to point to the new array so that the
 * user code can populate the array. The freeing of the dynamically
 * allocated array will be handled automatically
 * when the last downstream reader of the message has finished.
 * @param out The output port (by name).
 * @param len The length of the array to be sent.
 */
#define SET_NEW_ARRAY(out, len) \
do { \
        _Pragma ("Warning \"'SET_NEW_ARRAY' is deprecated.\""); \
        _LF_SET_NEW_ARRAY(out, len); \
} while (0)

/**
 * Version of lf_set() for output types given as 'type[number]'.
 *
 * This sets the _is_present variable corresponding to the specified output
 * to true (which causes the array message to be sent). The values in the
 * output are normally written directly to the array or struct before or
 * after this is called.
 * @param out The output port (by name).
 */
#define SET_PRESENT(out) \
do { \
        _Pragma ("Warning \"'SET_PRESENT' is deprecated.\""); \
        _LF_SET_PRESENT(out); \
} while (0)

/**
 * Version of lf_set() for output types given as 'type*' or 'type[]' where you want
 * to forward an input or action without copying it.
 *
 * The deallocation of memory is delegated to downstream reactors, which
 * automatically deallocate when the reference count drops to zero.
 * @param out The output port (by name).
 * @param token A pointer to token obtained from an input or action.
 */
#define lf_set_token(out, newtoken) _LF_SET_TOKEN(out, newtoken)
#define SET_TOKEN(out, newtoken) \
do { \
        _Pragma ("Warning \"'SET_TOKEN' is deprecated. Use 'lf_set_token' instead.\""); \
        _LF_SET_TOKEN(out, newtoken); \
} while (0)

/**
 * Set the destructor used to free "token->value" set on "out".
 * That memory will be automatically freed once all downstream
 * reactions no longer need the value.
 * 
 * @param out The output port (by name) or input of a contained
 *            reactor in form input_name.port_name.
 * @param dtor A pointer to a void function that takes a pointer argument
 *             or NULL to use the default void free(void*) function. 
 */
#define lf_set_destructor(out, dtor) _LF_SET_DESTRUCTOR(out, dtor)

/**
 * Set the destructor used to copy construct "token->value" received
 * by "in" if "in" is mutable.
 * 
 * @param out The output port (by name) or input of a contained
 *            reactor in form input_name.port_name.
 * @param cpy_ctor A pointer to a void* function that takes a pointer argument
 *                 or NULL to use the memcpy operator.
 */
#define lf_set_copy_constructor(out, cpy_ctor) _LF_SET_COPY_CONSTRUCTOR(out, cpy_ctor)

//////////////////////////////////////////////////////////////
/////////////  SET_MODE Function (to switch a mode)

/**
 * Sets the next mode of a modal reactor. Same as SET for outputs, only
 * the last value will have effect if invoked multiple times.
 * Works only in reactions with the target mode declared as effect.
 *
 * @param mode The target mode to set for activation.
 */
#ifdef MODAL_REACTORS
#define lf_set_mode(mode) _LF_SET_MODE(mode)
#define SET_MODE(mode) \
do { \
        _Pragma ("Warning \"'SET_MODE' is deprecated. Use 'lf_set_mode' instead.\""); \
        _LF_SET_MODE(mode); \
} while (0)
#endif // MODAL_REACTORS

#endif // CTARGET_SET