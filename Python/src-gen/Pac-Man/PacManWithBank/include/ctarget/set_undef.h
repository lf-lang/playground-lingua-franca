/**
 * @file
 * @author Hou Seng Wong (housengw@berkeley.edu)
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
 * Undefining the macros in the ctarget/set.h
 * Refer to ctarget/set.h for details of the macros
 * 
 * Note for target language developers. This is one way of developing a target language where 
 * the C core runtime is adopted. This file is a translation layer that implements Lingua Franca 
 * APIs which interact with the internal _lf_SET and _lf_schedule APIs. This file can act as a 
 * template for future runtime developement for target languages.
 * For source generation, see xtext/org.icyphy.linguafranca/src/org/icyphy/generator/CCppGenerator.xtend.
 */


#ifdef CTARGET_SET
#undef CTARGET_SET

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
 */
#undef lf_set
#undef SET

/**
 * Version of lf_set for output types given as 'type[]' where you
 * want to send a previously dynamically allocated array.
 */
#undef SET_ARRAY

/**
 * Version of lf_set() for output types given as 'type*' that
 * allocates a new object of the type of the specified output port.
 */
#undef SET_NEW

/**
 * Version of lf_set() for output types given as 'type[]'.
 */
#undef SET_NEW_ARRAY

/**
 * Version of lf_set() for output types given as 'type[number]'.
 */
#undef SET_PRESENT

/**
 * Version of lf_set() for output types given as 'type*' or 'type[]' where you want
 * to forward an input or action without copying it.
 */
#undef lf_set_token
#undef SET_TOKEN

/**
 * Set the destructor used to free "token->value" set on "out".
 * That memory will be automatically freed once all downstream
 * reactions no longer need the value.
 */
#undef lf_set_destructor


/**
 * Set the destructor used to copy construct "token->value" received
 * by "in" if "in" is mutable.
 */
#undef lf_set_copy_constructor

//////////////////////////////////////////////////////////////
/////////////  SET_MODE Function (to switch a mode)

/**
 * Sets the next mode of a modal reactor. Same as SET for outputs, only
 * the last value will have effect if invoked multiple times.
 * Works only in reactions with the target mode declared as effect.
 */
#ifdef MODAL_REACTORS
#undef lf_set_mode
#undef SET_MODE
#endif

#endif // CTARGET_SET