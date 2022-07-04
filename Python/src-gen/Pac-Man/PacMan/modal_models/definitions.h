/**
 * @file
 * @author Edward A. Lee (eal@berkeley.edu)
 * @author Soroush Bateni (soroush@utdallas.edu)
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
 * Definitions needed for the modal models in the Python target.
 */

#ifndef PYTHON_MODAL_MODELS_DEFS_H
#define PYTHON_MODAL_MODELS_DEFS_H


#ifdef MODAL_REACTORS
#include <Python.h>
#include <structmember.h>
#include "../include/ctarget/ctarget.h"

/**
 * The struct used to represent modes in Python.
 * An instance of this struct is created when entering a reaction that
 * has declared a mode as an effect. This struct represents everything
 * needed to take a transition to that mode, including a pointer to
 * that mode and the type of transition (reset or history).
 */
typedef struct {
	PyObject_HEAD
	PyObject* mode;
	PyObject* lf_self;
	lf_mode_change_type_t change_type;
} mode_capsule_struct_t;


/**
 * Set a new mode for a modal model.
 */
static PyObject* py_mode_set(PyObject *self, PyObject *args);

/**
 * Convert a `reactor_mode_t` to a `mode_capsule_t`.
 */
PyObject* convert_C_mode_to_py(
		reactor_mode_t* mode,
		self_base_t* lf_self,
		lf_mode_change_type_t change_type
);

/**
 * @brief Initialize `mode_capsule_t` in the `current_module`.
 * 
 */
void initialize_mode_capsule_t(PyObject* current_module);

#else 
#define initialize_mode_capsule_t(...) 
#endif // MODAL_REACTORS

#endif // PYTHON_MODAL_MODELS_DEFS_H