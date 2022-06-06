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
 * Implementation of modal models support in the Python target.
 */

#include "definitions.h"

//////////// set Function /////////////

/**
 * Set a new mode for a modal model.
 */
static PyObject* py_mode_set(PyObject *mode_capsule, PyObject *args) {
    mode_capsule_struct_t* m = (mode_capsule_struct_t*)mode_capsule;

    reactor_mode_t* mode = PyCapsule_GetPointer(m->mode, "mode");
    if (mode == NULL) {
        lf_print_error("Null pointer received.");
        exit(1);
    }

    self_base_t* self = PyCapsule_GetPointer(m->lf_self, "lf_self");
    if (self == NULL) {
        lf_print_error("Null pointer received.");
        exit(1);
    }

    _LF_SET_MODE_WITH_TYPE(mode, m->change_type);

    Py_INCREF(Py_None);
    return Py_None;
}

//////////// Python Struct /////////////

/*
 * The function members of mode_capsule.
 * The set function is used to set a new mode.
 */
static PyMethodDef mode_capsule_methods[] = {
    {"set", (PyCFunction)py_mode_set, METH_NOARGS, "Set a new mode."},
    {NULL}  /* Sentinel */
};

/*
 * The definition of mode_capsule type object, which is
 * used to describe how mode_capsule behaves.
 */
static PyTypeObject mode_capsule_t = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "LinguaFranca.mode_capsule",
    .tp_doc = "mode_capsule objects",
    .tp_basicsize = sizeof(mode_capsule_struct_t),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = PyType_GenericNew,
    .tp_methods = mode_capsule_methods,
};


///////////////// Functions used in mode creation and initialization /////////////

/**
 * @brief Initialize `mode_capsule_t` in the `current_module`.
 * 
 */
void initialize_mode_capsule_t(PyObject* current_module) {
    // Initialize the mode_capsule type
    if (PyType_Ready(&mode_capsule_t) < 0) {
        return;
    }

    // Add the mode_capsule type to the module's dictionary.
    Py_INCREF(&mode_capsule_t);
    if (PyModule_AddObject(current_module, "mode_capsule", (PyObject *) &mode_capsule_t) < 0) {
        Py_DECREF(&mode_capsule_t);
        Py_DECREF(current_module);
        return;
    }
}

/**
 * Convert a `reactor_mode_t` to a `mode_capsule_t`.
 */
PyObject* convert_C_mode_to_py(
		reactor_mode_t* mode,
		self_base_t* lf_self,
		lf_mode_change_type_t change_type
) {
    // Create the mode struct in Python
	mode_capsule_struct_t* cap =
        (mode_capsule_struct_t*)PyObject_GC_New(mode_capsule_struct_t, &mode_capsule_t);
    if (cap == NULL) {
        lf_print_error_and_exit("Failed to convert mode.");
    }

    // Create the capsule to hold the reactor_mode_t* mode
    PyObject* capsule = PyCapsule_New(mode, "mode", NULL);
    if (capsule == NULL) {
        lf_print_error_and_exit("Failed to convert mode.");
    }
    // Fill in the Python mode struct.
    cap->mode = capsule;

    // Create a capsule to point to the self struct.
    PyObject* self_capsule = PyCapsule_New(lf_self, "lf_self", NULL);
    if (self_capsule == NULL) {
        lf_print_error_and_exit("Failed to convert self.");
    }
    cap->lf_self = self_capsule;

    cap->change_type = change_type;

	return cap;
}