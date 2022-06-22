/**
 * @file
 * @author Soroush Bateni (soroush@utdallas.edu)
 * @autohr Hou Seng Wong (housengw@berkeley.edu)
 *
 * @section LICENSE
Copyright (c) 2022, The University of California at Berkeley.
Copyright (c) 2021, The University of Texas at Dallas.

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
 * Implementation of functions defined in @see pythontarget.h
 */

#ifndef PYTHON_ACTION_H
#define PYTHON_ACTION_H

#include <Python.h>
#include <structmember.h>
#include "python_capsule_extension.h"

/**
 * The struct used to instantiate an action.
 * This is used 
 * in the PythonGenerator instead of redefining
 * a struct for each action.
 * This can be used for any Python object,
 * including lists and tuples.
 * PyObject* value: the value of the action with the generic Python type
 * is_present: indicates if the action is present
 *             at the current logical time
 **/
typedef struct {
    trigger_t* trigger;
    PyObject* value;
    bool is_present;
    bool has_value;
    lf_token_t* token;
    FEDERATED_CAPSULE_EXTENSION
} generic_action_instance_struct;

/**
 * The struct used to hold an action
 * that is sent to a Python reaction.
 * 
 * The "action" field holds a PyCapsule of the
 * void * pointer to an action.
 * 
 * The "value" field holds the action value
 * if anything is given. This value is copied over
 * from action->value each time an action is passed
 * to a Python reaction.
 * 
 * The "is_present" field is copied over
 * from action->value each time an action is passed
 * to a Python reaction.
 **/
typedef struct {
    PyObject_HEAD
    PyObject* action; // Hold the void* pointer to a C action instance. However, passing void* directly
                      // to Python is considered unsafe practice. Instead, this void* pointer to the C action
                      // will be stored in a PyCapsule. @see https://docs.python.org/3/c-api/capsule.html
    PyObject* value; // This value will be copied from the C action->value
    bool is_present; // Same as value, is_present will be copied from the C action->is_present
    FEDERATED_CAPSULE_EXTENSION
} generic_action_capsule_struct;

#endif