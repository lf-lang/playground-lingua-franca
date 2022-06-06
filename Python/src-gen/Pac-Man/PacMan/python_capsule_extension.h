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

#ifndef PYTHON_CAPSULE_EXTENSION_H
#define PYTHON_CAPSULE_EXTENSION_H

#ifdef FEDERATED
#ifdef FEDERATED_DECENTRALIZED
#define FEDERATED_GENERIC_EXTENSION \
    tag_t intended_tag; \
    instant_t physical_time_of_arrival;

#define FEDERATED_CAPSULE_EXTENSION \
    py_tag_t* intended_tag; \
    instant_t physical_time_of_arrival;

#define FEDERATED_CAPSULE_MEMBER \
    {"intended_tag", T_OBJECT, offsetof(generic_port_capsule_struct, intended_tag), READONLY, "Original intended tag of the event."}, \
    {"physical_time_of_arrival", T_LONG, offsetof(generic_port_capsule_struct, physical_time_of_arrival), READONLY, "Physical time of arrival of the original message."},

#define FEDERATED_ASSIGN_FIELDS(py_port, c_port) \
do { \
    py_port->intended_tag = convert_C_tag_to_py(c_port->intended_tag); \
    py_port->physical_time_of_arrival = c_port->physical_time_of_arrival; \
} while(0)

#else // FEDERATED_CENTRALIZED
#define FEDERATED_GENERIC_EXTENSION \
    instant_t physical_time_of_arrival;

#define FEDERATED_CAPSULE_EXTENSION FEDERATED_GENERIC_EXTENSION

#define FEDERATED_CAPSULE_MEMBER \
    {"physical_time_of_arrival", T_INT, offsetof(generic_port_capsule_struct, physical_time_of_arrival), READONLY, "Physical time of arrival of the original message."},

#define FEDERATED_ASSIGN_FIELDS(py_port, c_port) \
do { \
    py_port->physical_time_of_arrival = c_port->physical_time_of_arrival; \
} while(0)
#endif // FEDERATED_DECENTRALIZED
#else  // not FEDERATED
#define FEDERATED_GENERIC_EXTENSION // Empty
#define FEDERATED_CAPSULE_EXTENSION // Empty
#define FEDERATED_CAPSULE_MEMBER // Empty
#define FEDERATED_ASSIGN_FIELDS(py_port, c_port) // Empty
#define FEDERATED_COPY_FIELDS(py_port1, py_port2) // Empty
#endif // FEDERATED

#endif
