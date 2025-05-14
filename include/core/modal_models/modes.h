/*************
Copyright (c) 2021, Kiel University.

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
***************/

/**
 * Header file of the runtime infrastructure for modes in the C target of Lingua Franca.
 * This file contains user macros for setting new modes, forward declarations for
 * functions and types used in the generated code, and types definitions for the modal
 * representation.
 *
 * Any mode related code will only work in the presence of the MODAL_REACTORS
 * definition.
 * However, this header should be included regardless to provide definitions
 * for mode-unaware pre-compilation.
 *
 * This file is intended for direct include in reactor.h augmenting certain type
 * definitions.
 *
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 * @author{Soroush Bateni <soroush@utdallas.edu}
 */
#ifndef MODES_H
#define MODES_H

#ifdef MODAL_REACTORS

#include <stddef.h>
#include <stdbool.h>

#include "lf_types.h"
#include "tag.h"

typedef struct event_t event_t;
typedef struct reaction_t reaction_t;
typedef struct trigger_t trigger_t;

////////////////////////////////////////////////////////////
//// Macros for setting modes.

/**
 * Sets the next mode of a modal reactor with an explicit change type
 * (reset or history, from the enum `lf_mode_change_type_t`).
 * This macro is not meant to be used by LF programmers.
 * It is used in Python.
 *
 * @param mode The target mode to set for activation.
 * @param change_type The change type of the transition.
 */
#define _LF_SET_MODE_WITH_TYPE(mode, change_type)                                                                      \
  do {                                                                                                                 \
    ((self_base_t*)self)->_lf__mode_state.next_mode = mode;                                                            \
    ((self_base_t*)self)->_lf__mode_state.mode_change = change_type;                                                   \
  } while (0)

////////////////////////////////////////////////////////////
//// Type definitions for modal infrastructure.

/** Typedef for reactor_mode_t struct, used for representing a mode. */
typedef struct reactor_mode_t reactor_mode_t;
/** Typedef for reactor_mode_state_t struct, used for storing modal state of reactor and/or its relation to enclosing
 * modes. */
typedef struct reactor_mode_state_t reactor_mode_state_t;
/** Typedef for mode_state_variable_reset_data_t struct, used for storing data for resetting state variables nested in
 * modes. */
typedef struct mode_state_variable_reset_data_t mode_state_variable_reset_data_t;

/** Type of the mode change. */
typedef enum { no_transition, reset_transition, history_transition } lf_mode_change_type_t;

/** A struct to represent a single mode instace in a reactor instance. */
struct reactor_mode_t {
  reactor_mode_state_t* state; // Pointer to a struct with the reactor's mode state. INSTANCE.
  char* name;                  // Name of this mode.
  instant_t deactivation_time; // Time when the mode was left.
  uint8_t flags;               // Bit vector for several internal flags related to the mode.
};

/** A struct to store state of the modes in a reactor instance and/or its relation to enclosing modes. */
struct reactor_mode_state_t {
  reactor_mode_t* parent_mode;       // Pointer to the next enclosing mode (if exists).
  reactor_mode_t* initial_mode;      // Pointer to the initial mode.
  reactor_mode_t* current_mode;      // Pointer to the currently active mode (only locally active).
  reactor_mode_t* next_mode;         // Pointer to the next mode to activate at the end of this step (if set).
  lf_mode_change_type_t mode_change; // A mode change type flag.
};
/** A struct to store data for resetting state variables nested in modes. */
struct mode_state_variable_reset_data_t {
  reactor_mode_t* mode; // Pointer to the enclosing mode.
  void* target;         // Pointer to the target variable.
  void* source;         // Pointer to the data source.
  size_t size;          // The size of the variable.
};

////////////////////////////////////////////////////////////
//// Forward declaration
typedef struct environment_t environment_t;

////////////////////////////////////////////////////////////
//// Modes API
void _lf_initialize_modes(environment_t* env);
void _lf_handle_mode_changes(environment_t* env);
void _lf_handle_mode_triggered_reactions(environment_t* env);
bool _lf_mode_is_active(reactor_mode_t* mode);
void _lf_initialize_mode_states(environment_t* env, reactor_mode_state_t* states[], int states_size);
void _lf_process_mode_changes(environment_t* env, reactor_mode_state_t* states[], int states_size,
                              mode_state_variable_reset_data_t reset_data[], int reset_data_size,
                              trigger_t* timer_triggers[], int timer_triggers_size);
void _lf_add_suspended_event(event_t* event);
void _lf_handle_mode_startup_reset_reactions(environment_t* env, reaction_t** startup_reactions,
                                             int startup_reactions_size, reaction_t** reset_reactions,
                                             int reset_reactions_size, reactor_mode_state_t* states[], int states_size);
void _lf_handle_mode_shutdown_reactions(environment_t* env, reaction_t** shutdown_reactions,
                                        int shutdown_reactions_size);
void _lf_terminate_modal_reactors(environment_t* env);

#else  /* IF NOT MODAL_REACTORS */
/*
 * Reactions and triggers must have a mode pointer to set up connection to enclosing modes,
 * also when they are precompiled without modal reactors in order to later work in modal reactors.
 * Hence define mode type as void in the absence of modes to treat mode pointer as void pointers for that time being.
 */
typedef void reactor_mode_t;
#endif /* MODAL_REACTORS */
#endif /* MODES_H */
