#ifndef _simulator_H
#define _simulator_H
#ifndef _SIMULATOR_H // necessary for arduino-cli, which automatically includes headers that are not used
#include "pythontarget.h"
#include <limits.h>
#include "low_level_platform/api/low_level_platform.h"
#include "include/api/schedule.h"
#include "include/core/reactor.h"
#include "include/core/reactor_common.h"
#include "include/core/threaded/scheduler.h"
#include "include/core/mixed_radix.h"
#include "include/core/port.h"
#include "include/core/environment.h"
int lf_reactor_c_main(int argc, const char* argv[]);
#ifdef __cplusplus
extern "C" {
#endif
#include "../include/api/schedule.h"
#include "../include/core/reactor.h"
#ifdef __cplusplus
}
#endif
typedef struct simulator_self_t{
    self_base_t base; // This field is only to be used by the runtime, not the user.
    interval_t interval;
    PyObject* initial_speeds;
    PyObject* initial_speed;
    PyObject* positions;
    PyObject* start_pos;
    PyObject* max_acceleration;
    PyObject* drag_acceleration;
    PyObject* current_pos;
    PyObject* current_velocity;
    PyObject* current_throttle;
    PyObject* current_brake;
    PyObject* velocity_vector;
    int end[0]; // placeholder; MSVC does not compile empty structs
} simulator_self_t;
typedef generic_port_instance_struct _simulator_vehicle_command_t;
typedef generic_port_instance_struct _simulator_vehicle_stat_t;
typedef generic_port_instance_struct _simulator_vehicle_pos_t;
#endif
#endif
