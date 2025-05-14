#ifndef _vehicle_H
#define _vehicle_H
#ifndef _VEHICLE_H // necessary for arduino-cli, which automatically includes headers that are not used
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
typedef struct vehicle_self_t{
    self_base_t base; // This field is only to be used by the runtime, not the user.
    PyObject* vehicle_id;
    PyObject* current_pos;
    PyObject* last_pos;
    PyObject* granted_time_to_enter;
    PyObject* intersection_pos;
    PyObject* goal_reached;
    PyObject* velocity;
    int end[0]; // placeholder; MSVC does not compile empty structs
} vehicle_self_t;
typedef generic_port_instance_struct _vehicle_vehicle_stat_t;
typedef generic_port_instance_struct _vehicle_vehicle_pos_t;
typedef generic_port_instance_struct _vehicle_grant_t;
typedef generic_port_instance_struct _vehicle_request_t;
typedef generic_port_instance_struct _vehicle_control_t;
typedef generic_port_instance_struct _vehicle_goal_reached_t;
typedef generic_action_instance_struct _vehicle_delay_t;
#endif
#endif
