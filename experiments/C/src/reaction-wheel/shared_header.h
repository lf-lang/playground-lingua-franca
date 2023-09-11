
#ifndef SHARED_HEADER_GUARD
#define SHARED_HEADER_GUARD

#include <math.h>
#include <time.h>
#include <stdlib.h>


typedef struct  {
    double platform_inertia; // [kg * m^2]
    double wheel_inertia; // [kg * m^2]
    double viscous_friction; // [m * N * m]
    double coulomb_friction; // [m * N * m]
    double max_platform_position; // + pi [radian]
    double min_platform_position; // - pi [radian]
} WheelConfiguration;

typedef struct {
    double voltage;
    double max_ampere;
    double min_ampere;
} MotorConfiguration;
    
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#endif // SHARED_HEADER_GUARD
