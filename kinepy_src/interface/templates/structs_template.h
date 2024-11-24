#ifndef INTERFACE_STRUCTS_H

#include "interface/configuration.h"

/**
 * Values represent the conversion coefficient from user unit to SI unit.
 * E.g. 1mm = 0.001m therefore, unit_system.length = 0.001 .
 * Kinepy internally stores and handles physical quantities in SI units, this means:
 *  - each physical quantity value the user provides to kinepy will be multiplied by its corresponding unit value;
 *  - each physical quantity value the user retrieves from kinepy has been divided by its corresponding unit value.
 */
typedef struct {
    float_type angle;
    float_type length;
    float_type mass;
    float_type moment_of_inertia;
    float_type force;
    float_type torque;
    float_type dimensionless;
} UnitSystem;


typedef struct {
    KpConfiguration config;
    UnitSystem * unit_system;
    struct {
        float_type mass;
        float_type moment_of_inertia;
        float_type gx;
        float_type gy;
    } * solid_parameters_ptr;
    struct {
        float_type x1;
        float_type y1;
        float_type x2;
        float_type y2;
    } * joint_parameters_ptr;
    struct {
        float_type ratio;
        float_type v0;
    } * relation_parameters_ptr;
} System;


typedef struct {
    uint32_t frame_count;

    float_type * _temp_arrays[4];

    float_type * solid_orientation_x;
    float_type * solid_orientation_y;
    float_type * solid_x;
    float_type * solid_y;

    float_type * _solid_force_x;
    float_type * _solid_force_y;
    float_type * _solid_torque;

    float_type * joint_value;

} Result;

#endif