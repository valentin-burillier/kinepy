#ifndef INTERFACE_STRUCTS_H

#include "stdint.h"

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
    float_type mass;
    float_type moment_of_inertia;
    float_type gx;
    float_type gy;
} SolidDescription;

typedef struct {
    uint32_t obj_count;
    SolidDescription * solid_description_ptr;
} SolidDescriptionArray;

typedef struct {
    uint32_t solid1;
    uint32_t solid2;
    uint8_t type;
    float_type constraint1_x;
    float_type constraint1_y;
    float_type constraint2_x;
    float_type constraint2_y;
} JointDescription;

typedef struct {
    uint32_t obj_count;
    uint32_t * solid1_ptr;
    uint32_t * solid2_ptr;
    uint8_t * type_ptr;
    struct joint_constraints {
        float_type x1;
        float_type y1;
        float_type x2;
        float_type y2;
    } * constraint_ptr;
} JointDescriptionArray;


typedef struct {
    uint32_t joint1;
    uint32_t joint2;
    uint8_t type;
    float_type ratio;
    float_type v0;
} RelationDescription;

typedef struct {
    uint32_t obj_count;
    uint32_t * joint1_ptr;
    uint32_t * joint2_ptr;
    uint8_t * type_ptr;
    struct relation_parameters {
        float_type ratio;
        float_type v0;
    } * parameter_ptr;
} RelationDescriptionArray;


typedef struct {
    SolidDescriptionArray solids;
    JointDescriptionArray joints;
    RelationDescriptionArray relations;
    UnitSystem * unit_system;
} System;

#endif