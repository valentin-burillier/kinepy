#ifndef LAYOUTS_H
#define LAYOUTS_H

#include "stdint.h"

typedef enum {
    KINEPY_SUCCESS,
    KINEPY_GENERIC_FAILURE,
    KINEPY_INVALID_INPUT,
    KINEPY_NO_GRAPH_FOUND,
    KINEPY_MALLOC_FAILED
} KinepyResult;

#define ATTRIBUTES(TYPE, NAME, ...) TYPE NAME;
#define ARRAY_ATTRIBUTES(TYPE, NAME, ...) TYPE * NAME##_ptr;

#define SOLID_DESC_LAYOUT(CB, FLOAT)                            \
    CB(FLOAT, mass, STD_UNIT, mass)                             \
    CB(FLOAT, moment_of_inertia, STD_UNIT, moment_of_inertia)   \
    CB(FLOAT, gx, STD_UNIT, length)                             \
    CB(FLOAT, gy, STD_UNIT, length)

#define SOLID_RES_LAYOUT(CB, FLOAT)                     \
    CB(FLOAT, origin_x, STD_UNIT, length)               \
    CB(FLOAT, origin_y, STD_UNIT, length)               \
    CB(FLOAT, orientation_x, STD_UNIT, dimensionless)   \
    CB(FLOAT, orientation_y, STD_UNIT, dimensionless)   \
    CB(FLOAT, _dynamic_x, STD_UNIT, force)              \
    CB(FLOAT, _dynamic_y, STD_UNIT, force)              \
    CB(FLOAT, _dynamic_m, STD_UNIT, torque)

#define JOINT_DESC_LAYOUT(CB, FLOAT)                            \
    CB(uint32_t, solid1, NOT_A_PHYSICAL_QUANTITY, is_existing_solid)  \
    CB(uint32_t, solid2, NOT_A_PHYSICAL_QUANTITY, is_existing_solid)  \
    CB(uint8_t, type, NOT_A_PHYSICAL_QUANTITY, is_existing_joint_type)\
    CB(FLOAT, constraint1_x, STD_UNIT, length)                  \
    CB(FLOAT, constraint1_y, STD_UNIT, length)                  \
    CB(FLOAT, constraint2_x, STD_UNIT, length)                  \
    CB(FLOAT, constraint2_y, STD_UNIT, length)

#define RELATION_DESC_LAYOUT(CB, FLOAT) \
    CB(uint32_t, joint1, NOT_A_PHYSICAL_QUANTITY, is_existing_joint)  \
    CB(uint32_t, joint2, NOT_A_PHYSICAL_QUANTITY, is_existing_joint)  \
    CB(uint8_t, type, NOT_A_PHYSICAL_QUANTITY, is_existing_relation_type) \
    CB(FLOAT, ratio, RELATION_RATIO)    \
    CB(FLOAT, v0, RELATION_V0)

#define MAKE_STRUCTS(LAYOUT, BASE_NAME, ARRAY_ATTR, FLOAT, F_SUFFIX) \
typedef struct {                                                     \
    LAYOUT(ATTRIBUTES, FLOAT)                                        \
} BASE_NAME##F_SUFFIX;                                               \
typedef struct {                                                     \
    ARRAY_ATTR                                                       \
    LAYOUT(ARRAY_ATTRIBUTES, FLOAT)                                  \
} BASE_NAME##Array##F_SUFFIX;

#define MAKE_STRUCTS_ALL_PRECISIONS(LAYOUT, BASE_NAME, ARRAY_ATTR) \
MAKE_STRUCTS(LAYOUT, BASE_NAME, ARRAY_ATTR, float, _s)             \
MAKE_STRUCTS(LAYOUT, BASE_NAME, ARRAY_ATTR, double, _d)

#define SYSTEM_LAYOUT(CB, ...) \
    CB(SOLID_DESC_LAYOUT, SolidDescription, solid_description, DESC, __VA_ARGS__) \
    CB(JOINT_DESC_LAYOUT, JointDescription, joint_description, DESC, __VA_ARGS__) \
    CB(SOLID_RES_LAYOUT, SolidResult, solid_result, RES, __VA_ARGS__)


#define RES_ALLOC_PARAM size_t obj_count, size_t frame_count
#define DESC_ALLOC_PARAM size_t obj_count

#define RES_INTER_PARAM size_t obj_index, size_t frame_index
#define DESC_INTER_PARAM size_t obj_index

#define RES_ARRAY_ATTR size_t obj_count; size_t frame_count;
#define DESC_ARRAY_ATTR size_t obj_count;

#define RES_SETTER(LAYOUT, NAME, ATTR_NAME, TYPE, FLOAT, F_SUFFIX)
#define DESC_SETTER(LAYOUT, NAME, ATTR_NAME, TYPE, FLOAT, F_SUFFIX)  \
uint32_t KINEPY_set_##ATTR_NAME##F_SUFFIX(System##F_SUFFIX * system, TYPE##_INTER_PARAM, NAME##F_SUFFIX const * input);


#define DECLARE_INTERFACE(LAYOUT, NAME, ATTR_NAME, TYPE, FLOAT, F_SUFFIX) \
uint32_t KINEPY_allocate_##ATTR_NAME##s##F_SUFFIX(System##F_SUFFIX * system, TYPE##_ALLOC_PARAM);                           \
void KINEPY_free_##ATTR_NAME##s##F_SUFFIX(System##F_SUFFIX * system);                                                   \
void KINEPY_get_##ATTR_NAME##F_SUFFIX(System##F_SUFFIX const * system, TYPE##_INTER_PARAM, NAME##F_SUFFIX * output);       \
TYPE##_SETTER(LAYOUT, NAME, ATTR_NAME, TYPE, FLOAT, F_SUFFIX)

#define DECLARE_INTERFACE_ALL_PRECISIONS(LAYOUT, BASE_NAME, ATTR_NAME, TYPE)    \
DECLARE_INTERFACE(LAYOUT, BASE_NAME, ATTR_NAME, TYPE, float, _s)                \
DECLARE_INTERFACE(LAYOUT, BASE_NAME, ATTR_NAME, TYPE, double, _d)

#define GENERATE_STRUCTS(LAYOUT, NAME, ATTR_NAME, TYPE, ...)                 \
MAKE_STRUCTS_ALL_PRECISIONS(LAYOUT, NAME, TYPE##_ARRAY_ATTR)

#define GENERATE_INTERFACE(LAYOUT, NAME, ATTR_NAME, TYPE, ...)                 \
DECLARE_INTERFACE_ALL_PRECISIONS(LAYOUT, NAME, ATTR_NAME, TYPE)


#define SYSTEM_ATTR(_LAYOUT, NAME, ATTR_NAME, _TYPE, F_SUFFIX, ...) NAME##Array##F_SUFFIX ATTR_NAME##_array;

typedef struct {
    size_t count;
    uint32_t * joints;
} JointArray;

typedef struct {
    uint32_t joint_index;
    uint32_t orientation;
} GraphStepEdge;

typedef struct {
    uint32_t isostatic_graph;
    uint32_t * eq_indices; // |eq_indices| = isostatic_graph->vertex_count+1
    uint32_t * eqs; // |eqs| = eq_indices[isostatic_graph->vertex_count]
    GraphStepEdge * edges; // |edges| = isostatic_graph->edge_count
    uint8_t solution_index; // solution_index < isostatic_graph->solution_count
} GraphStep;

typedef struct {
    uint32_t joint_index;
    uint32_t first_eq_size;
    uint32_t second_eq_size;
    uint32_t * eqs;
} JointStep;

typedef enum {
    STEP_TYPE_GRAPH,
    STEP_TYPE_JOINT,
    STEP_TYPE_RELATION
} StepType;

typedef struct {
    StepType type;
    union {
        GraphStep graph_step;
        JointStep joint_step;
    };
} ResolutionStep;

typedef struct {
    size_t count;
    ResolutionStep * array;
} StepArray;

typedef struct {
    JointArray piloted_or_blocked_joints;
    StepArray steps;
} ResolutionMode;

#define MAKE_SYSTEM(FLOAT, F_SUFFIX) \
typedef struct {                     \
    FLOAT length;                    \
    FLOAT mass;                      \
    FLOAT moment_of_inertia;         \
    FLOAT force;                     \
    FLOAT torque;                    \
    FLOAT dimensionless;             \
} UnitSystem##F_SUFFIX;              \
                                     \
typedef struct {                     \
    /* Don't put any attribute that varies in size with floating point precision above SYSTEM_LAYOUT(...) */ \
    SYSTEM_LAYOUT(SYSTEM_ATTR, F_SUFFIX)\
    ResolutionMode kinematics;       \
    ResolutionMode dynamics;         \
    UnitSystem##F_SUFFIX * unit_system; \
} System##F_SUFFIX;

SYSTEM_LAYOUT(GENERATE_STRUCTS)

MAKE_SYSTEM(float, _s)
MAKE_SYSTEM(double, _d)

#define MAKE_INTERNAL_SYSTEM(LAYOUT, NAME, ATTR_NAME, TYPE, ... ) \
struct {TYPE##_ARRAY_ATTR LAYOUT(ARRAY_ATTRIBUTES, void)} ATTR_NAME##_array;
typedef struct {
    SYSTEM_LAYOUT(MAKE_INTERNAL_SYSTEM)
} system_internal;

SYSTEM_LAYOUT(GENERATE_INTERFACE)

typedef enum {
    JOINT_TYPE_EMPTY,
    JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_PRISMATIC,
    JOINT_TYPE_COUNT
} JointType;

typedef enum {
    RELATION_TYPE_EMPTY,
    RELATION_TYPE_GEAR,
    RELATION_TYPE_GEAR_RACK,
    RELATION_TYPE_DISTANT,
    RELATION_TYPE_EFFORTLESS,
    RELATION_TYPE_COUNT
} RelationType;

#endif //LAYOUTS_H
