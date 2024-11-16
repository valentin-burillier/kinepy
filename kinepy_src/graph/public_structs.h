#ifndef GRAPH_STRUCTS_H
#define GRAPH_STRUCTS_H

#include "stdint.h"

typedef struct {
    uint32_t count;
    uint32_t * joints;
} JointArray;

typedef struct {
    uint32_t joint_index;
    uint32_t orientation;
} GraphStepEdge;

typedef struct {
    uint32_t isostatic_graph;
    uint32_t * eq_indices;
    uint32_t * eqs;
    GraphStepEdge * edges;
    uint8_t solution_index;
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
    uint32_t count;
    ResolutionStep * array;
} StepArray;

typedef struct {
    JointArray piloted_or_blocked_joints;
    StepArray steps;
} ResolutionMode;

#endif //GRAPH_STRUCTS_H
