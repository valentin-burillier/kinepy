#ifndef PRIVATE_STRUCTS_H
#define PRIVATE_STRUCTS_H
#include "stdint.h"
#include "graph_data.h"

typedef struct {
    struct {
        uint32_t obj_count;
        void * solid_description_ptr;
    } solids;
    struct {
        uint32_t obj_count;
        uint32_t *solid1_ptr;
        uint32_t *solid2_ptr;
        uint8_t *type_ptr;
        void * constraint_ptr;
    } joints;
    struct {
        uint32_t obj_count;
        uint32_t *joint1_ptr;
        uint32_t *joint2_ptr;
        uint8_t *type_ptr;
        void * parameter_ptr;
    } relations;
    void * unit_system;
} system_internal;

typedef struct {
    uint8_t type;
    uint32_t joint_index;
} GraphNode;

typedef struct {
    uint32_t eq_count;
    uint32_t * eq_indices;
    uint32_t * eqs;
    uint32_t * solid_to_eq;

    GraphNode * adjacency;
    JointDegree * degrees;

    uint8_t * joint_state;
    uint32_t * joint_indices;
    uint32_t * joint_adjacency;
} Graph;

#endif //PRIVATE_STRUCTS_H
