#ifndef GRAPH_DATA_H
#define GRAPH_DATA_H

#include "stdint.h"

#define adjacency_size(SOLID_COUNT) ((SOLID_COUNT) * ((SOLID_COUNT) - 1) / 2)
#define certain_order_graph_index(X, Y, ARRAY_SIZE) (X * (2 * ARRAY_SIZE - X - 3) / 2 + Y - 1)
#define graph_index(X, Y, ARRAY_SIZE) X < Y ? certain_order_graph_index(X, Y, ARRAY_SIZE) : certain_order_graph_index(Y, X, ARRAY_SIZE)

#define check(FUNC_CALL) result = FUNC_CALL; if (result != KINEPY_SUCCESS)
#define malloc_array(type, item_count) malloc(sizeof(type) * (item_count))
#define set_alloc_array(name, item_count) name = malloc(sizeof(*name) * (item_count)); if (!name) {result = KINEPY_MALLOC_FAILED; goto malloc_err;} ++allocated
#define declare_alloc_array(name, type, item_count, const_ness) type * const_ness name = malloc_array(type, item_count); if (!name) {result = KINEPY_MALLOC_FAILED; goto malloc_err;} ++allocated

typedef union {
    struct {
        char revolute;
        char prismatic;
    };
    char arr[2];
} JointDegree;

typedef uint8_t Edge[2];

typedef struct {
    uint32_t vertex_count;
    uint8_t const * adjacency;
    JointDegree const * degrees;
    uint32_t edge_count;
    Edge const * edges;
    uint8_t solution_count;
} IsostaticGraphInfo;

typedef enum {
    GRAPH_RRR,
    GRAPH_RRP,
    GRAPH_PPR,
    ISOSTATIC_GRAPH_COUNT,
} IsostaticGraph;

extern IsostaticGraphInfo const ISOSTATIC_GRAPHS[ISOSTATIC_GRAPH_COUNT];


#endif //GRAPH_DATA_H
