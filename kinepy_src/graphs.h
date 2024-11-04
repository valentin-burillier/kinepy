#ifndef GRAPHS_H
#define GRAPHS_H

#include "layouts.h"



#define KINEPY_check(FUNC_CALL) result = FUNC_CALL; if (result != KINEPY_SUCCESS)

#define malloc_array(type, item_count) malloc(sizeof(type) * (item_count))
#define set_alloc_array(name, type, item_count) name = malloc_array(type, item_count); if (!name) {result = KINEPY_MALLOC_FAILED; goto malloc_err;} ++allocated
#define declare_alloc_array(name, type, item_count, const_ness) type * const_ness name = malloc_array(type, item_count); if (!name) {result = KINEPY_MALLOC_FAILED; goto malloc_err;} ++allocated

typedef struct {
    JointType type;
    uint32_t joint_index;
} GraphNode;

typedef union {
    struct {
        char revolute;
        char prismatic;
    };
    char arr[2];
} JointDegree;

//#define SYMMETRIC_MATRIX_ADJACENCY_TYPE
#define adjacency_size(SOLID_COUNT) ((SOLID_COUNT) * ((SOLID_COUNT) - 1) / 2)
// prerequisite: X < Y
#define certain_order_graph_index(X, Y, ARRAY_SIZE) (X * (2 * ARRAY_SIZE - X - 3) / 2 + Y - 1)
#define graph_index(X, Y, ARRAY_SIZE) X < Y ? certain_order_graph_index(X, Y, ARRAY_SIZE) : certain_order_graph_index(Y, X, ARRAY_SIZE)
#define graph_node(graph, index_mode, X, Y) graph.adjacency[index_mode(X, Y, graph.eq_count)]
typedef uint8_t Edge[2];

typedef struct {
    uint32_t vertex_count;
    JointType const * adjacency;
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

typedef struct {
    uint32_t eq_count;
    GraphNode * adjacency;
    JointDegree * joint_degrees;

    uint32_t * eq_indices;
    uint32_t * eqs;
    uint32_t * solid_to_eq;
} Graph;

extern IsostaticGraphInfo const ISOSTATIC_GRAPHS[ISOSTATIC_GRAPH_COUNT];

void make_graph_adjacency(system_internal const * system, Graph * graph);
void compute_joint_degrees(Graph const * graph);
int compare_degrees(JointDegree d1, JointDegree d2);

uint8_t find_isomorphism_test_graph(int isostatic_graph_index, uint32_t * exploration_stack, uint32_t * vertex_shuffle, Graph const * graph);
uint32_t find_isomorphism(Graph const * graph, uint32_t * result_graph_index, uint32_t ** result_isomorphism);
uint32_t merge_graph(Graph * graph, uint32_t const * group_to_merge, uint32_t group_size);
uint32_t determine_computation_order(system_internal const * system, ResolutionMode * resolution_mode);



#endif //GRAPHS_H
