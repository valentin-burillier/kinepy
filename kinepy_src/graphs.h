#ifndef GRAPHS_H
#define GRAPHS_H

#include "layouts.h"

typedef struct GraphNode {
    JointType type;
    uint32_t joint_index;
} GraphNode;

typedef union JointDegree {
    struct {
        char revolute;
        char prismatic;
    };
    char arr[2];
} JointDegree;

//#define SYMMETRIC_MATRIX_ADJACENCY_TYPE
#define adjacency_size(SOLID_COUNT) SOLID_COUNT * (SOLID_COUNT - 1) / 2
// prerequisite: X < Y
#define certain_order_graph_index(X, Y, ARRAY_SIZE) (X * (2 * ARRAY_SIZE - X - 3) / 2 + Y - 1)
#define graph_index(X, Y, ARRAY_SIZE) X < Y ? certain_order_graph_index(X, Y, ARRAY_SIZE) : certain_order_graph_index(Y, X, ARRAY_SIZE)

typedef char Edge[2];

typedef struct IsostaticGraphInfo {
    size_t const vertex_count;
    JointType const * const adjacency;
    JointDegree const * const degrees;
    size_t const edge_count;
    Edge const * const edges;
} IsostaticGraphInfo;


typedef enum IsostaticGraph {
    GRAPH_RRR,
    GRAPH_RRP,
    GRAPH_PPR,
    ISOSTATIC_GRAPH_COUNT
} IsostaticGraph;

extern IsostaticGraphInfo const ISOSTATIC_GRAPHS[ISOSTATIC_GRAPH_COUNT];

inline int compare_degrees(JointDegree d1, JointDegree d2);
void make_graph(system_internal const * system, size_t solid_count, GraphNode * graph);
void determine_computation_order(system_internal const * system);
void compute_joint_degrees(GraphNode const * graph, uint32_t solid_count, JointDegree * result);
GraphNode * merge_graph(GraphNode const * graph, uint32_t solid_count, uint32_t const * group_to_merge, uint32_t group_size);
uint8_t find_isomorphism_test_graph(int isostatic_graph_index, uint32_t * exploration_stack, uint32_t * vertex_shuffle, GraphNode const * graph, JointDegree const * degrees, uint32_t solid_count);
uint32_t find_isomorphism(GraphNode const * graph, JointDegree const * degrees, uint32_t solid_count, uint32_t ** result_isomorphism);

#endif //GRAPHS_H
