#ifndef GRAPHS_H
#define GRAPHS_H

#include "system.h"

typedef struct GraphNode {
    JointType type;
    uint32_t joint_index;
} GraphNode;

//#define TRIANGULAR_GRAPH_TYPE
#define SYMMETRIC_GRAPH_TYPE

#ifdef SYMMETRIC_GRAPH_TYPE
#undef TRIANGULAR_GRAPH_TYPE

#define NODE_COUNT(SOLID_COUNT) SOLID_COUNT * SOLID_COUNT
#define GRAPH_INDEX(X, Y, NODES) ((X) + (Y) * (NODES))
#else
    #ifndef TRIANGULAR_GRAPH_TYPE
        #define TRIANGULAR_GRAPH_TYPE
    #endif
#endif

#ifdef TRIANGULAR_GRAPH_TYPE

#define NODE_COUNT(SOLID_COUNT) SOLID_COUNT * (SOLID_COUNT - 1) / 2
// prerequisite: X < Y
#define GRAPH_INDEX(X, Y, ARRAY_SIZE) (ARRAY_SIZE - (X - 3) * X / 2 + Y - 1)

#endif

extern JointType const GRAPH_RRR[NODE_COUNT(3)];
extern JointType const GRAPH_RRP[NODE_COUNT(3)];
extern JointType const GRAPH_PPR[NODE_COUNT(3)];

#define ISOSTATIC_GRAPH_NUMBER 3
extern size_t const ISOSTATIC_GRAPH_VERTEX_COUNT[ISOSTATIC_GRAPH_NUMBER];
extern JointType const * ISOSTATIC_GRAPHS[ISOSTATIC_GRAPH_NUMBER];


void make_graph(JointDescriptionArrayView const * joint_array, size_t solid_count, GraphNode * graph);
void determine_computation_order(System const * system);
size_t find_isomorphism(GraphNode const * graph, uint32_t solid_count, uint32_t ** isomorphism);

typedef union JointDegree {
    struct {
        char revolute;
        char prismatic;
    };
    char arr[2];
} JointDegree;

void joint_degrees(GraphNode const * graph, uint32_t solid_count, JointDegree * result);

#endif //GRAPHS_H
