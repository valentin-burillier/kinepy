#ifndef GRAPHS_H
#define GRAPHS_H

#include "system.h"

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


#define UPPER_TRIANGULAR_ADJACENCY_TYPE
//#define SYMMETRIC_MATRIX_ADJACENCY_TYPE

#ifdef SYMMETRIC_MATRIX_ADJACENCY_TYPE
#undef UPPER_TRIANGULAR_ADJACENCY_TYPE

#define NODE_COUNT(SOLID_COUNT) SOLID_COUNT * SOLID_COUNT
#define certain_order_graph_index(X, Y, NODES) ((X) + (Y) * (NODES))
#define graph_index certain_order_graph_index
#define GRAPH_MARK(SOLID_COUNT) (SOLID_COUNT)
#else
    #ifndef TRIANGULAR_GRAPH_TYPE
        #define TRIANGULAR_GRAPH_TYPE
    #endif
#endif

#ifdef UPPER_TRIANGULAR_ADJACENCY_TYPE

#define NODE_COUNT(SOLID_COUNT) SOLID_COUNT * (SOLID_COUNT - 1) / 2
// prerequisite: X < Y
#define certain_order_graph_index(X, Y, ARRAY_SIZE) (ARRAY_SIZE - (X - 3) * X / 2 + Y - 1)
#define graph_index(X, Y, ARRAY_SIZE) X < Y ? certain_order_graph_index(X, Y, ARRAY_SIZE) : certain_order_graph_index(Y, X, ARRAY_SIZE)
#define GRAPH_MARK(SOLID_COUNT) NODE_COUNT(SOLID_COUNT)

#endif
typedef char Edge[2];

typedef struct IsostaticGraphInfo {
    size_t const vertex_count;
    size_t const mark;
    JointType const * const adjacency;
    JointDegree const * const degrees;
    size_t const edge_count;
    Edge const * const edges;
} IsostaticGraphInfo;

extern JointType const GRAPH_RRR_ADJACENCY[NODE_COUNT(3)];
extern JointType const GRAPH_RRP_ADJACENCY[NODE_COUNT(3)];
extern JointType const GRAPH_PPR_ADJACENCY[NODE_COUNT(3)];

extern Edge const DYAD_EDGES[3];

extern JointDegree const GRAPH_RRR_DEGREES[3];
extern JointDegree const GRAPH_RRP_DEGREES[3];
extern JointDegree const GRAPH_PPR_DEGREES[3];

#define ISOSTATIC_GRAPH_NUMBER 3
extern IsostaticGraphInfo const ISOSTATIC_GRAPHS[ISOSTATIC_GRAPH_NUMBER];


void make_graph(JointDescriptionArrayView const * joint_array, size_t solid_count, GraphNode * graph);
void determine_computation_order(System const * system);
void compute_joint_degrees(GraphNode const * graph, uint32_t solid_count, JointDegree * result);
uint32_t void find_isomorphism(GraphNode const * graph, JointDegree const * degrees, uint32_t solid_count, uint32_t ** result);

#endif //GRAPHS_H
