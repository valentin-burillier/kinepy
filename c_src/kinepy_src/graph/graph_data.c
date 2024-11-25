#include "graph_data.h"
#include "interface/configuration.h"

/*
         0
        / \
       R   R
      /     \
     1 - R - 2
*/

uint8_t const GRAPH_RRR_ADJACENCY[] = {
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE
};

/*
         0
        / \
       R   R
      /     \
     1 - P - 2
*/
uint8_t const GRAPH_RRP_ADJACENCY[] = {
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE, JOINT_TYPE_PRISMATIC
};


/*
         0
        / \
       P   P
      /     \
     1 - R - 2
*/
uint8_t const GRAPH_PPR_ADJACENCY[] = {
    JOINT_TYPE_PRISMATIC, JOINT_TYPE_PRISMATIC, JOINT_TYPE_REVOLUTE
};

Edge const DYAD_EDGES[] = {
    {0, 1},
    {0, 2},
    {1, 2}
};

JointDegree const GRAPH_RRR_DEGREES[] = {
    {.revolute=2, .prismatic=0},
    {.revolute=2, .prismatic=0},
    {.revolute=2, .prismatic=0}
};
JointDegree const GRAPH_RRP_DEGREES[] = {
    {.revolute=2, .prismatic=0},
    {.revolute=1, .prismatic=1},
    {.revolute=1, .prismatic=1}
};
JointDegree const GRAPH_PPR_DEGREES[] = {
    {.revolute=0, .prismatic=2},
    {.revolute=1, .prismatic=1},
    {.revolute=1, .prismatic=1}
};

IsostaticGraphInfo const ISOSTATIC_GRAPHS[] = {
    {
        .vertex_count = 3,
        .adjacency = GRAPH_RRR_ADJACENCY,
        .degrees = GRAPH_RRR_DEGREES,
        .edge_count = sizeof(DYAD_EDGES) / sizeof(Edge),
        .edges = DYAD_EDGES,
        .solution_count = 2
    }, {
        .vertex_count = 3,
        .adjacency = GRAPH_RRP_ADJACENCY,
        .degrees = GRAPH_RRP_DEGREES,
        .edge_count = sizeof(DYAD_EDGES) / sizeof(Edge),
        .edges = DYAD_EDGES,
        .solution_count = 2
    },{
        .vertex_count = 3,
        .adjacency = GRAPH_PPR_ADJACENCY,
        .degrees = GRAPH_PPR_DEGREES,
        .edge_count = sizeof(DYAD_EDGES) / sizeof(Edge),
        .edges = DYAD_EDGES,
        .solution_count = 1
    }
};