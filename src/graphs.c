#include "graphs.h"


#ifdef SYMMETRIC_GRAPH_TYPE
/*
         0
        / \
       R   R
      /     \
     1 - R - 2
*/
JointType const GRAPH_RRR[] = {
    JOINT_TYPE_EMPTY, JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_EMPTY, JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE, JOINT_TYPE_EMPTY
};

/*
         0
        / \
       R   R
      /     \
     1 - P - 2
*/
JointType const GRAPH_RRP[] = {
    JOINT_TYPE_EMPTY, JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_EMPTY, JOINT_TYPE_PRISMATIC,
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_PRISMATIC, JOINT_TYPE_EMPTY
};


/*
         0
        / \
       P   P
      /     \
     1 - R - 2
*/
JointType const GRAPH_PPR[] = {
    JOINT_TYPE_EMPTY, JOINT_TYPE_PRISMATIC, JOINT_TYPE_PRISMATIC,
    JOINT_TYPE_PRISMATIC, JOINT_TYPE_EMPTY, JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_PRISMATIC, JOINT_TYPE_REVOLUTE, JOINT_TYPE_EMPTY
};

#endif

#ifdef TRIANGULAR_GRAPH_TYPE
/*
         0
        / \
       R   R
      /     \
     1 - R - 2
*/

JointType const GRAPH_RRR[] = {
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE
};

/*
         0
        / \
       R   R
      /     \
     1 - P - 2
*/
JointType const GRAPH_RRP[] = {
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE, JOINT_TYPE_PRISMATIC
};


/*
         0
        / \
       P   P
      /     \
     1 - R - 2
*/
JointType const GRAPH_PPR[] = {
    JOINT_TYPE_PRISMATIC, JOINT_TYPE_PRISMATIC, JOINT_TYPE_REVOLUTE
};

#endif


size_t const ISOSTATIC_GRAPH_VERTEX_COUNT[] = {
    3,
    3,
    3,
};

JointType const * ISOSTATIC_GRAPHS[] = {
    GRAPH_RRR,
    GRAPH_RRP,
    GRAPH_PPR
};


void make_graph(JointDescriptionArrayView const * const joint_array, size_t const solid_count, GraphNode * const graph) {
#ifdef TRIANGULAR_GRAPH_TYPE
    size_t const mark = NODE_COUNT(solid_count);
#endif
#ifdef SYMMETRIC_GRAPH_TYPE
    size_t const mark = solid_count;
#endif
    for (int index = 0; index < joint_array->count; index++) {
        JointType type = get_joint_description_type(joint_array, index);
        uint32_t solid1 = get_joint_description_solid1(joint_array, index);
        uint32_t solid2 = get_joint_description_solid1(joint_array, index);

        GraphNode const node = {
            .type = type,
            .joint_index = index
        };
#ifdef TRIANGULAR_GRAPH_TYPE
        graph[solid2 > solid1 ? GRAPH_INDEX(solid1, solid2, mark) : GRAPH_INDEX(solid2, solid1, mark)] = node;
#endif
#ifdef SYMMETRIC_GRAPH_TYPE
        graph[GRAPH_INDEX(solid1, solid2, mark)] = node;
        graph[GRAPH_INDEX(solid2, solid1, mark)] = node;
#endif
    }
}

void joint_degrees(GraphNode const * const graph, uint32_t const solid_count, JointDegree * const result) {
#ifdef TRIANGULAR_GRAPH_TYPE
    uint32_t index = 0;
    for (int x = 0; x < solid_count; ++x) {
        for (int y = x+1; y < solid_count; ++y) {
            if (graph[index].type) {
                ++result[x].arr[graph[index].type - 1];
                ++result[y].arr[graph[index].type - 1];
            }
            ++index;
        }
    }
#endif
#ifdef SYMMETRIC_GRAPH_TYPE
    uint32_t index;
    for (int x = 0; x < solid_count; ++x) {
        index = x * solid_count + x + 1;
        for (int y = x+1; y < solid_count; ++y) {
            if (graph[index].type) {
                ++result[x].arr[graph[index].type - 1];
                ++result[y].arr[graph[index].type - 1];
            }
            ++index;
        }
    }
#endif
}

size_t find_isomorphism(GraphNode const * const graph, uint32_t const solid_count, uint32_t ** const isomorphism) {

    return -1;
}


void determine_computation_order(System const * const system) {
    uint32_t const solid_count = system->solids.count;
    GraphNode * const solid_graph = malloc(sizeof(GraphNode) * NODE_COUNT(solid_count));
    if (!solid_graph) {
        return;
    }
    make_graph(&system->joints, solid_count, solid_graph);

    JointDegree * degrees = malloc(sizeof(JointDegree) * solid_count);
    if (!degrees) {
        free(solid_graph);
        return;
    }
    joint_degrees(solid_graph, solid_count, degrees);

    free(degrees);
    free(solid_graph);
}

