#include "graphs.h"
#include "string.h"

#ifdef SYMMETRIC_MATRIX_ADJACENCY_TYPE
/*
         0
        / \
       R   R
      /     \
     1 - R - 2
*/
JointType const GRAPH_RRR_ADJACENCY[] = {
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
JointType const GRAPH_RRP_ADJACENCY[] = {
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
JointType const GRAPH_PPR_ADJACENCY[] = {
    JOINT_TYPE_EMPTY, JOINT_TYPE_PRISMATIC, JOINT_TYPE_PRISMATIC,
    JOINT_TYPE_PRISMATIC, JOINT_TYPE_EMPTY, JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_PRISMATIC, JOINT_TYPE_REVOLUTE, JOINT_TYPE_EMPTY
};

#endif

#ifdef UPPER_TRIANGULAR_ADJACENCY_TYPE
/*
         0
        / \
       R   R
      /     \
     1 - R - 2
*/

JointType const GRAPH_RRR_ADJACENCY[] = {
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE
};

/*
         0
        / \
       R   R
      /     \
     1 - P - 2
*/
JointType const GRAPH_RRP_ADJACENCY[] = {
    JOINT_TYPE_REVOLUTE, JOINT_TYPE_REVOLUTE, JOINT_TYPE_PRISMATIC
};


/*
         0
        / \
       P   P
      /     \
     1 - R - 2
*/
JointType const GRAPH_PPR_ADJACENCY[] = {
    JOINT_TYPE_PRISMATIC, JOINT_TYPE_PRISMATIC, JOINT_TYPE_REVOLUTE
};

#endif

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
        .mark = GRAPH_MARK(3),
        .degrees = GRAPH_RRR_DEGREES,
        .edge_count = sizeof(DYAD_EDGES) / sizeof(Edge),
        .edges = DYAD_EDGES
    }, {
        .vertex_count = 3,
        .adjacency = GRAPH_RRP_ADJACENCY,
        .mark = GRAPH_MARK(3),
        .degrees = GRAPH_RRP_DEGREES,
        .edge_count = sizeof(DYAD_EDGES) / sizeof(Edge),
        .edges = DYAD_EDGES
    },{
        .vertex_count = 3,
        .adjacency = GRAPH_PPR_ADJACENCY,
        .mark = GRAPH_MARK(3),
        .degrees = GRAPH_PPR_DEGREES,
        .edge_count = sizeof(DYAD_EDGES) / sizeof(Edge),
        .edges = DYAD_EDGES
    }
};

void make_graph(JointDescriptionArrayView const * const joint_array, size_t const solid_count, GraphNode * const graph) {
    size_t const mark = GRAPH_MARK(solid_count);

    for (int index = 0; index < joint_array->count; index++) {
        JointType type = get_joint_description_type(joint_array, index);
        uint32_t solid1 = get_joint_description_solid1(joint_array, index);
        uint32_t solid2 = get_joint_description_solid2(joint_array, index);

        GraphNode const node = {
            .type = type,
            .joint_index = index
        };

        graph[graph_index(solid1, solid2, mark)] = node;
#ifdef SYMMETRIC_MATRIX_ADJACENCY_TYPE
        graph[graph_index(solid2, solid1, mark)] = node;
#endif
    }
}

void compute_joint_degrees(GraphNode const * const graph, uint32_t const solid_count, JointDegree * const result) {
#ifdef UPPER_TRIANGULAR_ADJACENCY_TYPE
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
#ifdef SYMMETRIC_MATRIX_ADJACENCY_TYPE
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

/**
 * Graph isomorphism vertex eligibility heuristic
 * false: compared vertices cannot be isomorphic
 * true: compared vertices could be isomorphic
 * @param d1 smaller graph vertex JointDegree
 * @param d2 bigger graph vertex JointDegree
 * @return d1 <= d2
 */
inline int compare_degrees(JointDegree const d1, JointDegree const d2) {
    return (d1.prismatic <= d2.prismatic) && (d1.revolute <= d2.revolute);
}

int can_match(uint32_t const * const vertex_shuffle, uint32_t const isomorphism_stage, uint32_t const new_vertex, GraphNode const * const user_graph, uint32_t const user_graph_mark, IsostaticGraphInfo const * const isostatic_graph) {
    uint32_t isostatic_vertex_index = isomorphism_stage;
    while (isostatic_vertex_index) {
        --isostatic_vertex_index;
        if (isostatic_graph->adjacency[certain_order_graph_index(isostatic_vertex_index, isomorphism_stage, isostatic_graph->mark)] != user_graph[graph_index(new_vertex, vertex_shuffle[isostatic_vertex_index], user_graph_mark)].type) {
            return 0;
        }
    }
    return 1;
}

uint32_t find_isomorphism(GraphNode const * const graph, JointDegree const * const degrees, uint32_t const solid_count, uint32_t ** const result) {
    uint32_t * const vertex_shuffle = malloc(solid_count * sizeof(uint32_t));
    uint32_t * const exploration_stack = malloc(solid_count * sizeof(uint32_t));
    *result = 0;

    if (!vertex_shuffle) {
        return -1;
    }
    if (!exploration_stack) {
        free(vertex_shuffle);
        return -1;
    }
    for (uint32_t i = 0; i < solid_count; ++i) {
        vertex_shuffle[i] = i;
        exploration_stack[i] = i;
    }

    uint32_t user_graph_mark = GRAPH_MARK(solid_count);

    for (int isostatic_graph_index = 0; isostatic_graph_index < ISOSTATIC_GRAPH_NUMBER; ++isostatic_graph_index) {
        size_t isomorphism_stage = 0;
        IsostaticGraphInfo const * const target_graph = ISOSTATIC_GRAPHS + isostatic_graph_index;

        while (isomorphism_stage < target_graph->vertex_count && exploration_stack[0] < solid_count) {

            /**
             * Shuffle index >= isomorphism_stage
             * Current partial isomorphism is vertex_shuffle[:isomorphism_stage] (upper bound is excluded)
             */
            uint32_t shuffle_index;
            for (shuffle_index = exploration_stack[isomorphism_stage]; shuffle_index < solid_count; shuffle_index++) {
                uint32_t vertex_index = vertex_shuffle[shuffle_index];

                // vertex is not eligible
                if (!compare_degrees(target_graph->degrees[isomorphism_stage], degrees[vertex_index]) || !can_match(vertex_shuffle, isomorphism_stage, vertex_index, graph, user_graph_mark, target_graph)) {
                    continue;
                }

                // push vertex to the stack
                exploration_stack[isomorphism_stage] = shuffle_index;

                vertex_shuffle[shuffle_index] = vertex_shuffle[isomorphism_stage];
                vertex_shuffle[isomorphism_stage] = vertex_index;

                ++isomorphism_stage;
                break;
            }
            // no vertex is eligible
            if (shuffle_index == solid_count) {
                // pop the stack
                --isomorphism_stage;
                shuffle_index = exploration_stack[isomorphism_stage];

                // TODO: Edge case isomorphism_stage == 0; Assumption: no problem
                vertex_shuffle[isomorphism_stage] ^= vertex_shuffle[shuffle_index];
                vertex_shuffle[shuffle_index] ^= vertex_shuffle[isomorphism_stage];
                vertex_shuffle[isomorphism_stage] ^= vertex_shuffle[shuffle_index];

                ++exploration_stack[isomorphism_stage];
            }
        }
        // valid isomorphism
        if (isomorphism_stage == target_graph->vertex_count) {
            *result = malloc(target_graph->vertex_count * sizeof(uint32_t));
            if (*result) {
                memcpy(*result, vertex_shuffle, target_graph->vertex_count * sizeof(uint32_t));
            }

            free(vertex_shuffle);
            free(exploration_stack);

            return isostatic_graph_index;
        }
    }
    // no valid isomorphism is found
    free(vertex_shuffle);
    free(exploration_stack);
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
    compute_joint_degrees(solid_graph, solid_count, degrees);

    free(degrees);
    free(solid_graph);
}

