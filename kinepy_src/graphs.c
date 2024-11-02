#include "string.h"
#include "graphs.h"
#include "stdlib.h"
#include "stdio.h"


void make_graph(system_internal const * const system, size_t const solid_count, GraphNode * const graph) {
    for (int index = 0; index < system->joint_description_array.obj_count; index++) {
        JointType type = system->joint_description_array.type_ptr[index];
        uint32_t solid1 = system->joint_description_array.solid1_ptr[index];
        uint32_t solid2 = system->joint_description_array.solid2_ptr[index];

        GraphNode const node = {
            .type = type,
            .joint_index = index
        };

        graph[graph_index(solid1, solid2, solid_count)] = node;
    }
}

void compute_joint_degrees(GraphNode const * const graph, uint32_t const solid_count, JointDegree * const result) {
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
}

/**
 * Graph isomorphism vertex eligibility heuristic
 * false: compared vertices cannot be isomorphic
 * true: compared vertices could be isomorphic
 * @param d1 smaller graph vertex JointDegree
 * @param d2 bigger graph vertex JointDegree
 * @return d1 <= d2
 */
int compare_degrees(JointDegree const d1, JointDegree const d2) {
    return (d1.prismatic <= d2.prismatic) && (d1.revolute <= d2.revolute);
}

/**
 * Graph isomorphism vertex eligibility, check if new_vertex has the same adjacency in the current sub-graph than its counter-part in the isostatic_graph
 * precondition: previous vertices are eligible
 * @param vertex_shuffle
 * @param isomorphism_stage
 * @param new_vertex
 * @param user_graph
 * @param solid_count
 * @param isostatic_graph
 * @return
 */
int can_match(uint32_t const * const vertex_shuffle, uint32_t const isomorphism_stage, uint32_t const new_vertex, GraphNode const * const user_graph, uint32_t const solid_count, IsostaticGraphInfo const * const isostatic_graph) {
    uint32_t isostatic_vertex_index = isomorphism_stage;
    while (isostatic_vertex_index) {
        --isostatic_vertex_index;
        if (isostatic_graph->adjacency[certain_order_graph_index(isostatic_vertex_index, isomorphism_stage, isostatic_graph->vertex_count)] != user_graph[graph_index(new_vertex, vertex_shuffle[isostatic_vertex_index], solid_count)].type) {
            return 0;
        }
    }
    return 1;
}



void merge_eqs(uint32_t * const eqs, uint32_t * const eq_indices, uint32_t const eq_count, uint32_t const * const group_to_merge, uint32_t const group_size) {
    uint32_t const solid_count = eq_indices[eq_count];
    uint8_t * const merge_state = malloc(eq_count * sizeof(uint8_t));
    if (!merge_state) {
        return;
    }
    for (int index = 0; index < eq_count; ++index) {
        merge_state[index] = 0;
    }

    uint32_t not_merged_displacement = 0;
    for (int index = 0; index < group_size; ++index) {
        merge_state[group_to_merge[index]] = 1;
        not_merged_displacement += eq_indices[group_to_merge[index]+1] - eq_indices[group_to_merge[index]];
    }
    uint32_t const new_eq_count = eq_count - group_size + 1;
    uint32_t * const new_eqs = malloc(eq_indices[eq_count] * sizeof(uint32_t));
    if (!new_eqs) {
        free(merge_state);
        return;
    }

    uint32_t merged_displacement = 0;
    uint32_t solid_index = 0;
    uint8_t merged_count = 0;
    for (int eq_index = 0; eq_index < eq_count; ++eq_index) {
        uint32_t displacement = 0;
        merged_count += merge_state[eq_index];
        if (merge_state[eq_index]) {
            not_merged_displacement -= eq_indices[eq_index+1] - eq_indices[eq_index];
            displacement = -merged_displacement;
        } else {
            if (merged_count) {
                merged_displacement += eq_indices[eq_index + 1] - eq_indices[eq_index];
                displacement = not_merged_displacement;
            }
        }

        for (; solid_index < eq_indices[eq_index] + 1; ++solid_index) {
            new_eqs[solid_index + displacement] = eqs[solid_index];
        }
        if (merged_count && !merge_state[eq_index]) {
            eq_indices[eq_index - merged_count + 1] = eq_indices[eq_index] + displacement;
        }
    }
    eq_indices[new_eq_count] = solid_count;

    memcpy(eqs, new_eqs, solid_count * sizeof(uint32_t));
    free(new_eqs);
    free(merge_state);
}

void merge_graph(GraphNode * const graph, uint32_t const eq_count, uint32_t const * const group_to_merge, uint32_t const group_size) {
    uint8_t * const merge_state = malloc(eq_count * sizeof(uint8_t));
    if (!merge_state) {
        return;
    }
    uint32_t group_min = *group_to_merge;
    for (int index = 0; index < eq_count; ++index) {
        merge_state[index] = 0;
    }

    for (int index = 0; index < group_size; ++index) {
        merge_state[group_to_merge[index]] = 1;
        if (group_to_merge[index] < group_min) {
            group_min = group_to_merge[index];
        }
    }
    uint32_t const new_graph_size = eq_count - group_size + 1;
    GraphNode * const new_graph = malloc(adjacency_size(new_graph_size));
    if (!new_graph) {
        free(merge_state);
        return;
    }
    for (int index = 0; index < adjacency_size(new_graph_size); index++) {
        new_graph[index].type = JOINT_TYPE_EMPTY;
        new_graph[index].joint_index = -1;
    }

    uint32_t merge_members = 0;
    for (int x = 0; x < eq_count; ++x) {
        merge_members += merge_state[x] - (x == group_min);
        uint32_t const old_merge_members = merge_members;
        uint32_t const new_x = merge_state[x] ? group_min : x - merge_members;

        for (int y = x+1; y < eq_count; ++y) {
            merge_members += merge_state[y]  - (y == group_min);
            uint32_t const new_y = merge_state[y] ? group_min : y - merge_members;
            if (new_x == new_y || graph[certain_order_graph_index(x, y, eq_count)].type == JOINT_TYPE_EMPTY) {
                continue;
            }
            new_graph[graph_index(new_x, new_y, new_graph_size)] = graph[certain_order_graph_index(x, y, eq_count)];
        }

        merge_members = old_merge_members;
    }
    memcpy(graph, new_graph, adjacency_size(new_graph_size) * sizeof(GraphNode));
    free(merge_state);
    free(new_graph);
}

uint8_t find_isomorphism_test_graph(int const isostatic_graph_index, uint32_t * const exploration_stack, uint32_t * const vertex_shuffle, GraphNode const * const graph, JointDegree const * const degrees, uint32_t const solid_count) {
    size_t isomorphism_stage = 0;
    IsostaticGraphInfo const * const target_graph = ISOSTATIC_GRAPHS + isostatic_graph_index;

    while (isomorphism_stage < target_graph->vertex_count && (isomorphism_stage != 0 || exploration_stack[0] < solid_count)) {
        if (exploration_stack[isomorphism_stage] < solid_count) {
            uint32_t const shuffle_index = exploration_stack[isomorphism_stage];
            uint32_t const vertex_index = vertex_shuffle[shuffle_index];
            ++exploration_stack[isomorphism_stage];

            // vertex is not eligible
            if (!compare_degrees(target_graph->degrees[isomorphism_stage], degrees[vertex_index]) || !can_match(vertex_shuffle, isomorphism_stage, vertex_index, graph, solid_count, target_graph)) {
                continue;
            }
            // push vertex
            if (shuffle_index != isomorphism_stage) {
                vertex_shuffle[shuffle_index] = vertex_shuffle[isomorphism_stage];
                vertex_shuffle[isomorphism_stage] = vertex_index;
            }
            ++isomorphism_stage;
        } else {
            // pop the stack
            exploration_stack[isomorphism_stage] = isomorphism_stage;
            --isomorphism_stage;

            uint32_t exchange_index = exploration_stack[isomorphism_stage] - 1;
            if (exploration_stack[isomorphism_stage] - 1 != isomorphism_stage) {
                vertex_shuffle[exchange_index] ^= vertex_shuffle[isomorphism_stage];
                vertex_shuffle[isomorphism_stage] ^= vertex_shuffle[exchange_index];
                vertex_shuffle[exchange_index] ^= vertex_shuffle[isomorphism_stage];
            }
        }
    }
    exploration_stack[0] = 0;
    return isomorphism_stage == target_graph->vertex_count;
}

uint32_t find_isomorphism(GraphNode const * const graph, JointDegree const * const degrees, uint32_t const solid_count, uint32_t ** const result_isomorphism) {
    uint32_t * const vertex_shuffle = malloc(solid_count * sizeof(uint32_t));
    uint32_t * const exploration_stack = malloc(solid_count * sizeof(uint32_t));
    uint32_t result_graph_index = -1;
    *result_isomorphism = 0;

    if (!vertex_shuffle) {
        return result_graph_index;
    }
    if (!exploration_stack) {
        free(vertex_shuffle);
        return result_graph_index;
    }
    for (uint32_t i = 0; i < solid_count; ++i) {
        vertex_shuffle[i] = i;
        exploration_stack[i] = i;
    }

    for (int isostatic_graph_index = 0; isostatic_graph_index < ISOSTATIC_GRAPH_COUNT && solid_count >= ISOSTATIC_GRAPHS[isostatic_graph_index].vertex_count; ++isostatic_graph_index) {
        IsostaticGraphInfo const * const target_graph = ISOSTATIC_GRAPHS + isostatic_graph_index;
        if (!find_isomorphism_test_graph(isostatic_graph_index, exploration_stack, vertex_shuffle, graph, degrees, solid_count)) {
            continue;
        }
        *result_isomorphism = malloc(target_graph->vertex_count * sizeof(uint32_t));
        if (*result_isomorphism) {
            memcpy(*result_isomorphism, vertex_shuffle, target_graph->vertex_count * sizeof(uint32_t));
            result_graph_index = isostatic_graph_index;
        }
        break;
    }
    free(vertex_shuffle);
    free(exploration_stack);
    return result_graph_index;
}


void determine_computation_order(system_internal const * const system) {
    uint32_t const solid_count = system->solid_description_array.obj_count;
    GraphNode * current_graph = malloc(sizeof(GraphNode) * adjacency_size(solid_count));
    if (!current_graph) {
        return;
    }
    make_graph(system, solid_count, current_graph);

    JointDegree * degrees = malloc(sizeof(JointDegree) * solid_count);
    if (!degrees) {
        free(current_graph);
        return;
    }
    compute_joint_degrees(current_graph, solid_count, degrees);

    uint32_t * eqs = malloc(solid_count * sizeof(uint32_t));
    if (!eqs) {
        free(current_graph);
        free(degrees);
        return;
    }
    uint32_t * eq_indices = malloc((solid_count+1) * sizeof(uint32_t));
    if (!eq_indices) {
        free(current_graph);
        free(degrees);
        free(eqs);
        return;
    }

    /**
     * At the beginning, every solid is its own equivalence class
     */
    for (int index = 0; index < solid_count; ++index) {
        eqs[index] = index;
        eq_indices[index] = index;
    }
    eq_indices[solid_count] = solid_count;

    uint32_t eq_count = solid_count;
    while (solid_count > 1) {
        uint32_t * isomorphism;
        uint32_t isostatic_graph = find_isomorphism(current_graph, degrees, eq_count, &isomorphism);
        if (isostatic_graph == -1) {
            break;
        }
        merge_graph(current_graph, eq_count, isomorphism, ISOSTATIC_GRAPHS[isostatic_graph].vertex_count);
        free(isomorphism);

        eq_count = eq_count - ISOSTATIC_GRAPHS[isostatic_graph].vertex_count + 1;
        compute_joint_degrees(current_graph, eq_count, degrees);
    }

    free(degrees);
    free(current_graph);
    free(eqs);
    free(eq_indices);
}
