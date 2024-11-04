#include "string.h"
#include "graphs.h"
#include "stdlib.h"
#include "stdio.h"


void make_graph_adjacency(system_internal const * const system, Graph * const graph) {
    for (int index = 0; index < system->joint_description_array.obj_count; index++) {
        JointType type = system->joint_description_array.type_ptr[index];
        uint32_t solid1 = system->joint_description_array.solid1_ptr[index];
        uint32_t solid2 = system->joint_description_array.solid2_ptr[index];

        GraphNode const node = {
            .type = type,
            .joint_index = index
        };

        graph->adjacency[graph_index(solid1, solid2, graph->eq_count)] = node;
    }
}

void compute_joint_degrees(Graph const * const graph) {
    uint32_t index = 0;
    for (int x = 0; x < graph->eq_count; ++x) {
        for (int y = x+1; y < graph->eq_count; ++y) {
            if (graph->adjacency[index].type) {
                ++(graph->joint_degrees[x].arr[graph->adjacency[index].type - 1]);
                ++(graph->joint_degrees[y].arr[graph->adjacency[index].type - 1]);
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
inline int compare_degrees(JointDegree const d1, JointDegree const d2) {
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
int can_match(uint32_t const * const vertex_shuffle, uint32_t const isomorphism_stage, uint32_t const new_vertex, Graph const * const user_graph, IsostaticGraphInfo const * const isostatic_graph) {
    uint32_t isostatic_vertex_index = isomorphism_stage;
    while (isostatic_vertex_index) {
        --isostatic_vertex_index;
        if (isostatic_graph->adjacency[certain_order_graph_index(isostatic_vertex_index, isomorphism_stage, isostatic_graph->vertex_count)] != user_graph->adjacency[graph_index(new_vertex, vertex_shuffle[isostatic_vertex_index], user_graph->eq_count)].type) {
            return 0;
        }
    }
    return 1;
}


uint8_t find_isomorphism_test_graph(int const isostatic_graph_index, uint32_t * const exploration_stack, uint32_t * const vertex_shuffle, Graph const * const graph) {
    size_t isomorphism_stage = 0;
    IsostaticGraphInfo const * const target_graph = ISOSTATIC_GRAPHS + isostatic_graph_index;

    while (isomorphism_stage < target_graph->vertex_count && (isomorphism_stage != 0 || exploration_stack[0] < graph->eq_count)) {
        if (exploration_stack[isomorphism_stage] < graph->eq_count) {
            uint32_t const shuffle_index = exploration_stack[isomorphism_stage];
            uint32_t const vertex_index = vertex_shuffle[shuffle_index];
            ++exploration_stack[isomorphism_stage];

            // vertex is not eligible
            if (!compare_degrees(target_graph->degrees[isomorphism_stage], graph->joint_degrees[vertex_index]) || !can_match(vertex_shuffle, isomorphism_stage, vertex_index, graph, target_graph)) {
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

uint32_t find_isomorphism(Graph const * const graph, uint32_t * const result_graph_index, uint32_t ** const result_isomorphism) {
    uint32_t result = KINEPY_NO_GRAPH_FOUND;
    uint8_t allocated = 0;

    declare_alloc_array(vertex_shuffle, uint32_t, graph->eq_count, const);
    declare_alloc_array(exploration_stack, uint32_t, graph->eq_count, const);

    *result_graph_index = ISOSTATIC_GRAPH_COUNT;
    *result_isomorphism = 0;
    static size_t const json = sizeof(IsostaticGraph);
    for (uint32_t i = 0; i < graph->eq_count; ++i) {
        vertex_shuffle[i] = i;
        exploration_stack[i] = i;
    }

    for (int isostatic_graph_index = 0; isostatic_graph_index < ISOSTATIC_GRAPH_COUNT && graph->eq_count >= ISOSTATIC_GRAPHS[isostatic_graph_index].vertex_count; ++isostatic_graph_index) {
        IsostaticGraphInfo const * const target_graph = ISOSTATIC_GRAPHS + isostatic_graph_index;
        if (!find_isomorphism_test_graph(isostatic_graph_index, exploration_stack, vertex_shuffle, graph)) {
            continue;
        }
        *result_isomorphism = malloc_array(uint32_t, target_graph->vertex_count);
        if (*result_isomorphism) {
            memcpy(*result_isomorphism, vertex_shuffle, target_graph->vertex_count * sizeof(uint32_t));
            *result_graph_index = isostatic_graph_index;
            result = KINEPY_SUCCESS;
        } else {
            result = KINEPY_MALLOC_FAILED;
        }
        break;
    }

malloc_err:
    switch (2 - allocated) {
        case 0:
            free(exploration_stack);
        case 1:
            free(vertex_shuffle);
        case 2:
        default:
            break;
    }
    return result;
}


uint32_t merge_graph(Graph * const graph, uint32_t const * group_to_merge, uint32_t group_size) {
    uint32_t result = KINEPY_SUCCESS;
    uint32_t allocated = 0;

    uint32_t const solid_count = graph->eq_indices[graph->eq_count];

    // which eqs are being merged
    declare_alloc_array(merge_state, uint8_t, graph->eq_count, const);

    uint32_t group_min = *group_to_merge;
    for (int index = 0; index < graph->eq_count; ++index) {
        merge_state[index] = 0;
    }
    uint32_t not_merged_displacement = 0;
    uint32_t merged_displacement = 0;
    for (int index = 0; index < group_size; ++index) {
        merge_state[group_to_merge[index]] = 1;
        // compute number of solids in merged group
        not_merged_displacement += graph->eq_indices[group_to_merge[index]+1] - graph->eq_indices[group_to_merge[index]];
        // find minimal eq_index
        if (group_to_merge[index] < group_min) {
            group_min = group_to_merge[index];
        }
    }
    uint32_t const new_eq_count = graph->eq_count - group_size + 1;
    declare_alloc_array(new_adjacency, GraphNode, adjacency_size(new_eq_count), const);

    for (int index = 0; index < adjacency_size(new_eq_count); index++) {
        new_adjacency[index].type = JOINT_TYPE_EMPTY;
        new_adjacency[index].joint_index = -1;
    }

    declare_alloc_array(new_eqs, uint32_t, solid_count, const);

    uint32_t merge_members = 0;
    uint32_t solid_index = 0;
    for (int x = 0; x < graph->eq_count; ++x) {
        merge_members += merge_state[x] - (x == group_min);
        uint32_t const old_merge_members = merge_members;
        uint32_t const new_x = merge_state[x] ? group_min : x - merge_members;

#pragma region Adjacency part
        for (int y = x+1; y < graph->eq_count; ++y) {
            merge_members += merge_state[y] - (y == group_min);
            uint32_t const new_y = merge_state[y] ? group_min : y - merge_members;
            if (new_x == new_y || graph->adjacency[certain_order_graph_index(x, y, graph->eq_count)].type == JOINT_TYPE_EMPTY) {
                continue;
            }
            new_adjacency[graph_index(new_x, new_y, new_eq_count)] = graph->adjacency[certain_order_graph_index(x, y, graph->eq_count)];
        }
        merge_members = old_merge_members;
#pragma endregion

#pragma region Eqs part
        uint32_t displacement = 0;
        if (merge_state[x]) {
            not_merged_displacement -= graph->eq_indices[x+1] - graph->eq_indices[x];
            displacement = -merged_displacement;
        } else if (group_min < x) {
            merged_displacement += graph->eq_indices[x+1] - graph->eq_indices[x];
            displacement = not_merged_displacement;
        }
        for (; solid_index < graph->eq_indices[x] + 1; ++solid_index) {
            new_eqs[solid_index + displacement] = graph->eqs[solid_index];
        }
        if (group_min < x && !merge_state[x]) {
            graph->eq_indices[new_x] = graph->eq_indices[x] + displacement;
        }
#pragma endregion
    }
    memcpy(graph->adjacency, new_adjacency, adjacency_size(new_eq_count) * sizeof(GraphNode));
    graph->eq_indices[new_eq_count] = solid_count;
    memcpy(graph->eqs, new_eqs, solid_count * sizeof(uint32_t));
    graph->eq_count = new_eq_count;

malloc_err:
    switch (3 - allocated) {
        case 0:
            free(new_eqs);
        case 1:
            free(new_adjacency);
        case 2:
            free(merge_state);
        case 3:
        default:
            break;
    }
    return result;
}



void clear_resolution_steps(ResolutionMode * const resolution_mode) {
    while (resolution_mode->steps.count) {
        resolution_mode->steps.count--;
        ResolutionStep * current = &resolution_mode->steps.array[resolution_mode->steps.count];
        switch (current->type) {
            case STEP_TYPE_GRAPH:
                free(current->graph_step.eq_indices);
                free(current->graph_step.eqs);
                free(current->graph_step.edges);
                break;
            case STEP_TYPE_JOINT:
                free(current->joint_step.eqs);
                break;
            case STEP_TYPE_RELATION:
                break;
        }
    }
    free(resolution_mode->steps.array);
    resolution_mode->steps.array = NULL;
}

uint32_t register_graph_resolution_step(system_internal const * const system, ResolutionMode * const resolution_mode, GraphNode const * const graph, uint32_t const isostatic_graph, uint32_t const * const isomorphism, uint32_t const * const eq_indices, uint32_t const * const eqs, uint32_t const * const solid_to_eq) {
    uint32_t result = KINEPY_SUCCESS;
    uint32_t allocated = 0;
    void * temp_ptr = realloc(resolution_mode->steps.array, (resolution_mode->steps.count + 1) * sizeof(ResolutionStep));
    if (!temp_ptr) {
        result = KINEPY_MALLOC_FAILED;
        goto malloc_err;
    }
    ++allocated;
    resolution_mode->steps.array = temp_ptr;

    ResolutionStep step = {
        .type = STEP_TYPE_GRAPH,
        .graph_step = {
            .isostatic_graph = isostatic_graph,
            .eq_indices = NULL,
            .eqs = NULL,
            .edges = NULL,
            .solution_index = 0
        }
    };
    set_alloc_array(step.graph_step.eq_indices, uint32_t, ISOSTATIC_GRAPHS[isostatic_graph].vertex_count+1);

    *step.graph_step.eq_indices = 0;
    for(int index = 0; index < ISOSTATIC_GRAPHS[isostatic_graph].vertex_count; ++index) {
        step.graph_step.eq_indices[index+1] = step.graph_step.eq_indices[index] + eq_indices[isomorphism[index]+1] - eq_indices[isomorphism[index]];
    }

    uint32_t const solid_count = step.graph_step.eq_indices[ISOSTATIC_GRAPHS[isostatic_graph].vertex_count];
    set_alloc_array(step.graph_step.eqs, uint32_t, solid_count);

    for (int index = 0; index < ISOSTATIC_GRAPHS[isostatic_graph].vertex_count; ++index) {
        memcpy(step.graph_step.eqs + step.graph_step.eq_indices[index], eqs + eq_indices[isomorphism[index]], step.graph_step.eq_indices[index+1] - step.graph_step.eq_indices[index]);
    }

    set_alloc_array(step.graph_step.edges, GraphStepEdge, ISOSTATIC_GRAPHS[isostatic_graph].edge_count);
    for (int index = 0; index < ISOSTATIC_GRAPHS[isostatic_graph].edge_count; ++index) {

    }

malloc_err:
    switch(allocated) {
        case 4:
            free(step.graph_step.edges);
        case 3:
            free(step.graph_step.eqs);
        case 2:
            free(step.graph_step.eq_indices);
        case 1:
            free(resolution_mode->steps.array);
        case 0:
        default:
            break;
    }
    return result;
}

uint32_t determine_computation_order(system_internal const * const system, ResolutionMode * const resolution_mode) {
    uint32_t result = KINEPY_SUCCESS;
    uint8_t allocated = 0;

    uint32_t const solid_count = system->solid_description_array.obj_count;

    declare_alloc_array(adjacency, GraphNode, adjacency_size(solid_count), const);
    declare_alloc_array(degrees, JointDegree, solid_count, const);
    declare_alloc_array(eqs, uint32_t, solid_count, const);
    declare_alloc_array(eq_indices, uint32_t, solid_count+1, const);
    declare_alloc_array(solid_to_eq, uint32_t, solid_count,);

    Graph graph = {
        .adjacency = adjacency,
        .eq_count = solid_count,
        .joint_degrees = degrees,
        .eqs = eqs,
        .eq_indices = eq_indices,
        .solid_to_eq = solid_to_eq
    };

    make_graph_adjacency(system, &graph);
    compute_joint_degrees(&graph);

    resolution_mode->steps.count = 0;
    resolution_mode->steps.array = NULL;


    // At the beginning, every solid is its own equivalence class
    for (int index = 0; index < solid_count; ++index) {
        eqs[index] = index;
        eq_indices[index] = index;
    }
    eq_indices[solid_count] = solid_count;

    while (solid_count > 1) {
        uint32_t * isomorphism;
        uint32_t isostatic_graph;
        KINEPY_check(find_isomorphism(&graph, &isostatic_graph, &isomorphism)){
            clear_resolution_steps(resolution_mode);
            break;
        }

        KINEPY_check(merge_graph(&graph, isomorphism, ISOSTATIC_GRAPHS[isostatic_graph].vertex_count)) {
            free(isomorphism);
            clear_resolution_steps(resolution_mode);
            break;
        }

        free(isomorphism);



        uint32_t* current_solid = eqs;
        for (int index = 0; index < graph.eq_count; ++index) {
            uint32_t * last = eqs + eq_indices[index+1];
            while (current_solid < last) {
                solid_to_eq[*current_solid] = index;
                ++current_solid;
            }
        }

        compute_joint_degrees(&graph);
    }


malloc_err:
    switch (5 - allocated) {
        case 0:
            free(solid_to_eq);
        case 1:
            free(eq_indices);
        case 2:
            free(eqs);
        case 3:
            free(degrees);
        case 4:
            free(adjacency);
        case 5:
        default:
            break;
    }
    return result;
}
