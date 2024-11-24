#include "string.h"
#include "stdlib.h"

#include "graph_interface.h"
#include "private_structs.h"
#include "internal/enums.h"

#define INHERITED_CONTINUITY_BIT 0x01
#define SOLVED_JOINT_BIT 0x10
#define NO_COMMON_EQ 255
#define GEAR_SHOULD_BE_FORMED 0
#define GEAR_MAY_NOT_BE_FORMED 1


void make_graph_adjacency(KpConfiguration const * const config, Graph * const graph) {
    for (uint32_t index = 0; index < adjacency_size(graph->eq_count); ++index) {
        graph->adjacency[index].type = JOINT_TYPE_EMPTY;
        graph->adjacency[index].joint_index = -1;
    }
    for (uint32_t index = 0; index < config->joint_count; ++index) {
        typeof(config->joints) joint = config->joints + index;
        uint32_t const node_index = graph_index(joint->solid1, joint->solid2, graph->eq_count);

        (graph->adjacency + node_index)->type = joint->type;
        (graph->adjacency + node_index)->joint_index = index;
    }
}

void compute_joint_degrees(Graph const * const graph) {
    uint32_t index = 0;
    for (int eq_index = 0; eq_index < graph->eq_count; ++eq_index) {
        graph->degrees[eq_index] = (JointDegree){.revolute=0, .prismatic=0};
    }
    for (int x = 0; x < graph->eq_count; ++x) {
        for (int y = x+1; y < graph->eq_count; ++y) {
            if (graph->adjacency[index].type) {
                uint8_t const type = graph->adjacency[index].type - 1;
                ++(graph->degrees[x].arr[type]);
                ++(graph->degrees[y].arr[type]);
            }
            ++index;
        }
    }
}


void compute_solid_to_eq(Graph const * const graph) {
    uint32_t* current_solid = graph->eqs;
    for (int index = 0; index < graph->eq_count; ++index) {
        uint32_t * last = graph->eqs + graph->eq_indices[index+1];
        while (current_solid < last) {
            graph->solid_to_eq[*current_solid] = index;
            ++current_solid;
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
            if (!compare_degrees(target_graph->degrees[isomorphism_stage], graph->degrees[vertex_index]) || !can_match(vertex_shuffle, isomorphism_stage, vertex_index, graph, target_graph)) {
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
    not_merged_displacement -= graph->eq_indices[group_min+1] - graph->eq_indices[group_min];

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
        uint32_t displacement;
        if (x <= group_min) {
            displacement = 0;
        } else if (merge_state[x]) {
            not_merged_displacement -= graph->eq_indices[x+1] - graph->eq_indices[x];
            displacement = -merged_displacement;
        } else {
            merged_displacement += graph->eq_indices[x+1] - graph->eq_indices[x];
            displacement = not_merged_displacement;
        }
        for (; solid_index < graph->eq_indices[x+1]; ++solid_index) {
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

    compute_solid_to_eq(graph);
    compute_joint_degrees(graph);
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


void kp_clear_resolution_steps(ResolutionMode * const resolution_mode) {
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
                break;
            case STEP_TYPE_JOINT_REVOLUTE:
            case STEP_TYPE_JOINT_PRISMATIC:
                free(current->joint_step.eqs);
                break;
            case STEP_TYPE_RELATION:
                break;
            case STEP_TYPE_RELATION_GEAR:
            case STEP_TYPE_RELATION_GEAR_RACK:
            case STEP_TYPE_RELATION_DISTANT:
            case STEP_TYPE_RELATION_EFFORTLESS:
                free(current->relation_step.eqs);
                break;
        }
    }
    free(resolution_mode->steps.array);
    resolution_mode->steps.array = NULL;
}

uint32_t register_graph_resolution_step(KpConfiguration const * const config, ResolutionMode * const resolution_mode, Graph const * const graph, uint32_t const isostatic_graph, uint32_t const * const isomorphism) {
    uint32_t result = KINEPY_SUCCESS;
    uint32_t allocated = 0;
    void * temp_ptr = realloc(resolution_mode->steps.array, (resolution_mode->steps.count + 1) * sizeof(ResolutionStep));
    if (!temp_ptr) {
        result = KINEPY_MALLOC_FAILED;
        goto malloc_err;
    }
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

    set_alloc_array(step.graph_step.eq_indices, ISOSTATIC_GRAPHS[isostatic_graph].vertex_count+1);
    *step.graph_step.eq_indices = 0;
    for(int index = 0; index < ISOSTATIC_GRAPHS[isostatic_graph].vertex_count; ++index) {
        step.graph_step.eq_indices[index+1] = step.graph_step.eq_indices[index] + graph->eq_indices[isomorphism[index]+1] - graph->eq_indices[isomorphism[index]];
    }


    uint32_t const solid_count = step.graph_step.eq_indices[ISOSTATIC_GRAPHS[isostatic_graph].vertex_count];
    set_alloc_array(step.graph_step.eqs, solid_count);
    for (int index = 0; index < ISOSTATIC_GRAPHS[isostatic_graph].vertex_count; ++index) {
        memcpy(step.graph_step.eqs + step.graph_step.eq_indices[index], graph->eqs + graph->eq_indices[isomorphism[index]], step.graph_step.eq_indices[index+1] - step.graph_step.eq_indices[index]);
    }


    set_alloc_array(step.graph_step.edges, ISOSTATIC_GRAPHS[isostatic_graph].edge_count);
    for (int index = 0; index < ISOSTATIC_GRAPHS[isostatic_graph].edge_count; ++index) {
        Edge const * const edge_ptr = ISOSTATIC_GRAPHS[isostatic_graph].edges + index;
        uint32_t eq_x = isomorphism[(*edge_ptr)[0]];
        uint32_t eq_y = isomorphism[(*edge_ptr)[1]];
        uint32_t joint_index = graph->adjacency[graph_index(eq_x, eq_y, graph->eq_count)].joint_index;

        step.graph_step.edges[index].joint_index = joint_index;
        step.graph_step.edges[index].orientation = graph->solid_to_eq[config->joints[joint_index].solid1] != eq_x;
        // equivalent to
        // step.graph_step.edges[index].orientation = graph->solid_to_eq[system->joints.config_ptr[joint_index].solid2] != eq_y;
    }
    resolution_mode->steps.array[resolution_mode->steps.count] = step;
    ++resolution_mode->steps.count;

    return result;
malloc_err:
    switch(3 - allocated) {
        case 0:
            free(step.graph_step.edges);
        case 1:
            free(step.graph_step.eqs);
        case 2:
            free(step.graph_step.eq_indices);
        case 3:
        default:
            break;
    }
    return result;
}

void push_joint(Graph * const graph, uint32_t const joint_index, uint8_t const continuity) {
    graph->joint_state[joint_index] |= SOLVED_JOINT_BIT | continuity;
    graph->joint_queue[graph->joint_queue_head] = joint_index;
    ++graph->joint_queue_head;
}

void push_gear(Graph * const graph, uint32_t const joint_index, uint32_t const relation_node_index) {
    uint32_t * ptr = (uint32_t*)&graph->gear_queue[graph->gear_queue_head];
    *ptr = joint_index;
    *(ptr+1) = relation_node_index;
    ++graph->gear_queue_head;
}

uint32_t solve_isostatic_graphs(KpConfiguration const * const config, ResolutionMode * const resolution_mode, Graph * const graph, uint8_t const certain_continuity) {
    uint32_t result = KINEPY_SUCCESS;

    uint32_t * isomorphism = NULL;
    while (graph->eq_count > 1) {
        uint32_t isostatic_graph;
        check(find_isomorphism(graph, &isostatic_graph, &isomorphism)){
            return result;
        }
        check(register_graph_resolution_step(config, resolution_mode, graph, isostatic_graph, isomorphism)) {
            break;
        }
        ResolutionStep const * const current_step = &resolution_mode->steps.array[resolution_mode->steps.count-1];
        for (int index = 0; index < ISOSTATIC_GRAPHS[isostatic_graph].edge_count; ++index) {
            uint32_t const joint_index = current_step->graph_step.edges[index].joint_index;
            if (graph->joint_state[joint_index] & SOLVED_JOINT_BIT) {
                result = KINEPY_FAILURE;
                break;
            }
            push_joint(graph, joint_index, certain_continuity);
        }
        if (result != KINEPY_SUCCESS) {
            break;
        }
        check(merge_graph(graph, isomorphism, ISOSTATIC_GRAPHS[isostatic_graph].vertex_count)) {
            break;
        }
        free(isomorphism);
        isomorphism = NULL;
    }
    free(isomorphism);
    return result;
}

uint8_t test_gear_conformity(KpConfiguration const * const config, Graph const * const graph, uint32_t const gear_index) {
    typeof(config->relations) relation_config = config->relations + gear_index;
    switch (relation_config->type) {
        case RELATION_TYPE_GEAR:
        case RELATION_TYPE_GEAR_RACK:
            break;
        default:
            return 0;
    }
    typeof(config->joints) joint1_config = config->joints + relation_config->joint1;
    typeof(config->joints) joint2_config = config->joints + relation_config->joint2;

    uint32_t const eq11 = graph->solid_to_eq[joint1_config->solid1];
    uint32_t const eq12 = graph->solid_to_eq[joint1_config->solid2];
    uint32_t const eq21 = graph->solid_to_eq[joint2_config->solid1];
    uint32_t const eq22 = graph->solid_to_eq[joint2_config->solid2];

    if (eq21 == eq11) {
        return 0b00;
    } else if (eq22 == eq11) {
        return 0b01;
    } else if (eq21 == eq12) {
        return 0b10;
    } else if (eq22 == eq12) {
        return 0b11;
    }
    return NO_COMMON_EQ;
}

uint32_t register_relation_node(KpConfiguration const* const config, ResolutionMode * const resolution_mode, Graph * const graph, RelationNode * const node, uint32_t const joint_index, uint8_t common_eq_mask) {
    uint32_t result;
    node->solved = 1;
    node->pair->solved = 1;

    push_joint(graph, node->joint_index, INHERITED_CONTINUITY_BIT);

    // register the step
    void* tmp = realloc(resolution_mode->steps.array, sizeof(*resolution_mode->steps.array) * (resolution_mode->steps.count+1));
    if (!tmp) {
        return KINEPY_MALLOC_FAILED;
    }
    resolution_mode->steps.array = tmp;
    ResolutionStep * const step = &resolution_mode->steps.array[resolution_mode->steps.count];

    step->type = STEP_TYPE_RELATION + config->relations[node->relation_index].type;

    step->relation_step.relation_index = node->relation_index;
    step->relation_step.flags = !(graph->joint_state[joint_index] & INHERITED_CONTINUITY_BIT) | (common_eq_mask << 6) | ((joint_index == config->relations[node->relation_index].joint1) << 5);

    uint32_t const eq1 = graph->solid_to_eq[config->joints[node->joint_index].solid1];
    uint32_t const eq2 = graph->solid_to_eq[config->joints[node->joint_index].solid2];

    step->relation_step.first_eq_size = graph->eq_indices[eq1+1] - graph->eq_indices[eq1];
    step->relation_step.second_eq_size = graph->eq_indices[eq2+1] - graph->eq_indices[eq2];
    step->relation_step.eqs = malloc((step->relation_step.first_eq_size+step->relation_step.second_eq_size) * sizeof(*step->relation_step.eqs));
    if (!step->relation_step.eqs) {
        return KINEPY_MALLOC_FAILED;
    }
    ++resolution_mode->steps.count;

    memcpy(step->relation_step.eqs, graph->eq_indices + eq1, step->relation_step.first_eq_size * sizeof(*step->relation_step.eqs));
    memcpy(step->relation_step.eqs + step->relation_step.first_eq_size, graph->eq_indices + eq2, step->relation_step.second_eq_size * sizeof(*step->relation_step.eqs));

    uint32_t group_to_merge[2] = {eq1, eq2};
    result = merge_graph(graph, group_to_merge, 2);
    return result;
}


uint32_t propagate_relation_resolution(KpConfiguration const * const config, ResolutionMode * const resolution_mode, Graph * const graph, uint8_t allowed_delaying) {
    uint32_t result = KINEPY_SUCCESS;
    while (graph->joint_queue_tail != graph->joint_queue_head) {
        // extract next queued joint
        uint32_t const current_joint = graph->joint_queue[graph->joint_queue_tail];
        ++graph->joint_queue_tail;

        for (uint32_t index = graph->joint_indices[current_joint]; index < graph->joint_indices[current_joint+1]; ++index) {
            RelationNode * const node = &graph->joint_adjacency[index];
            if (node->solved) {
                // ignore relation source (there is at most 1)
                continue;
            }
            if (graph->joint_state[node->joint_index] & SOLVED_JOINT_BIT) {
                return KINEPY_INVALID_CONFIGURATION_HYPERSTATIC_RELATION_SCHEME;
            }

            uint8_t const common_eq_mask = test_gear_conformity(config, graph, node->relation_index);
            if (common_eq_mask == NO_COMMON_EQ) {
                if (!allowed_delaying) {
                    return KINEPY_INVALID_CONFIGURATION_GEAR_RELATION_WITH_NO_COMMON_EQ;
                }
                push_gear(graph, current_joint, index);
                continue;
            }
            check(register_relation_node(config, resolution_mode, graph, node, current_joint, common_eq_mask)) {
                return result;
            }
        }
    }
    return result;
}

uint32_t try_delayed_gears(KpConfiguration const * const config, ResolutionMode * const resolution_mode, Graph * const graph) {
    uint32_t result = KINEPY_SUCCESS;

    uint32_t current = graph->gear_queue_tail;
    while (current != graph->gear_queue_head) {
        uint32_t * ptr = (uint32_t*)&graph->gear_queue[current];
        uint32_t const current_joint = *ptr;
        uint32_t const node_index = *(ptr+1);
        RelationNode * const node = &graph->joint_adjacency[node_index];
        ++current;

        if (node->solved) {
            continue;
        }
        uint8_t const common_eq_mask = test_gear_conformity(config, graph, node->relation_index);
        if (common_eq_mask == NO_COMMON_EQ) {
            continue;
        }
        check(register_relation_node(config, resolution_mode, graph, node, current_joint, common_eq_mask)) {
            return result;
        }
        check(propagate_relation_resolution(config, resolution_mode, graph, GEAR_MAY_NOT_BE_FORMED)) {
            return result;
        }
    }
    // pop as many nodes as possible
    while (graph->gear_queue_tail != graph->gear_queue_head) {
        uint32_t *ptr = (uint32_t *) &graph->gear_queue[graph->gear_queue_tail];
        uint32_t const node_index = *(ptr + 1);
        RelationNode *const node = &graph->joint_adjacency[node_index];
        if (node->solved) {
            ++graph->gear_queue_tail;
        } else {
            break;
        }
    }
    return result;
}

uint8_t test_all_gear_conformity(KpConfiguration const * const config, Graph const * const graph) {
    for (int index = 0; index < config->relation_count; ++index) {
        if (test_gear_conformity(config, graph, index) == NO_COMMON_EQ) {
            return 0;
        }
    }
    return 1;
}

uint32_t register_inputs(KpConfiguration const * const config, ResolutionMode * const resolution_mode, Graph * const graph) {
    uint32_t result = KINEPY_SUCCESS;

    void* tmp = realloc(resolution_mode->steps.array, sizeof(*resolution_mode->steps.array) * (resolution_mode->steps.count + resolution_mode->piloted_or_blocked_joints.count));
    if (!tmp) {
        return KINEPY_MALLOC_FAILED;
    }
    resolution_mode->steps.array = tmp;

    for (int index = 0; index < resolution_mode->piloted_or_blocked_joints.count; ++index) {
        uint32_t const joint_index = resolution_mode->piloted_or_blocked_joints.joints[index];

        ResolutionStep * const step = &resolution_mode->steps.array[resolution_mode->steps.count];
        step->type = STEP_TYPE_JOINT + config->joints[joint_index].type;

        step->joint_step.joint_index = joint_index;
        uint32_t const eq1 = graph->solid_to_eq[config->joints[joint_index].solid1];
        uint32_t const eq2 = graph->solid_to_eq[config->joints[joint_index].solid2];

        step->joint_step.first_eq_size = graph->eq_indices[eq1+1] - graph->eq_indices[eq1];
        step->joint_step.second_eq_size = graph->eq_indices[eq2+1] - graph->eq_indices[eq2];
        step->joint_step.eqs = malloc((step->joint_step.first_eq_size+step->joint_step.second_eq_size) * sizeof(*step->joint_step.eqs));

        if (!step->joint_step.eqs) {
            return KINEPY_MALLOC_FAILED;
        }
        ++resolution_mode->steps.count;
        memcpy(step->joint_step.eqs, graph->eq_indices + eq1, step->joint_step.first_eq_size * sizeof(*step->joint_step.eqs));
        memcpy(step->joint_step.eqs + step->joint_step.first_eq_size, graph->eq_indices + eq2, step->joint_step.second_eq_size * sizeof(*step->joint_step.eqs));

        uint32_t const group_to_merge[] = {eq1, eq2};
        check(merge_graph(graph, group_to_merge, 2)) {
            return result;
        }
        push_joint(graph, joint_index, INHERITED_CONTINUITY_BIT);
    }
    return result;
}


uint32_t internal_determine_computation_order_body(KpConfiguration const * const config, ResolutionMode * const resolution_mode, Graph * const graph) {
    uint32_t result;
    while (1) {
        result = solve_isostatic_graphs(config, resolution_mode, graph, INHERITED_CONTINUITY_BIT);
        if (result != KINEPY_NO_GRAPH_FOUND) {
            // result might also be SUCCESS, which means the system is completely solved
            return result;
        }
        if (graph->joint_queue_tail == graph->joint_queue_head) {
            break;
        }
        check(try_delayed_gears(config, resolution_mode, graph)) {
            return result;
        }
        check(propagate_relation_resolution(config, resolution_mode, graph, GEAR_MAY_NOT_BE_FORMED)) {
            return result;
        }
        check(try_delayed_gears(config, resolution_mode, graph)) {
            return result;
        }
    }

    if (graph->gear_queue_tail != graph->gear_queue_head || !test_all_gear_conformity(config, graph)) {
        return KINEPY_INVALID_CONFIGURATION_GEAR_RELATION_WITH_NO_COMMON_EQ;
    }
    check(register_inputs(config, resolution_mode, graph)) {
        return result;
    }
    check(propagate_relation_resolution(config, resolution_mode, graph, GEAR_SHOULD_BE_FORMED)) {
        return result;
    }

    while (1) {
        result = solve_isostatic_graphs(config, resolution_mode, graph, 0);
        if (result != KINEPY_NO_GRAPH_FOUND || graph->joint_queue_tail == graph->joint_queue_head) {
            if (graph->eq_count == 1) {
                return KINEPY_SUCCESS;
            }
            return result;
        }
        check(propagate_relation_resolution(config, resolution_mode, graph, GEAR_SHOULD_BE_FORMED)) {
            return result;
        }
    }
}


void make_joint_adjacency(KpConfiguration const * const config, Graph * const graph) {
    memset(graph->joint_adjacency, 0xff, sizeof(*graph->joint_adjacency) * 2 * config->relation_count);
    for (int index = 0; index < config->relation_count; ++index) {
        uint32_t const joint1 = config->relations[index].joint1;
        uint32_t const joint2 = config->relations[index].joint2;

        RelationNode * ptr = &graph->joint_adjacency[graph->joint_indices[joint1]];
        while (ptr->joint_index != 0xffffffff) {
            ++ptr;
        }
        ptr->joint_index = joint2;
        ptr->relation_index = index;
        ptr->solved = 0;
        RelationNode * const old_ptr = ptr;

        ptr = &graph->joint_adjacency[graph->joint_indices[joint2]];
        while (ptr->joint_index != 0xffffffff) {
            ++ptr;
        }
        ptr->joint_index = joint1;
        ptr->relation_index = index;
        ptr->solved = 0;

        ptr->pair = old_ptr;
        old_ptr->pair = ptr;
    }
}


uint32_t hyper_statism(KpConfiguration const * const config, ResolutionMode const * const resolution_mode) {
    int32_t const value = 2 * config->joint_count - 3 * (config->solid_count - 1) + resolution_mode->piloted_or_blocked_joints.count + config->relation_count;
    if (value < 0) {
        return KINEPY_INVALID_CONFIGURATION_HYPOSTATIC_SYSTEM;
    }
    if (value > 0) {
        return KINEPY_INVALID_CONFIGURATION_HYPERSTATIC_SYSTEM;
    }
    return KINEPY_SUCCESS;
}

uint32_t kp_determine_computation_order(KpConfiguration const * const config, ResolutionMode * const resolution_mode) {
    uint32_t result;
    check(hyper_statism(config, resolution_mode)) {
        return result;
    }

#pragma region Setting up
    uint8_t allocated = 0;
    uint32_t const solid_count = config->solid_count;
    Graph graph = {0};

    void** const graph_ptr[] = {
        (void **) &graph.adjacency,
        (void **) &graph.degrees,
        (void **) &graph.eqs,
        (void **) &graph.eq_indices,
        (void **) &graph.solid_to_eq,
        (void **) &graph.joint_state,
        (void **) &graph.joint_indices,
        (void **) &graph.joint_adjacency,
        (void **) &graph.joint_queue,
        (void **) &graph.gear_queue
    };

    graph.eq_count = solid_count;
    set_alloc_array(graph.adjacency, adjacency_size(solid_count));
    set_alloc_array(graph.degrees, solid_count);
    set_alloc_array(graph.eqs, solid_count);
    set_alloc_array(graph.eq_indices, solid_count+1);
    set_alloc_array(graph.solid_to_eq, solid_count);
    set_alloc_array(graph.joint_state, config->joint_count);
    set_alloc_array(graph.joint_indices, config->joint_count + 1);
    set_alloc_array(graph.joint_adjacency, 2 * config->relation_count);
    set_alloc_array(graph.joint_queue, config->joint_count);
    set_alloc_array(graph.gear_queue, config->relation_count);
    graph.joint_queue_head = 0;
    graph.joint_queue_tail = 0;
    graph.gear_queue_tail = 0;
    graph.gear_queue_tail = 0;

    memset(graph.degrees, 0x00, sizeof(*graph.degrees) * solid_count);

    for (int index = 0; index < config->joint_count; ++index) {
        graph.joint_state[index] = (config->joints[index].type == JOINT_TYPE_PRISMATIC) * INHERITED_CONTINUITY_BIT;
    }

    memset(graph.joint_indices, 0, sizeof(*graph.joint_indices) * (config->joint_count + 1));
    for (int index = 0; index < config->relation_count; ++index) {
        uint32_t const joint1 = config->relations[index].joint1;
        uint32_t const joint2 = config->relations[index].joint2;

        graph.joint_indices[joint1]++;
        graph.joint_indices[joint2]++;
    }
    for (int index = 0; index < config->joint_count; ++index) {
        graph.joint_indices[index+1] += graph.joint_indices[index];
    }

    make_graph_adjacency(config, &graph);
    make_joint_adjacency(config, &graph);
    compute_joint_degrees(&graph);

    resolution_mode->steps.count = 0;
    resolution_mode->steps.array = NULL;

    // At the beginning, every solid is its own equivalence class
    for (int index = 0; index < solid_count; ++index) {
        graph.eqs[index] = index;
        graph.eq_indices[index] = index;
        graph.solid_to_eq[index] = index;
    }
    graph.eq_indices[solid_count] = solid_count;
#pragma endregion

    check(internal_determine_computation_order_body(config, resolution_mode, &graph)) {
        kp_clear_resolution_steps(resolution_mode);
    }

#pragma region Cleaning up
malloc_err:
    for (int index = 0; index < allocated; ++index) {
        free(*(graph_ptr[index]));
    }
#pragma endregion

    return result;
}
