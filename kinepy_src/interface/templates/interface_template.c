#include "stdlib.h"
#include "math.h"
#include "internal/enums.h"
#include "interface_template.h"
#include "interface/helper_functions.h"

#ifndef allocate_array
#define allocate_array(NAME, count) NAME = malloc((count) * sizeof(*(NAME))); if (!NAME)
#define allocate_array_jump(NAME, count) NAME = malloc((count) * sizeof(*(NAME))); if (!NAME) {goto malloc_err;} ++allocated
#endif

uint32_t allocate_system(System * const system, uint32_t const solid_count, uint32_t const joint_count, uint32_t const relation_count) {
    uint8_t allocated = 0;

    void** const pointers[] = {
        (void**)&system->solid_parameters_ptr,
        (void**)&system->config.joints,
        (void**)&system->joint_parameters_ptr,
        (void**)&system->config.relations,
        (void**)&system->relation_parameters_ptr,
    };

    allocate_array_jump(system->solid_parameters_ptr, solid_count);
    system->config.solid_count = solid_count;

    allocate_array_jump(system->config.joints, joint_count);
    allocate_array_jump(system->joint_parameters_ptr, joint_count);
    system->config.joint_count = joint_count;

    allocate_array_jump(system->config.relations, relation_count);
    allocate_array_jump(system->relation_parameters_ptr, relation_count);
    system->config.relation_count = relation_count;

    return KINEPY_SUCCESS;
malloc_err:
    for (int index = 0; index < allocated; ++index) {
        free(*pointers[index]);
    }
    return KINEPY_MALLOC_FAILED;
}

void free_system(System * const system) {
    free(system->solid_parameters_ptr);
    system->config.solid_count = 0;

    free(system->config.joints);
    free(system->joint_parameters_ptr);
    system->config.joint_count = 0;

    free(system->config.relations);
    free(system->relation_parameters_ptr);
    system->config.relation_count = 0;

}
uint32_t update_solid_physics(System * const system, uint32_t const solid_index, float_type const mass, float_type const moment_of_inertia, float_type const gx, float_type const gy) {
    if (!is_existing_solid(&system->config, solid_index)){
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    typeof(system->solid_parameters_ptr) array = system->solid_parameters_ptr + solid_index;
    array->mass = mass * system->unit_system->mass;
    array->moment_of_inertia = moment_of_inertia * system->unit_system->moment_of_inertia;
    array->gx = gx * system->unit_system->length;
    array->gy = gy * system->unit_system->length;

    return KINEPY_SUCCESS;
}


uint32_t update_revolute_constraints(System * const system, uint32_t const joint_index, float_type const x1, float_type const y1, float_type const x2, float_type const y2) {
    if (system->config.joints[joint_index].type != JOINT_TYPE_REVOLUTE) {
        return KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE;
    }

    typeof(system->joint_parameters_ptr) constraints = system->joint_parameters_ptr + joint_index;
    constraints->x1 = x1 * system->unit_system->length;
    constraints->y1 = y1 * system->unit_system->length;
    constraints->x2 = x2 * system->unit_system->length;
    constraints->y2 = y2 * system->unit_system->length;
    return KINEPY_SUCCESS;
}

uint32_t update_prismatic_constraints(System * const system, uint32_t const joint_index, float_type alpha1, float_type distance1, float_type alpha2, float_type distance2) {
    if (!is_existing_joint(&system->config, joint_index)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    if (system->config.joints[joint_index].type != JOINT_TYPE_PRISMATIC) {
        return KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE;
    }
    alpha1 *= system->unit_system->angle;
    distance1 *= system->unit_system->length;
    alpha2 *= system->unit_system->angle;
    distance2 *= system->unit_system->length;

    typeof(system->joint_parameters_ptr) constraints = system->joint_parameters_ptr + joint_index;
    constraints->x1 = -trig(sin)(alpha1) * distance1;
    constraints->y1 = trig(cos)(alpha1) * distance1;
    constraints->x2 = -trig(sin)(alpha2) * distance2;
    constraints->y2 = trig(cos)(alpha2) * distance2;
    return KINEPY_SUCCESS;
}


float_type get_v0_unit(System const * const system, uint32_t const relation_index) {
    uint8_t const joint2_type = system->config.joints[system->config.relations[relation_index].joint2].type;
    if (joint2_type == JOINT_TYPE_PRISMATIC) {
        return system->unit_system->length;
    } else {
        return system->unit_system->angle;
    }
}

float_type get_ratio_unit(System const * const system, uint32_t const relation_index) {
    uint8_t const joint1_type = system->config.joints[system->config.relations[relation_index].joint1].type;
    uint8_t const joint2_type = system->config.joints[system->config.relations[relation_index].joint2].type;

    if (joint1_type == joint2_type) {
        return system->unit_system->dimensionless;
    }
    if (joint2_type == JOINT_TYPE_PRISMATIC) {
        return system->unit_system->length / system->unit_system->angle;
    } else {
        return system->unit_system->angle / system->unit_system->length;
    }
}


void update_relation_parameters(System * const system, uint32_t const obj_index, float_type const ratio, float_type const v0) {
    typeof(system->relation_parameters_ptr) constraints = system->relation_parameters_ptr + obj_index;
    constraints->ratio = ratio * get_ratio_unit(system, obj_index);
    constraints->v0 = v0 * get_v0_unit(system, obj_index);
}


