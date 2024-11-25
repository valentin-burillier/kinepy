#include "stdlib.h"
#include "math.h"
#include "internal/enums.h"
#include "interface_template.h"
#include "interface/helper_functions.h"
#include "internal/util_macros.h"

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
    __typeof__(system->solid_parameters_ptr) array = system->solid_parameters_ptr + solid_index;
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

    __typeof__(system->joint_parameters_ptr) constraints = system->joint_parameters_ptr + joint_index;
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

    __typeof__(system->joint_parameters_ptr) constraints = system->joint_parameters_ptr + joint_index;
    constraints->x1 = -math(sin)(alpha1) * distance1;
    constraints->y1 = math(cos)(alpha1) * distance1;
    constraints->x2 = -math(sin)(alpha2) * distance2;
    constraints->y2 = math(cos)(alpha2) * distance2;
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
    __typeof__(system->relation_parameters_ptr) constraints = system->relation_parameters_ptr + obj_index;
    constraints->ratio = ratio * get_ratio_unit(system, obj_index);
    constraints->v0 = v0 * get_v0_unit(system, obj_index);
}


uint32_t allocate_result(KpConfiguration const * config, uint32_t frame_count, Result * result) {
    uint32_t allocated = 0;

    float_type ** arrays[] = {
        result->_temp_arrays + 0,
        result->_temp_arrays + 1,
        result->_temp_arrays + 2,
        result->_temp_arrays + 3,
        &result->solid_orientation_x,
        &result->solid_orientation_y,
        &result->solid_x,
        &result->solid_y,
        &result->_solid_force_x,
        &result->_solid_force_y,
        &result->_solid_torque,
        &result->joint_value
    };
    allocate_result_array_jump(result->_temp_arrays[0], frame_count);
    allocate_result_array_jump(result->_temp_arrays[1], frame_count);
    allocate_result_array_jump(result->_temp_arrays[2], frame_count);
    allocate_result_array_jump(result->_temp_arrays[3], frame_count);

    allocate_result_array_jump(result->solid_orientation_x, frame_count * config->solid_count);
    allocate_result_array_jump(result->solid_orientation_y, frame_count * config->solid_count);
    allocate_result_array_jump(result->solid_x, frame_count * config->solid_count);
    allocate_result_array_jump(result->solid_y, frame_count * config->solid_count);

    allocate_result_array_jump(result->_solid_force_x, frame_count * config->solid_count);
    allocate_result_array_jump(result->_solid_force_y, frame_count * config->solid_count);
    allocate_result_array_jump(result->_solid_torque, frame_count * config->solid_count);

    allocate_result_array_jump(result->joint_value, frame_count * config->joint_count);

    result->frame_count = frame_count;

    return KINEPY_SUCCESS;
malloc_err:
    for (int index = 0; index < allocated; ++index) {
        free_result_array(*arrays[index]);
    }
    return KINEPY_MALLOC_FAILED;
}

void free_result(Result * result) {
    free_result_array(result->_temp_arrays[0]);
    free_result_array(result->_temp_arrays[1]);
    free_result_array(result->_temp_arrays[2]);
    free_result_array(result->_temp_arrays[3]);

    free_result_array(result->solid_orientation_x);
    free_result_array(result->solid_orientation_y);
    free_result_array(result->solid_x);
    free_result_array(result->solid_y);

    free_result_array(result->_solid_force_x);
    free_result_array(result->_solid_force_y);
    free_result_array(result->_solid_torque);

    free_result_array(result->joint_value);
}

