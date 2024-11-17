#include "stdlib.h"
#include "math.h"
#include "internal/enums.h"
#include "interface_template.h"


#ifndef allocate_array
#define allocate_array(NAME) NAME = malloc(obj_count * sizeof(*(NAME))); if (!NAME)
#define allocate_array_jump(NAME) NAME = malloc(obj_count * sizeof(*(NAME))); if (!NAME) {goto malloc_err;} ++allocated
#endif

uint32_t allocate_solid_descriptions(System * const system, uint32_t const obj_count) {
    system->solids.obj_count = obj_count;
    allocate_array(system->solids.solid_description_ptr) {
        return KINEPY_MALLOC_FAILED;
    }
    return KINEPY_SUCCESS;
}

void free_solid_descriptions(System * const system) {
    system->solids.obj_count = 0;
    free(system->solids.solid_description_ptr);
}
void get_solid_description(System const * system, uint32_t const obj_index, SolidDescription * const output) {
    SolidDescription * object = system->solids.solid_description_ptr + obj_index;
    output->mass = object->mass / system->unit_system->mass;
    output->moment_of_inertia = object->mass / system->unit_system->moment_of_inertia;
    output->gx = object->gx / system->unit_system->length;
    output->gy = object->gy / system->unit_system->length;
}


uint32_t set_solid_description(System * const system, uint32_t const obj_index, SolidDescription const * const input) {
    SolidDescriptionArray * const array = &system->solids;
    if (input->mass < 0.0 || input->moment_of_inertia < 0.0) {
        return KINEPY_INVALID_INPUT_NEGATIVE_INERTIAL_VALUE;
    }
    array->solid_description_ptr[obj_index].mass = system->unit_system->mass * input->mass;
    array->solid_description_ptr[obj_index].moment_of_inertia = system->unit_system->moment_of_inertia * input->moment_of_inertia;
    array->solid_description_ptr[obj_index].gx = system->unit_system->length * input->gx;
    array->solid_description_ptr[obj_index].gy = system->unit_system->length * input->gy;
    return KINEPY_SUCCESS;
}

uint32_t allocate_joint_descriptions(System * const system, uint32_t const obj_count) {
    system->joints.obj_count = obj_count;
    uint32_t allocated = 0;
    allocate_array_jump(system->joints.solid1_ptr);
    allocate_array_jump(system->joints.solid2_ptr);
    allocate_array_jump(system->joints.type_ptr);
    allocate_array_jump(system->joints.constraint_ptr);
    return KINEPY_SUCCESS;

    malloc_err:
    switch (4 - allocated) {
        case 0:
            free(system->joints.constraint_ptr);
        case 1:
            free(system->joints.type_ptr);
        case 2:
            free(system->joints.solid2_ptr);
        case 3:
            free(system->joints.solid1_ptr);
        default:
            break;
    }
    return KINEPY_MALLOC_FAILED;
}
void free_joint_descriptions(System * const system) {
    system->joints.obj_count = 0;
    free(system->joints.solid1_ptr);
    free(system->joints.solid2_ptr);
    free(system->joints.type_ptr);
    free(system->joints.constraint_ptr);

}
void get_joint_description(System const * const system, uint32_t const obj_index, JointDescription * const output) {
    JointDescriptionArray const * const array = &system->joints;
    output->solid1 = array->solid1_ptr[obj_index];
    output->solid2 = array->solid2_ptr[obj_index];
    output->type = array->type_ptr[obj_index];

    typeof(*system->joints.constraint_ptr) * constraints = &array->constraint_ptr[obj_index];
    output->constraint1_x = constraints->x1 / system->unit_system->length;
    output->constraint1_y = constraints->y1 / system->unit_system->length;
    output->constraint2_x = constraints->x2 / system->unit_system->length;
    output->constraint2_y = constraints->y2 / system->unit_system->length;
}


uint32_t update_revolute_constraints(System * const system, uint32_t const obj_index, float_type const x1, float_type const y1, float_type const x2, float_type const y2) {
    if (system->joints.type_ptr[obj_index] != JOINT_TYPE_REVOLUTE) {
        return KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE;
    }

    typeof(*system->joints.constraint_ptr) * constraints = &system->joints.constraint_ptr[obj_index];
    constraints->x1 = x1 * system->unit_system->length;
    constraints->y1 = y1 * system->unit_system->length;
    constraints->x2 = x2 * system->unit_system->length;
    constraints->y2 = y2 * system->unit_system->length;
    return KINEPY_SUCCESS;
}

uint32_t update_prismatic_constraints(System * const system, uint32_t const obj_index, float_type alpha1, float_type distance1, float_type alpha2, float_type distance2) {
    if (system->joints.type_ptr[obj_index] != JOINT_TYPE_PRISMATIC) {
        return KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE;
    }
    alpha1 *= system->unit_system->angle;
    distance1 *= system->unit_system->length;
    alpha2 *= system->unit_system->angle;
    distance2 *= system->unit_system->length;

    typeof(*system->joints.constraint_ptr) * constraints = &system->joints.constraint_ptr[obj_index];
    constraints->x1 = -trig(sin)(alpha1) * distance1;
    constraints->y1 = trig(cos)(alpha1) * distance1;
    constraints->x2 = -trig(sin)(alpha2) * distance2;
    constraints->y2 = trig(cos)(alpha2) * distance2;
    return KINEPY_SUCCESS;
}


uint32_t is_existing_solid(System const * const system, uint32_t const solid_index) {
    return solid_index < system->solids.obj_count;
}

uint32_t is_existing_joint_type(uint8_t const type) {
    return JOINT_TYPE_EMPTY < type && type < JOINT_TYPE_COUNT;
}

uint32_t set_joint_description(System * const system, uint32_t const obj_index, JointDescription const * const input) {
    JointDescriptionArray * const array = &system->joints;
    if (!is_existing_solid(system, input->solid1) || !is_existing_solid(system, input->solid2)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    if (input->solid1 == input->solid2) {
        return KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS;
    }
    if (!is_existing_joint_type(input->type)) {
        return KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE;
    }

    array->solid1_ptr[obj_index] = input->solid1;
    array->solid2_ptr[obj_index] = input->solid2;
    array->type_ptr[obj_index] = input->type;

    typeof(*system->joints.constraint_ptr) * constraints = &array->constraint_ptr[obj_index];
    constraints->x1 = input->constraint1_x * system->unit_system->length;
    constraints->y1 = input->constraint1_y * system->unit_system->length;
    constraints->x2 = input->constraint2_x * system->unit_system->length;
    constraints->y2 = input->constraint2_y * system->unit_system->length;

    return KINEPY_SUCCESS;
}

uint32_t allocate_relation_descriptions(System * const system, uint32_t const obj_count) {
    system->relations.obj_count = obj_count;
    uint32_t allocated = 0;
    allocate_array_jump(system->relations.joint1_ptr);
    allocate_array_jump(system->relations.joint2_ptr);
    allocate_array_jump(system->relations.type_ptr);
    allocate_array_jump(system->relations.parameter_ptr);
    return KINEPY_SUCCESS;

    malloc_err:
    switch (4 - allocated) {
        case 0:
            free(system->relations.parameter_ptr);
        case 1:
            free(system->relations.type_ptr);
        case 2:
            free(system->relations.joint2_ptr);
        case 3:
            free(system->relations.joint1_ptr);
        default:
            break;
    }
    return KINEPY_MALLOC_FAILED;
}
void free_relation_descriptions(System * const system) {
    system->relations.obj_count = 0;
    free(system->relations.joint1_ptr);
    free(system->relations.joint2_ptr);
    free(system->relations.type_ptr);
    free(system->relations.parameter_ptr);
}

float_type get_v0_unit(System const * const system, uint32_t const relation_index) {
    uint8_t const joint2_type = system->joints.type_ptr[system->relations.joint2_ptr[relation_index]];
    if (joint2_type == JOINT_TYPE_PRISMATIC) {
        return system->unit_system->length;
    } else {
        return system->unit_system->angle;
    }
}

float_type get_ratio_unit(System const * const system, uint32_t const relation_index) {
    uint8_t const joint1_type = system->joints.type_ptr[system->relations.joint1_ptr[relation_index]];
    uint8_t const joint2_type = system->joints.type_ptr[system->relations.joint2_ptr[relation_index]];

    if (joint1_type == joint2_type) {
        return system->unit_system->dimensionless;
    }
    if (joint2_type == JOINT_TYPE_PRISMATIC) {
        return system->unit_system->length / system->unit_system->angle;
    } else {
        return system->unit_system->angle / system->unit_system->length;
    }
}

void get_relation_description(System const * const system, uint32_t const obj_index, RelationDescription * const output) {
    RelationDescriptionArray const * const array = &system->relations;
    output->joint1 = array->joint1_ptr[obj_index];
    output->joint2 = array->joint2_ptr[obj_index];
    output->type = array->type_ptr[obj_index];

    typeof(*array->parameter_ptr) * constraints = &array->parameter_ptr[obj_index];
    output->ratio = constraints->ratio / get_ratio_unit(system, obj_index);
    output->v0 = constraints->v0 / get_v0_unit(system, obj_index);
}

uint8_t is_existing_joint(System const * const system, uint32_t const joint_index) {
    return joint_index < system->joints.obj_count;
}

uint8_t is_existing_relation_type(uint8_t const type) {
    return RELATION_TYPE_EMPTY < type && type < RELATION_TYPE_COUNT;
}

uint8_t is_valid_relation(System const * const system, uint8_t type, uint32_t joint1, uint32_t joint2) {
    switch (type) {
        case RELATION_TYPE_GEAR:
            return (system->joints.type_ptr[joint1] == JOINT_TYPE_REVOLUTE && system->joints.type_ptr[joint2] == JOINT_TYPE_REVOLUTE);
        case RELATION_TYPE_GEAR_RACK:
            return (system->joints.type_ptr[joint1] == JOINT_TYPE_REVOLUTE && system->joints.type_ptr[joint2] == JOINT_TYPE_PRISMATIC);
        default:
            return 1;
    }
}

void update_relation_parameters(System * const system, uint32_t const obj_index, float_type const ratio, float_type const v0) {
    typeof(*system->relations.parameter_ptr) * constraints = &system->relations.parameter_ptr[obj_index];
    constraints->ratio = ratio * get_ratio_unit(system, obj_index);
    constraints->v0 = v0 * get_v0_unit(system, obj_index);
}

uint32_t set_relation_description(System * system, uint32_t obj_index, RelationDescription const * input) {
    RelationDescriptionArray * const array = &system->relations;
    if (input->joint1 == input->joint2) {
        return KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS;
    }
    if (!is_existing_joint(system, input->joint1) && !is_existing_joint(system, input->joint2)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    if (!is_existing_relation_type(input->type)) {
        return KINEPY_INVALID_INPUT_WRONG_RELATION_TYPE;
    }
    if (!is_valid_relation(system, input->type, input->joint1, input->joint2)) {
        return KINEPY_INVALID_INPUT_UNMATCHED_RELATION_TYPE_AND_JOINT_TYPES;
    }

    array->joint1_ptr[obj_index] = input->joint1;
    array->joint2_ptr[obj_index] = input->joint2;
    array->type_ptr[obj_index] = input->type;

    typeof(*array->parameter_ptr) * constraints = &array->parameter_ptr[obj_index];
    constraints->ratio =  input->ratio * get_ratio_unit(system, obj_index);
    constraints->v0 = input->v0 * get_v0_unit(system, obj_index);

    return KINEPY_SUCCESS;
}
