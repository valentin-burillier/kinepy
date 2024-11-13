#include "layouts.h"
#include "immintrin.h"

#define MEM_ALIGNMENT 0x20

uint8_t is_existing_solid(system_internal const * const system, uint32_t const solid_index) {
    return solid_index < system->solid_description_array.obj_count;
}
uint8_t is_existing_joint_type(system_internal const * const __attribute__((unused)) system, uint8_t const type) {
    return JOINT_TYPE_EMPTY < type && type < JOINT_TYPE_COUNT;
}

uint8_t is_existing_joint(system_internal const * const system, uint32_t const joint_index) {
    return joint_index < system->joint_description_array.obj_count;
}

uint8_t is_existing_relation_type(system_internal const * const __attribute__((unused)) system, uint8_t const type) {
    return RELATION_TYPE_EMPTY < type && type < RELATION_TYPE_COUNT;
}

uint8_t is_valid_relation(system_internal const * const system, uint8_t type, uint32_t joint1, uint32_t joint2) {
    switch (type) {
        case RELATION_TYPE_GEAR:
            return (system->joint_description_array.type_ptr[joint1] == JOINT_TYPE_REVOLUTE && system->joint_description_array.type_ptr[joint2] == JOINT_TYPE_REVOLUTE);
        case RELATION_TYPE_GEAR_RACK:
            return (system->joint_description_array.type_ptr[joint1] == JOINT_TYPE_REVOLUTE && system->joint_description_array.type_ptr[joint2] == JOINT_TYPE_PRISMATIC);
        default:
            return 1;
    }
}

uint32_t KINEPY_allocate_solid_descriptions_s(System_s *const system, uint32_t const obj_count) {
    SolidDescriptionArray_s *const array = &system->solid_description_array;
    array->obj_count = obj_count;
    uint32_t const size = obj_count;
    uint8_t allocated = 0;
    array->mass_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->mass_ptr) { 
        goto err_alloc; 
    }
    ++allocated;
    array->moment_of_inertia_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->moment_of_inertia_ptr) { 
        goto err_alloc; 
    }
    ++allocated;
    array->gx_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->gx_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->gy_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->gy_ptr) {
        goto err_alloc;
    }
    ++allocated;
    return KINEPY_SUCCESS;

err_alloc:
    if (allocated) {
        _mm_free(array->mass_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->moment_of_inertia_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->gx_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->gy_ptr);
        --allocated;
    }
    return KINEPY_MALLOC_FAILED;
}

void KINEPY_free_solid_descriptions_s(System_s *const system) {
    SolidDescriptionArray_s *const array = &system->solid_description_array;
    array->obj_count = 0;
    _mm_free(array->mass_ptr);
    _mm_free(array->moment_of_inertia_ptr);
    _mm_free(array->gx_ptr);
    _mm_free(array->gy_ptr);
}

void KINEPY_get_solid_description_s(System_s const *const system, uint32_t const obj_index, SolidDescription_s *const output) {
    SolidDescriptionArray_s const *const array = &system->solid_description_array;
    uint32_t const index = obj_index;
    output->mass = array->mass_ptr[index] / system->unit_system->mass;
    output->moment_of_inertia = array->moment_of_inertia_ptr[index] / system->unit_system->moment_of_inertia;
    output->gx = array->gx_ptr[index] / system->unit_system->length;
    output->gy = array->gy_ptr[index] / system->unit_system->length;
}

uint32_t KINEPY_set_solid_description_s(System_s *const system, uint32_t const obj_index, SolidDescription_s const *const input) {
    SolidDescriptionArray_s *const array = &system->solid_description_array;
    uint32_t const index = obj_index;
    if (input->mass < 0.0 || input->moment_of_inertia < 0.0) {
        return KINEPY_INVALID_INPUT_NEGATIVE_INERTIAL_VALUE;
    }
    array->mass_ptr[index] = system->unit_system->mass * input->mass;
    array->moment_of_inertia_ptr[index] = system->unit_system->moment_of_inertia * input->moment_of_inertia;
    array->gx_ptr[index] = system->unit_system->length * input->gx;
    array->gy_ptr[index] = system->unit_system->length * input->gy;
    return KINEPY_SUCCESS;
}

uint32_t KINEPY_allocate_joint_descriptions_s(System_s *const system, uint32_t const obj_count) {
    JointDescriptionArray_s *const array = &system->joint_description_array;
    array->obj_count = obj_count;
    uint32_t const size = obj_count;
    uint8_t allocated = 0;
    array->solid1_ptr = _mm_malloc(size * sizeof(uint32_t), MEM_ALIGNMENT);
    if (!array->solid1_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->solid2_ptr = _mm_malloc(size * sizeof(uint32_t), MEM_ALIGNMENT);
    if (!array->solid2_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->type_ptr = _mm_malloc(size * sizeof(uint8_t), MEM_ALIGNMENT);
    if (!array->type_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->constraint1_x_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->constraint1_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->constraint1_y_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->constraint1_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->constraint2_x_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->constraint2_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->constraint2_y_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->constraint2_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    return KINEPY_SUCCESS;
    err_alloc:
    if (allocated) {
        _mm_free(array->solid1_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->solid2_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->type_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->constraint1_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->constraint1_y_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->constraint2_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->constraint2_y_ptr);
        --allocated;
    }
    return KINEPY_MALLOC_FAILED;
}

void KINEPY_free_joint_descriptions_s(System_s *const system) {
    JointDescriptionArray_s *const array = &system->joint_description_array;
    array->obj_count = 0;
    _mm_free(array->solid1_ptr);
    _mm_free(array->solid2_ptr);
    _mm_free(array->type_ptr);
    _mm_free(array->constraint1_x_ptr);
    _mm_free(array->constraint1_y_ptr);
    _mm_free(array->constraint2_x_ptr);
    _mm_free(array->constraint2_y_ptr);
}

void KINEPY_get_joint_description_s(System_s const *const system, uint32_t const obj_index, JointDescription_s *const output) {
    JointDescriptionArray_s const *const array = &system->joint_description_array;
    uint32_t const index = obj_index;
    output->solid1 = array->solid1_ptr[index];
    output->solid2 = array->solid2_ptr[index];
    output->type = array->type_ptr[index];
    output->constraint1_x = array->constraint1_x_ptr[index] / system->unit_system->length;
    output->constraint1_y = array->constraint1_y_ptr[index] / system->unit_system->length;
    output->constraint2_x = array->constraint2_x_ptr[index] / system->unit_system->length;
    output->constraint2_y = array->constraint2_y_ptr[index] / system->unit_system->length;
}

uint32_t KINEPY_set_joint_description_s(System_s *const system, uint32_t const obj_index, JointDescription_s const *const input) {
    JointDescriptionArray_s *const array = &system->joint_description_array;
    uint32_t const index = obj_index;
    if (input->solid1 == input->solid2) {
        return KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS;
    }
    if (!is_existing_solid((system_internal*) system, input->solid1)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    array->solid1_ptr[index] = input->solid1;
    if (!is_existing_solid((system_internal*) system, input->solid2)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    array->solid2_ptr[index] = input->solid2;
    if (!is_existing_joint_type((system_internal*) system, input->type)) {
        return KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE;
    }
    array->type_ptr[index] = input->type;
    array->constraint1_x_ptr[index] = system->unit_system->length * input->constraint1_x;
    array->constraint1_y_ptr[index] = system->unit_system->length * input->constraint1_y;
    array->constraint2_x_ptr[index] = system->unit_system->length * input->constraint2_x;
    array->constraint2_y_ptr[index] = system->unit_system->length * input->constraint2_y;
    return KINEPY_SUCCESS;
}

uint32_t KINEPY_allocate_solid_results_s(System_s *const system, uint32_t const frame_count) {
    SolidResultArray_s *const array = &system->solid_result_array;
    array->frame_count = frame_count;
    uint32_t const size = system->solid_description_array.obj_count * frame_count;
    uint8_t allocated = 0;
    array->origin_x_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->origin_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->origin_y_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->origin_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->orientation_x_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->orientation_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->orientation_y_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->orientation_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->_dynamic_x_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->_dynamic_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->_dynamic_y_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->_dynamic_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->_dynamic_m_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->_dynamic_m_ptr) {
        goto err_alloc;
    }
    ++allocated;
    return KINEPY_SUCCESS;

err_alloc:
    if (allocated) {
        _mm_free(array->origin_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->origin_y_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->orientation_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->orientation_y_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->_dynamic_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->_dynamic_y_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->_dynamic_m_ptr);
        --allocated;
    }
    return KINEPY_MALLOC_FAILED;
}

void KINEPY_free_solid_results_s(System_s *const system) {
    SolidResultArray_s *const array = &system->solid_result_array;
    array->frame_count = 0;
    _mm_free(array->origin_x_ptr);
    _mm_free(array->origin_y_ptr);
    _mm_free(array->orientation_x_ptr);
    _mm_free(array->orientation_y_ptr);
    _mm_free(array->_dynamic_x_ptr);
    _mm_free(array->_dynamic_y_ptr);
    _mm_free(array->_dynamic_m_ptr);
}

void KINEPY_get_solid_result_s(System_s const *const system, uint32_t const obj_index, uint32_t const frame_index, SolidResult_s *const output) {
    SolidResultArray_s const *const array = &system->solid_result_array;
    uint32_t const index = obj_index * array->frame_count + frame_index;
    output->origin_x = array->origin_x_ptr[index] / system->unit_system->length;
    output->origin_y = array->origin_y_ptr[index] / system->unit_system->length;
    output->orientation_x = array->orientation_x_ptr[index] / system->unit_system->dimensionless;
    output->orientation_y = array->orientation_y_ptr[index] / system->unit_system->dimensionless;
    output->_dynamic_x = array->_dynamic_x_ptr[index] / system->unit_system->force;
    output->_dynamic_y = array->_dynamic_y_ptr[index] / system->unit_system->force;
    output->_dynamic_m = array->_dynamic_m_ptr[index] / system->unit_system->torque;
}

uint32_t KINEPY_allocate_relation_descriptions_s(System_s *const system, uint32_t const obj_count) {
    RelationDescriptionArray_s *const array = &system->relation_description_array;
    array->obj_count = obj_count;
    uint32_t const size = obj_count;
    uint8_t allocated = 0;
    array->joint1_ptr = _mm_malloc(size * sizeof(uint32_t), MEM_ALIGNMENT);
    if (!array->joint1_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->joint2_ptr = _mm_malloc(size * sizeof(uint32_t), MEM_ALIGNMENT);
    if (!array->joint2_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->type_ptr = _mm_malloc(size * sizeof(uint8_t), MEM_ALIGNMENT);
    if (!array->type_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->ratio_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->ratio_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->v0_ptr = _mm_malloc(size * sizeof(float), MEM_ALIGNMENT);
    if (!array->v0_ptr) {
        goto err_alloc;
    }
    ++allocated;
    return KINEPY_SUCCESS;

err_alloc:
    if (allocated) {
        _mm_free(array->joint1_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->joint2_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->type_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->ratio_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->v0_ptr);
        --allocated;
    }
    return KINEPY_MALLOC_FAILED;
}

void KINEPY_free_relation_descriptions_s(System_s *const system) {
    RelationDescriptionArray_s *const array = &system->relation_description_array;
    array->obj_count = 0;
    _mm_free(array->joint1_ptr);
    _mm_free(array->joint2_ptr);
    _mm_free(array->type_ptr);
    _mm_free(array->ratio_ptr);
    _mm_free(array->v0_ptr);
}

void KINEPY_get_relation_description_s(System_s const *const system, uint32_t const obj_index, RelationDescription_s *const output) {
    RelationDescriptionArray_s const *const array = &system->relation_description_array;
    uint32_t const index = obj_index;
    output->joint1 = array->joint1_ptr[index];
    output->joint2 = array->joint2_ptr[index];
    output->type = array->type_ptr[index];

    // TODO: weird units
    output->ratio = array->ratio_ptr[index];
    output->v0 = array->v0_ptr[index];
}

uint32_t KINEPY_set_relation_description_s(System_s *const system, uint32_t const obj_index, RelationDescription_s const *const input) {
    RelationDescriptionArray_s *const array = &system->relation_description_array;
    uint32_t const index = obj_index;
    if (input->joint1 == input->joint2) {
        return KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS;
    }
    if (!is_existing_joint((system_internal*) system, input->joint1)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    array->joint1_ptr[index] = input->joint1;
    if (!is_existing_joint((system_internal*) system, input->joint2)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    array->joint2_ptr[index] = input->joint2;
    if (!is_existing_relation_type((system_internal*) system, input->type)) {
        return KINEPY_INVALID_INPUT_WRONG_RELATION_TYPE;
    }
    if(!is_valid_relation((system_internal*) system, input->type, input->joint1, input->joint2)) {
        return KINEPY_INVALID_INPUT_UNMATCHED_RELATION_TYPE_AND_JOINT_TYPES;
    }
    array->type_ptr[index] = input->type;

    // TODO: weird units
    array->ratio_ptr[index] = input->ratio;
    array->v0_ptr[index] = input->v0;
    return KINEPY_SUCCESS;
}

uint32_t KINEPY_allocate_solid_descriptions_d(System_d *const system, uint32_t const obj_count) {
    SolidDescriptionArray_d *const array = &system->solid_description_array;
    array->obj_count = obj_count;
    uint32_t const size = obj_count;
    uint8_t allocated = 0;
    array->mass_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->mass_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->moment_of_inertia_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->moment_of_inertia_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->gx_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->gx_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->gy_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->gy_ptr) { goto err_alloc; }
    ++allocated;
    return KINEPY_SUCCESS;

err_alloc:
    if (allocated) {
        _mm_free(array->mass_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->moment_of_inertia_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->gx_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->gy_ptr);
        --allocated;
    }
    return KINEPY_MALLOC_FAILED;
}

void KINEPY_free_solid_descriptions_d(System_d *const system) {
    SolidDescriptionArray_d *const array = &system->solid_description_array;
    array->obj_count = 0;
    _mm_free(array->mass_ptr);
    _mm_free(array->moment_of_inertia_ptr);
    _mm_free(array->gx_ptr);
    _mm_free(array->gy_ptr);
}

void KINEPY_get_solid_description_d(System_d const *const system, uint32_t const obj_index,SolidDescription_d *const output) {
    SolidDescriptionArray_d const *const array = &system->solid_description_array;
    uint32_t const index = obj_index;
    output->mass = array->mass_ptr[index] / system->unit_system->mass;
    output->moment_of_inertia = array->moment_of_inertia_ptr[index] / system->unit_system->moment_of_inertia;
    output->gx = array->gx_ptr[index] / system->unit_system->length;
    output->gy = array->gy_ptr[index] / system->unit_system->length;
}

uint32_t KINEPY_set_solid_description_d(System_d *const system, uint32_t const obj_index, SolidDescription_d const *const input) {
    SolidDescriptionArray_d *const array = &system->solid_description_array;
    uint32_t const index = obj_index;
    if (input->mass < 0.0 || input->moment_of_inertia < 0.0) {
        return KINEPY_INVALID_INPUT_NEGATIVE_INERTIAL_VALUE;
    }
    array->mass_ptr[index] = system->unit_system->mass * input->mass;
    array->moment_of_inertia_ptr[index] = system->unit_system->moment_of_inertia * input->moment_of_inertia;
    array->gx_ptr[index] = system->unit_system->length * input->gx;
    array->gy_ptr[index] = system->unit_system->length * input->gy;
    return KINEPY_SUCCESS;
}

uint32_t KINEPY_allocate_joint_descriptions_d(System_d *const system, uint32_t const obj_count) {
    JointDescriptionArray_d *const array = &system->joint_description_array;
    array->obj_count = obj_count;
    uint32_t const size = obj_count;
    uint8_t allocated = 0;
    array->solid1_ptr = _mm_malloc(size * sizeof(uint32_t), MEM_ALIGNMENT);
    if (!array->solid1_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->solid2_ptr = _mm_malloc(size * sizeof(uint32_t), MEM_ALIGNMENT);
    if (!array->solid2_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->type_ptr = _mm_malloc(size * sizeof(uint8_t), MEM_ALIGNMENT);
    if (!array->type_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->constraint1_x_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->constraint1_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->constraint1_y_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->constraint1_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->constraint2_x_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->constraint2_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->constraint2_y_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->constraint2_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    return KINEPY_SUCCESS;

err_alloc:
    if (allocated) {
        _mm_free(array->solid1_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->solid2_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->type_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->constraint1_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->constraint1_y_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->constraint2_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->constraint2_y_ptr);
        --allocated;
    }
    return KINEPY_MALLOC_FAILED;
}

void KINEPY_free_joint_descriptions_d(System_d *const system) {
    JointDescriptionArray_d *const array = &system->joint_description_array;
    array->obj_count = 0;
    _mm_free(array->solid1_ptr);
    _mm_free(array->solid2_ptr);
    _mm_free(array->type_ptr);
    _mm_free(array->constraint1_x_ptr);
    _mm_free(array->constraint1_y_ptr);
    _mm_free(array->constraint2_x_ptr);
    _mm_free(array->constraint2_y_ptr);
}

void KINEPY_get_joint_description_d(System_d const *const system, uint32_t const obj_index, JointDescription_d *const output) {
    JointDescriptionArray_d const *const array = &system->joint_description_array;
    uint32_t const index = obj_index;
    output->solid1 = array->solid1_ptr[index];
    output->solid2 = array->solid2_ptr[index];
    output->type = array->type_ptr[index];
    output->constraint1_x = array->constraint1_x_ptr[index] / system->unit_system->length;
    output->constraint1_y = array->constraint1_y_ptr[index] / system->unit_system->length;
    output->constraint2_x = array->constraint2_x_ptr[index] / system->unit_system->length;
    output->constraint2_y = array->constraint2_y_ptr[index] / system->unit_system->length;
}

uint32_t KINEPY_set_joint_description_d(System_d *const system, uint32_t const obj_index, JointDescription_d const *const input) {
    JointDescriptionArray_d *const array = &system->joint_description_array;
    uint32_t const index = obj_index;
    if (input->solid1 == input->solid2) {
        return KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS;
    }
    if (!is_existing_solid((system_internal*) system, input->solid1)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    array->solid1_ptr[index] = input->solid1;
    if (!is_existing_solid((system_internal*) system, input->solid2)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    array->solid2_ptr[index] = input->solid2;
    if (!is_existing_joint_type((system_internal*) system, input->type)) {
        return KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE;
    }
    array->type_ptr[index] = input->type;
    array->constraint1_x_ptr[index] = system->unit_system->length * input->constraint1_x;
    array->constraint1_y_ptr[index] = system->unit_system->length * input->constraint1_y;
    array->constraint2_x_ptr[index] = system->unit_system->length * input->constraint2_x;
    array->constraint2_y_ptr[index] = system->unit_system->length * input->constraint2_y;
    return KINEPY_SUCCESS;
}

uint32_t KINEPY_allocate_solid_results_d(System_d *const system, uint32_t const frame_count) {
    SolidResultArray_d *const array = &system->solid_result_array;
    array->frame_count = frame_count;
    uint32_t const size = system->solid_description_array.obj_count * frame_count;
    uint8_t allocated = 0;
    array->origin_x_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->origin_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->origin_y_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->origin_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->orientation_x_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->orientation_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->orientation_y_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->orientation_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->_dynamic_x_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->_dynamic_x_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->_dynamic_y_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->_dynamic_y_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->_dynamic_m_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->_dynamic_m_ptr) {
        goto err_alloc;
    }
    ++allocated;
    return KINEPY_SUCCESS;

err_alloc:
    if (allocated) {
        _mm_free(array->origin_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->origin_y_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->orientation_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->orientation_y_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->_dynamic_x_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->_dynamic_y_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->_dynamic_m_ptr);
        --allocated;
    }
    return KINEPY_MALLOC_FAILED;
}

void KINEPY_free_solid_results_d(System_d *const system) {
    SolidResultArray_d *const array = &system->solid_result_array;
    array->frame_count = 0;
    _mm_free(array->origin_x_ptr);
    _mm_free(array->origin_y_ptr);
    _mm_free(array->orientation_x_ptr);
    _mm_free(array->orientation_y_ptr);
    _mm_free(array->_dynamic_x_ptr);
    _mm_free(array->_dynamic_y_ptr);
    _mm_free(array->_dynamic_m_ptr);
}

void KINEPY_get_solid_result_d(System_d const *const system, uint32_t const obj_index, uint32_t const frame_index, SolidResult_d *const output) {
    SolidResultArray_d const *const array = &system->solid_result_array;
    uint32_t const index = obj_index * array->frame_count + frame_index;
    output->origin_x = array->origin_x_ptr[index] / system->unit_system->length;
    output->origin_y = array->origin_y_ptr[index] / system->unit_system->length;
    output->orientation_x = array->orientation_x_ptr[index] / system->unit_system->dimensionless;
    output->orientation_y = array->orientation_y_ptr[index] / system->unit_system->dimensionless;
    output->_dynamic_x = array->_dynamic_x_ptr[index] / system->unit_system->force;
    output->_dynamic_y = array->_dynamic_y_ptr[index] / system->unit_system->force;
    output->_dynamic_m = array->_dynamic_m_ptr[index] / system->unit_system->torque;
}

uint32_t KINEPY_allocate_relation_descriptions_d(System_d *const system, uint32_t const obj_count) {
    RelationDescriptionArray_d *const array = &system->relation_description_array;
    array->obj_count = obj_count;
    uint32_t const size = obj_count;
    uint8_t allocated = 0;
    array->joint1_ptr = _mm_malloc(size * sizeof(uint32_t), MEM_ALIGNMENT);
    if (!array->joint1_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->joint2_ptr = _mm_malloc(size * sizeof(uint32_t), MEM_ALIGNMENT);
    if (!array->joint2_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->type_ptr = _mm_malloc(size * sizeof(uint8_t), MEM_ALIGNMENT);
    if (!array->type_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->ratio_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->ratio_ptr) {
        goto err_alloc;
    }
    ++allocated;
    array->v0_ptr = _mm_malloc(size * sizeof(double), MEM_ALIGNMENT);
    if (!array->v0_ptr) {
        goto err_alloc;
    }
    ++allocated;
    return KINEPY_SUCCESS;

err_alloc:
    if (allocated) {
        _mm_free(array->joint1_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->joint2_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->type_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->ratio_ptr);
        --allocated;
    }
    if (allocated) {
        _mm_free(array->v0_ptr);
        --allocated;
    }
    return KINEPY_MALLOC_FAILED;
}

void KINEPY_free_relation_descriptions_d(System_d *const system) {
    RelationDescriptionArray_d *const array = &system->relation_description_array;
    array->obj_count = 0;
    _mm_free(array->joint1_ptr);
    _mm_free(array->joint2_ptr);
    _mm_free(array->type_ptr);
    _mm_free(array->ratio_ptr);
    _mm_free(array->v0_ptr);
}

void KINEPY_get_relation_description_d(System_d const *const system, uint32_t const obj_index, RelationDescription_d *const output) {
    RelationDescriptionArray_d const *const array = &system->relation_description_array;
    uint32_t const index = obj_index;
    output->joint1 = array->joint1_ptr[index];
    output->joint2 = array->joint2_ptr[index];
    output->type = array->type_ptr[index];
    // TODO: weird units
    output->ratio = array->ratio_ptr[index];
    output->v0 = array->v0_ptr[index];
}

uint32_t KINEPY_set_relation_description_d(System_d *const system, uint32_t const obj_index, RelationDescription_d const *const input) {
    RelationDescriptionArray_d *const array = &system->relation_description_array;
    uint32_t const index = obj_index;
    if (input->joint1 == input->joint2) {
        return KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS;
    }
    if (!is_existing_joint((system_internal*) system, input->joint1)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    array->joint1_ptr[index] = input->joint1;
    if (!is_existing_joint((system_internal*) system, input->joint2)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    array->joint2_ptr[index] = input->joint2;
    if (!is_existing_relation_type((system_internal*) system, input->type)) {
        return KINEPY_INVALID_INPUT_WRONG_RELATION_TYPE;
    }
    if(!is_valid_relation((system_internal*) system, input->type, input->joint1, input->joint2)) {
        return KINEPY_INVALID_INPUT_UNMATCHED_RELATION_TYPE_AND_JOINT_TYPES;
    }
    array->type_ptr[index] = input->type;

    // TODO: weird units
    array->ratio_ptr[index] = input->ratio;
    array->v0_ptr[index] = input->v0;
    return KINEPY_SUCCESS;
}
