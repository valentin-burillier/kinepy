#include "layouts.h"
#include "immintrin.h"

#define RES_CONST_ALLOC_PARAM size_t const obj_count, size_t const frame_count
#define DESC_CONST_ALLOC_PARAM size_t const obj_count

#define RES_CONST_INTER_PARAM size_t const obj_index, size_t const frame_index
#define DESC_CONST_INTER_PARAM size_t const obj_index

#define RES_ALLOC_SIZE size_t const size = obj_count * frame_count;
#define DESC_ALLOC_SIZE size_t const size = obj_count;

#define RES_ALLOC_ATTR array->obj_count = obj_count; array->frame_count = frame_count;
#define DESC_ALLOC_ATTR array->obj_count = obj_count;

#define RES_FREE_ATTR array->obj_count = 0; array->frame_count = 0;
#define DESC_FREE_ATTR array->obj_count = 0;

#define RES_INTER_INDEX size_t const index = obj_index * array->frame_count + frame_index;
#define DESC_INTER_INDEX size_t const index = obj_index;

#define ALLOC_ARRAY(TYPE, NAME, ...) array->NAME##_ptr = _mm_malloc(size * sizeof(TYPE), 0x20); if (!array->NAME##_ptr) {goto err_alloc;} ++allocated;
#define ALLOC_ARRAY_ERR(TYPE, NAME, ...) if (allocated) {_mm_free(array->NAME##_ptr); return 0;} --allocated;

#define FREE_ARRAY(TYPE, NAME, ...) _mm_free(array->NAME##_ptr);



#define STD_UNIT_SET(TYPE, NAME, UNAME, ...) array->NAME##_ptr[index] = system->unit_system->UNAME * input->NAME;
#define STD_UNIT_GET(TYPE, NAME, UNAME, ...) output->NAME = array->NAME##_ptr[index] / system->unit_system->UNAME;

#define NOT_A_PHYSICAL_QUANTITY_SET(TYPE, NAME, FUNCTION, ...) if (!FUNCTION((system_internal*)system, input->NAME)){return KINEPY_INVALID_INPUT;} array->NAME##_ptr[index] = input->NAME;
#define NOT_A_PHYSICAL_QUANTITY_GET(TYPE, NAME, FUNCTION, ...) output->NAME = array->NAME##_ptr[index];

#define GET_OBJECT(TYPE, NAME, FUNC, ARG) FUNC##_GET(TYPE, NAME, ARG)
#define SET_OBJECT(TYPE, NAME, FUNC, ARG) FUNC##_SET(TYPE, NAME, ARG)

#define RES_SETTER_IMPL(LAYOUT, NAME, ATTR_NAME, TYPE, FLOAT, F_SUFFIX)
#define DESC_SETTER_IMPL(LAYOUT, NAME, ATTR_NAME, TYPE, FLOAT, F_SUFFIX) \
uint32_t KINEPY_set_##ATTR_NAME##F_SUFFIX(System##F_SUFFIX * const system, TYPE##_CONST_INTER_PARAM, NAME##F_SUFFIX const * const input){      \
    NAME##Array##F_SUFFIX * const array = &system->ATTR_NAME##_array;    \
    TYPE##_INTER_INDEX                                                   \
    LAYOUT(SET_OBJECT, FLOAT)                                            \
    return KINEPY_SUCCESS;                                               \
}


#define IMPLEMENT_INTERFACE(LAYOUT, NAME, ATTR_NAME, TYPE, FLOAT, F_SUFFIX) \
uint32_t KINEPY_allocate_##ATTR_NAME##s##F_SUFFIX(System##F_SUFFIX * const system, TYPE##_CONST_ALLOC_PARAM) { \
    NAME##Array##F_SUFFIX * const array = &system->ATTR_NAME##_array;       \
    TYPE##_ALLOC_ATTR                                                       \
    TYPE##_ALLOC_SIZE                                                       \
    uint8_t allocated = 0;                                                  \
    LAYOUT(ALLOC_ARRAY, FLOAT)                                              \
    return KINEPY_SUCCESS;                                                  \
err_alloc:                                                                  \
    LAYOUT(ALLOC_ARRAY_ERR, FLOAT)                                          \
    return KINEPY_MALLOC_FAILED;                                            \
}                                                                           \
                                                                            \
void KINEPY_free_##ATTR_NAME##s##F_SUFFIX(System##F_SUFFIX * const system) {\
    NAME##Array##F_SUFFIX * const array = &system->ATTR_NAME##_array;       \
    TYPE##_FREE_ATTR                                                        \
    LAYOUT(FREE_ARRAY, FLOAT)                                               \
}                                                                           \
                                                                            \
void KINEPY_get_##ATTR_NAME##F_SUFFIX(System##F_SUFFIX const * const system, TYPE##_CONST_INTER_PARAM, NAME##F_SUFFIX * const output) { \
    NAME##Array##F_SUFFIX const * const array = &system->ATTR_NAME##_array; \
    TYPE##_INTER_INDEX                                                      \
    LAYOUT(GET_OBJECT, FLOAT)                                               \
}                                                                           \
TYPE##_SETTER_IMPL(LAYOUT, NAME, ATTR_NAME, TYPE, FLOAT, F_SUFFIX)

uint8_t is_existing_solid(system_internal const * const system, uint32_t const solid_index) {
    return solid_index < system->solid_description_array.obj_count;
}
uint8_t is_existing_joint_type(system_internal const * const __attribute__((unused)) system, uint8_t const type) {
    return JOINT_TYPE_EMPTY < type && type < JOINT_TYPE_COUNT;
}

SYSTEM_LAYOUT(IMPLEMENT_INTERFACE, float, _s)
