#include "interface.h"
#include "stdlib.h"

#define DESCRIPTION_ALLOC_CONST_PARAMETERS size_t const count
#define RESULT_ALLOC_CONST_PARAMETERS uint8_t index_mode, size_t const obj_count, size_t const frame_count

#define DESCRIPTION_GET_SET_CONST_PARAMETERS size_t const index
#define RESULT_GET_SET_CONST_PARAMETERS size_t const index, size_t const frame

#define DESCRIPTION_ALLOC_TOTAL array->count = count; size_t const total = count;
#define RESULT_ALLOC_TOTAL array->index_mode = index_mode; array->base_count = obj_count; array->frame_count = frame_count; size_t const total = obj_count * frame_count;

#define DESCRIPTION_RESET_ARRAY_ATTRIBUTES array->count = 0;
#define RESULT_RESET_ARRAY_ATTRIBUTES array->index_mode = -1; array->base_count = 0; array->frame_count = 0;

#define DESCRIPTION_SET_ARRAY_VIEW_ATTRIBUTES out_view->count = array->count;
#define RESULT_SET_ARRAY_VIEW_ATTRIBUTES out_view->index_mode = array->index_mode; out_view->base_count = array->base_count; out_view->frame_count = array->frame_count;

#define DESCRIPTION_ACTUAL_INDEX size_t const actual_index = index;
#define RESULT_ACTUAL_INDEX size_t const actual_index = index * (view->index_mode * view->frame_count + 1 - view->index_mode) + frame * ((1-view->index_mode) * view->base_count + view->index_mode);

#define DESCRIPTION_FIRST_ARRAY_PTR(CLASS_NAME) ((void*) out_view + offsetof(CLASS_NAME##ArrayView, count) + sizeof(out_view->count))
#define RESULT_FIRST_ARRAY_PTR(CLASS_NAME) ((void*) out_view + offsetof(CLASS_NAME##ArrayView, frame_count) + sizeof(out_view->frame_count))

#define ALLOCATE_ATTRIBUTE(TYPE, NAME, ...) array->NAME##_array = malloc(total * sizeof(TYPE)); if (!array->NAME##_array) {goto error_allocate;} ++allocated;
#define ERR_ALLOCATE_ATTRIBUTE(TYPE, NAME, ...) if (!allocated) {return 0;} free(array->NAME##_array); --allocated;
#define ALLOCATOR(CLASS, CLASS_NAME, ATTR_NAME, TYPE, ID)                                                   \
uint8_t allocate_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * const array, TYPE##_ALLOC_CONST_PARAMETERS) {  \
    TYPE##_ALLOC_TOTAL                                                                                      \
    uint8_t allocated = 0;                                                                                  \
                                                                                                            \
    CLASS##_LAYOUT_##ID(ALLOCATE_ATTRIBUTE)                                                                 \
    return 1;                                                                                               \
error_allocate:                                                                                             \
    CLASS##_LAYOUT_##ID(ERR_ALLOCATE_ATTRIBUTE)                                                             \
    return 0;                                                                                               \
}

#define FREE_ATTRIBUTE(TYPE, NAME, ...) free(array->NAME##_array);
#define DE_ALLOCATOR(CLASS, CLASS_NAME, ATTR_NAME, TYPE, ID)    \
void free_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * array) {  \
    TYPE##_RESET_ARRAY_ATTRIBUTES                               \
    CLASS##_LAYOUT_##ID(FREE_ATTRIBUTE)                         \
}

#define SIMPLE_GET_VIEW_ATTRIBUTE(TYPE, NAME, ...) *(void**)attribute = (void*)array->NAME##_array; attribute += sizeof(void*); *(size_t*)attribute = sizeof(TYPE); attribute += sizeof(size_t);
#define GET_VIEW_ATTRIBUTE(TYPE, NAME, _GARBAGE, PARENT_TYPE, PARENT_NAME, ...) *(void**)attribute = (void*)&(array->PARENT_NAME##_array->NAME); attribute += sizeof(void*); *(size_t*)attribute = sizeof(PARENT_TYPE); attribute += sizeof(size_t);
#define GET_VIEW_BODY(TYPE, NAME, LAYOUT, ...) LAYOUT(GET_VIEW_ATTRIBUTE, TYPE, NAME)

#define IMPLEMENT_VIEW_GETTER(CLASS, CLASS_NAME, ATTR_NAME, TYPE, ID)                                                           \
void get_##ATTR_NAME##_##ID##_array_view(CLASS_NAME##ID##Array const * const array, CLASS_NAME##ArrayView * const out_view) {   \
    TYPE##_SET_ARRAY_VIEW_ATTRIBUTES                                                                                            \
    void * attribute = TYPE##_FIRST_ARRAY_PTR(CLASS_NAME);                     \
    CLASS##_LAYOUT_##ID(GET_VIEW_BODY)                                                                                          \
}

#define ATTRIBUTE_SETTER_GETTER(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, CLASS_TYPE, ...) \
TYPE get_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * const view, CLASS_TYPE##_GET_SET_CONST_PARAMETERS) {          \
    CLASS_TYPE##_ACTUAL_INDEX                                                                                                   \
    return *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * actual_index);\
}                                                                                                      \
void set_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * const view, CLASS_TYPE##_GET_SET_CONST_PARAMETERS, TYPE const value) { \
    CLASS_TYPE##_ACTUAL_INDEX                                                                                                        \
    *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * actual_index) = value;\
}


#define GET_GENERIC_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) output->NAME = *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * actual_index);
#define SET_GENERIC_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * actual_index) = input->NAME;

#define IMPLEMENT_ATTRIBUTE_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
void get_##ATTR_NAME(CLASS_NAME##ArrayView const * const view, TYPE##_GET_SET_CONST_PARAMETERS, CLASS_NAME * const output) { \
    TYPE##_ACTUAL_INDEX                         \
    CLASS##_LAYOUT_0(GET_GENERIC_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME) \
} \
void set_##ATTR_NAME(CLASS_NAME##ArrayView const * const view, TYPE##_GET_SET_CONST_PARAMETERS, CLASS_NAME const * const input) {        \
    TYPE##_ACTUAL_INDEX                         \
    CLASS##_LAYOUT_0(SET_GENERIC_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME) \
}

#define IMPLEMENT_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
CLASS(ALLOCATOR, CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
CLASS(DE_ALLOCATOR, CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
CLASS(IMPLEMENT_VIEW_GETTER, CLASS, CLASS_NAME, ATTR_NAME, TYPE)  \
CLASS##_LAYOUT_0(ATTRIBUTE_SETTER_GETTER, CLASS, CLASS_NAME, ATTR_NAME, TYPE)\
IMPLEMENT_ATTRIBUTE_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE)

IMPLEMENT_INTERFACE(JOINT_RESULT, JointResult, joint_result, RESULT)
IMPLEMENT_INTERFACE(SOLID_RESULT, SolidResult, solid_result, RESULT)
IMPLEMENT_INTERFACE(JOINT_DESCRIPTION, JointDescription, joint_description, DESCRIPTION)
IMPLEMENT_INTERFACE(SOLID_DESCRIPTION, SolidDescription, solid_description, DESCRIPTION)

