#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdint.h>

#include "structure_layouts/solid_description.h"
#include "structure_layouts/solid_result.h"
#include "structure_layouts/joint_description.h"
#include "structure_layouts/joint_result.h"

#define STRUCT_ARRAY_ATTRIBUTE(TYPE, NAME, ...) TYPE * NAME##_array;

#define DESCRIPTION_ARRAY_ATTRIBUTES uint32_t count;
#define RESULT_ARRAY_ATTRIBUTES uint8_t index_mode; uint32_t base_count; uint32_t frame_count;

#define DESCRIPTION_ALLOC_PARAMETERS size_t count
#define RESULT_ALLOC_PARAMETERS uint8_t index_mode, size_t obj_count, size_t frame_count

#define DESCRIPTION_GET_SET_PARAMETERS size_t index
#define RESULT_GET_SET_PARAMETERS size_t index, size_t frame

#define MAKE_STRUCTS(ATTRIBUTES, NAME, TYPE) \
typedef struct NAME {ATTRIBUTES(STRUCT_ATTRIBUTE)} NAME; \
typedef struct NAME##Array {TYPE##_ARRAY_ATTRIBUTES ATTRIBUTES(STRUCT_ARRAY_ATTRIBUTE)} NAME##Array;

#define SIMPLE(CALLBACK, TYPE, NAME, ...) SIMPLE_##CALLBACK(TYPE, NAME, _,  TYPE, NAME, __VA_ARGS__)


#define STRUCTS_BY_ID(LAYOUTS, CLASS_NAME, TYPE, ID) MAKE_STRUCTS(LAYOUTS##_LAYOUT_##ID, CLASS_NAME##ID, TYPE)
#define UNION_ATTR(CLASS_NAME, ATTR_NAME, ID) CLASS_NAME##ID __##ATTR_NAME##_##ID;

#define DECLARE_MEMORY_INTERFACE(CLASS_NAME, ATTR_NAME, TYPE, ID)                                        \
uint8_t allocate_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * array, TYPE##_ALLOC_PARAMETERS);                      \
void free_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * array);

#define VIEW_GETTER(CLASS, CLASS_NAME, ATTR_NAME, ID) \
void get_##ATTR_NAME##_##ID##_array_view(CLASS_NAME##ID##Array const * array, CLASS_NAME##ArrayView * out_view);

#define VIEW_ATTRIBUTE(TYPE, NAME, ...) void * NAME##_ptr; size_t NAME##_offset;
#define VIEW_STRUCT(CLASS, CLASS_NAME, TYPE) \
typedef struct CLASS_NAME##ArrayView {       \
    TYPE##_ARRAY_ATTRIBUTES                  \
    CLASS##_LAYOUT_0(VIEW_ATTRIBUTE)         \
} CLASS_NAME##ArrayView;

#define DECLARE_GET_SET_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, CLASS_TYPE, ...) \
TYPE get_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * view, CLASS_TYPE##_GET_SET_PARAMETERS); \
void set_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * view, CLASS_TYPE##_GET_SET_PARAMETERS, TYPE value);

#define DECLARE_GENERIC_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
void get_##ATTR_NAME(CLASS_NAME##ArrayView const * view, TYPE##_GET_SET_PARAMETERS, CLASS_NAME * output); \
void set_##ATTR_NAME(CLASS_NAME##ArrayView const * view, TYPE##_GET_SET_PARAMETERS, CLASS_NAME const * input);

#define DECLARE_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
CLASS##_LAYOUT_0(DECLARE_GET_SET_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
DECLARE_GENERIC_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE)

#define MAKE_LAYOUT_STRUCTS(LAYOUTS, CLASS_NAME, ATTR_NAME, TYPE) \
LAYOUTS(STRUCTS_BY_ID, LAYOUTS, CLASS_NAME, TYPE)                 \
typedef union CLASS_NAME {                                        \
    struct {                                                      \
         LAYOUTS##_LAYOUT_0(STRUCT_ATTRIBUTE)                     \
    };                                                            \
    LAYOUTS(UNION_ATTR, CLASS_NAME, ATTR_NAME)                    \
} CLASS_NAME;                                                     \
LAYOUTS(DECLARE_MEMORY_INTERFACE, CLASS_NAME, ATTR_NAME, TYPE)    \
VIEW_STRUCT(LAYOUTS, CLASS_NAME, TYPE)                            \
LAYOUTS(VIEW_GETTER, LAYOUTS, CLASS_NAME, ATTR_NAME)              \
DECLARE_INTERFACE(LAYOUTS, CLASS_NAME, ATTR_NAME, TYPE)


MAKE_LAYOUT_STRUCTS(JOINT_RESULT, JointResult, joint_result, RESULT)
MAKE_LAYOUT_STRUCTS(SOLID_RESULT, SolidResult, solid_result, RESULT)
MAKE_LAYOUT_STRUCTS(JOINT_DESCRIPTION, JointDescription, joint_description, DESCRIPTION)
MAKE_LAYOUT_STRUCTS(SOLID_DESCRIPTION, SolidDescription, solid_description, DESCRIPTION)


#endif //INTERFACE_H
