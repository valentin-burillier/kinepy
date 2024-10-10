#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>
#include <stdlib.h>
#define STRUCT_ATTRIBUTE(TYPE, NAME, ...) TYPE NAME;
#define STRUCT_ARRAY_ATTRIBUTE(TYPE, NAME, ...) TYPE * NAME##_array;

#define MAKE_RESULT_STRUCTS(ATTRIBUTES, NAME) \
typedef struct NAME {ATTRIBUTES(STRUCT_ATTRIBUTE)} NAME; \
typedef struct NAME##Array {uint8_t index_mode; uint32_t base_count; uint32_t frame_count; ATTRIBUTES(STRUCT_ARRAY_ATTRIBUTE)} NAME##Array;

#define MAKE_DESCRIPTION_STRUCTS(ATTRIBUTES, NAME) \
typedef struct NAME {ATTRIBUTES(STRUCT_ATTRIBUTE)} NAME; \
typedef struct NAME##Array {uint32_t count; ATTRIBUTES(STRUCT_ARRAY_ATTRIBUTE)} NAME##Array;

#define POINT_LAYOUT(CALLBACK, ...) \
     CALLBACK(float, x, SIMPLE, __VA_ARGS__)        \
     CALLBACK(float, y, SIMPLE, __VA_ARGS__)

typedef struct Point {
    POINT_LAYOUT(STRUCT_ATTRIBUTE)
} Point;

#define INERTIA_LAYOUT(CALLBACK, ...) \
     CALLBACK(float, mass, SIMPLE, __VA_ARGS__)     \
     CALLBACK(float, moment, SIMPLE, __VA_ARGS__)

typedef struct Inertia {
    INERTIA_LAYOUT(STRUCT_ATTRIBUTE)
} Inertia;

#define OBJECT_PAIR_LAYOUT(CALLBACK, ...) \
    CALLBACK(uint32_t, obj1, SIMPLE, __VA_ARGS__)         \
    CALLBACK(uint32_t, obj2, SIMPLE, __VA_ARGS__)

typedef struct ObjectPair {
    OBJECT_PAIR_LAYOUT(STRUCT_ATTRIBUTE)
} ObjectPair;

#define SIMPLE(CALLBACK, TYPE, NAME, ...) SIMPLE_##CALLBACK(TYPE, NAME, _,  TYPE, NAME, __VA_ARGS__)

#pragma region SolidDescriptions
#define SOLID_DESCRIPTION_LAYOUT_0(CALLBACK, ...) \
    CALLBACK(float, mass, SIMPLE, __VA_ARGS__)                       \
    CALLBACK(float, moment_of_inertia, SIMPLE, __VA_ARGS__)          \
    CALLBACK(float, gx, SIMPLE, __VA_ARGS__)                         \
    CALLBACK(float, gy, SIMPLE, __VA_ARGS__)


#define SOLID_DESCRIPTION_LAYOUT_1(CALLBACK, ...)    \
    CALLBACK(float, mass, SIMPLE, __VA_ARGS__)               \
    CALLBACK(float, moment_of_inertia, SIMPLE, __VA_ARGS__)  \
    CALLBACK(Point, g, POINT_LAYOUT, __VA_ARGS__)

#define SOLID_DESCRIPTION_LAYOUT_2(CALLBACK, ...)    \
    CALLBACK(Inertia, inertia, INERTIA_LAYOUT, __VA_ARGS__)  \
    CALLBACK(Point, g, POINT_LAYOUT, __VA_ARGS__)

#define SOLID_DESCRIPTION_LAYOUT_3(CALLBACK, ...)                        \
    CALLBACK(SolidDescription0, solid, SOLID_DESCRIPTION_LAYOUT_0, __VA_ARGS__)

#define SOLID_DESCRIPTION(CALLBACK, ...) \
    CALLBACK(__VA_ARGS__, 0)              \
    CALLBACK(__VA_ARGS__, 1)              \
    CALLBACK(__VA_ARGS__, 2)              \
    CALLBACK(__VA_ARGS__, 3)

#pragma endregion

#pragma region SolidResults
#define SOLID_RESULT_LAYOUT_0(CALLBACK, ...) \
    CALLBACK(float, x, SIMPLE, __VA_ARGS__)          \
    CALLBACK(float, y, SIMPLE, __VA_ARGS__)          \
    CALLBACK(float, angle, SIMPLE, __VA_ARGS__)

#define SOLID_RESULT_LAYOUT_1(CALLBACK, ...)     \
    CALLBACK(Point, position, POINT_LAYOUT, __VA_ARGS__) \
    CALLBACK(float, angle, SIMPLE, __VA_ARGS__)

#define SOLID_RESULT_LAYOUT_2(CALLBACK, ...)                     \
    CALLBACK(SolidResult0, result, SOLID_RESULT_LAYOUT_0, __VA_ARGS__)


#define SOLID_RESULT(CALLBACK, ...) \
    CALLBACK(__VA_ARGS__, 0)         \
    CALLBACK(__VA_ARGS__, 1)         \
    CALLBACK(__VA_ARGS__, 2)

#pragma endregion

#pragma region JointDescriptions

typedef enum JointType {
    JOINT_TYPE_EMPTY,
    JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_PRISMATIC
} JointType;


#define JOINT_DESCRIPTION_LAYOUT_0(CALLBACK, ...) \
    CALLBACK(JointType, type, SIMPLE, __VA_ARGS__)        \
    CALLBACK(uint32_t, solid1, SIMPLE, __VA_ARGS__)       \
    CALLBACK(uint32_t, solid2, SIMPLE, __VA_ARGS__)       \
    CALLBACK(float, x1, SIMPLE, __VA_ARGS__)              \
    CALLBACK(float, y1, SIMPLE, __VA_ARGS__)              \
    CALLBACK(float, x2, SIMPLE, __VA_ARGS__)              \
    CALLBACK(float, y2, SIMPLE, __VA_ARGS__)

#define JOINT_DESCRIPTION_LAYOUT_1(CALLBACK, ...) \
    CALLBACK(JointType, type, SIMPLE, __VA_ARGS__)        \
    CALLBACK(uint32_t, solid1, SIMPLE, __VA_ARGS__)       \
    CALLBACK(uint32_t, solid2, SIMPLE, __VA_ARGS__)       \
    CALLBACK(Point, p1, POINT_LAYOUT, __VA_ARGS__)        \
    CALLBACK(Point, p2, POINT_LAYOUT, __VA_ARGS__)

#define JOINT_DESCRIPTION_LAYOUT_2(CALLBACK, ...)        \
    CALLBACK(JointType, type, SIMPLE, __VA_ARGS__)               \
    CALLBACK(ObjectPair, solids, OBJECT_PAIR_LAYOUT, __VA_ARGS__)\
    CALLBACK(Point, p1, POINT_LAYOUT, __VA_ARGS__)               \
    CALLBACK(Point, p2, POINT_LAYOUT, __VA_ARGS__)

#define JOINT_DESCRIPTION_LAYOUT_3(CALLBACK, ...)                        \
    CALLBACK(JointDescription0, joint, JOINT_DESCRIPTION_LAYOUT_0, __VA_ARGS__)

#define JOINT_DESCRIPTION(CALLBACK, ...) \
    CALLBACK(__VA_ARGS__, 0)              \
    CALLBACK(__VA_ARGS__, 1)              \
    CALLBACK(__VA_ARGS__, 2)              \
    CALLBACK(__VA_ARGS__, 3)

#pragma endregion

#pragma region JointResults

#define JOINT_RESULT_LAYOUT_0(CALLBACK, ...)         \
    CALLBACK(float, kinematic_value, SIMPLE, __VA_ARGS__)    \
    CALLBACK(float, force_x, SIMPLE, __VA_ARGS__)            \
    CALLBACK(float, force_y, SIMPLE, __VA_ARGS__)            \
    CALLBACK(float, torque, SIMPLE, __VA_ARGS__)

#define JOINT_RESULT_LAYOUT_1(CALLBACK, ...)         \
    CALLBACK(float, kinematic_value, SIMPLE, __VA_ARGS__)    \
    CALLBACK(Point, force, POINT_LAYOUT, __VA_ARGS__)        \
    CALLBACK(float, torque, SIMPLE, __VA_ARGS__)

#define JOINT_RESULT_LAYOUT_2(CALLBACK, ...) \
    CALLBACK(JointResult0, joint, JOINT_RESULT_LAYOUT_0, __VA_ARGS__)

#define JOINT_RESULT(CALLBACK, ...) \
    CALLBACK(__VA_ARGS__, 0)         \
    CALLBACK(__VA_ARGS__, 1)         \
    CALLBACK(__VA_ARGS__, 2)

#pragma endregion

#define CROSS_PRODUCT(FIRST, ...) FIRST(__VA_ARGS__)

#define STRUCT_NAMES(LAYOUTS, CLASS_NAME, TYPE, ID) MAKE_##TYPE##_STRUCTS(LAYOUTS##_LAYOUT_##ID, CLASS_NAME##ID)
#define UNION_ATTR(CLASS_NAME, ATTR_NAME, ID) CLASS_NAME##ID __##ATTR_NAME##_##ID;

#define DECLARE_DESCRIPTION_INTERFACE(CLASS_NAME, ATTR_NAME, ID)                                        \
uint8_t allocate_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * array, size_t ATTR_NAME##_count);                      \
void free_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * array);

#define DECLARE_RESULT_INTERFACE(CLASS_NAME, ATTR_NAME, ID)                                        \
uint8_t allocate_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * array, uint8_t index_mode, size_t description_count, size_t frame_count);  \
void free_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * array);

#define VIEW_GETTER(CLASS, CLASS_NAME, ATTR_NAME, ID) \
void get_##ATTR_NAME##_##ID##_array_view(CLASS_NAME##ID##Array const * array, CLASS_NAME##ArrayView * out_view);


#define VIEW_ATTRIBUTE(TYPE, NAME, ...) void * NAME##_ptr; size_t NAME##_offset;
#define DESCRIPTION_VIEW(CLASS, CLASS_NAME) \
typedef struct CLASS_NAME##ArrayView {                 \
    uint32_t count;                                    \
    CLASS##_LAYOUT_0(VIEW_ATTRIBUTE)                   \
} CLASS_NAME##ArrayView;

#define RESULT_VIEW(CLASS, CLASS_NAME) \
typedef struct CLASS_NAME##ArrayView { \
    uint32_t index_mode;               \
    uint32_t base_count;               \
    uint32_t frame_count;              \
    CLASS##_LAYOUT_0(VIEW_ATTRIBUTE)   \
} CLASS_NAME##ArrayView;

#define DESCRIPTION_DECLARE_GET_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) TYPE get_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * view, size_t index);
#define DESCRIPTION_DECLARE_SET_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) void set_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * view, size_t index, TYPE value);
#define RESULT_DECLARE_GET_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) TYPE get_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * view, size_t index, size_t frame);
#define RESULT_DECLARE_SET_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) void set_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * view, size_t index, size_t frame, TYPE value);

#define DECLARE_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
CLASS##_LAYOUT_0(TYPE##_DECLARE_GET_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME) \
CLASS##_LAYOUT_0(TYPE##_DECLARE_SET_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME) \
TYPE##_DECLARE_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME)

#define RESULT_DECLARE_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME) \
void get_##ATTR_NAME(CLASS_NAME##ArrayView const * view, size_t index, size_t frame_index, CLASS_NAME * output); \
void set_##ATTR_NAME(CLASS_NAME##ArrayView const * view, size_t index, size_t frame_index, CLASS_NAME const * input);

#define DESCRIPTION_DECLARE_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME) \
void get_##ATTR_NAME(CLASS_NAME##ArrayView const * view, size_t index, CLASS_NAME * output); \
void set_##ATTR_NAME(CLASS_NAME##ArrayView const * view, size_t index, CLASS_NAME const * input);

#define MAKE_LAYOUT_STRUCTS(LAYOUTS, CLASS_NAME, ATTR_NAME, TYPE) \
LAYOUTS(STRUCT_NAMES, LAYOUTS, CLASS_NAME, TYPE)                  \
typedef union CLASS_NAME {                                        \
    struct {                                                      \
         LAYOUTS##_LAYOUT_0(STRUCT_ATTRIBUTE)                     \
    };                                                            \
    LAYOUTS(UNION_ATTR, CLASS_NAME, ATTR_NAME)                    \
} CLASS_NAME;                                                     \
LAYOUTS(DECLARE_##TYPE##_INTERFACE, CLASS_NAME, ATTR_NAME)        \
TYPE##_VIEW(LAYOUTS, CLASS_NAME)                                  \
LAYOUTS(VIEW_GETTER, LAYOUTS, CLASS_NAME, ATTR_NAME)              \
DECLARE_INTERFACE(LAYOUTS, CLASS_NAME, ATTR_NAME, TYPE)


MAKE_LAYOUT_STRUCTS(JOINT_RESULT, JointResult, joint_result, RESULT)
MAKE_LAYOUT_STRUCTS(SOLID_RESULT, SolidResult, solid_result, RESULT)
MAKE_LAYOUT_STRUCTS(JOINT_DESCRIPTION, JointDescription, joint_description, DESCRIPTION)
MAKE_LAYOUT_STRUCTS(SOLID_DESCRIPTION, SolidDescription, solid_description, DESCRIPTION)


#define ALLOCATE_ATTRIBUTE(TYPE, NAME, ...) array->NAME##_array = malloc(total * sizeof(TYPE)); if (!array->NAME##_array) {goto error_allocate;} ++allocated;
#define ERR_ALLOCATE_ATTRIBUTE(TYPE, NAME, ...) if (!allocated) {return 0;} free(array->NAME##_array); --allocated;
#define DESCRIPTION_ALLOCATOR(CLASS, CLASS_NAME, ATTR_NAME, ID) \
uint8_t allocate_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * const array, size_t const ATTR_NAME##_count) { \
    array->count = ATTR_NAME##_count;                                                       \
    size_t const total = ATTR_NAME##_count;                     \
    uint8_t allocated = 0;                                                                \
                                                                \
    CLASS##_LAYOUT_##ID(ALLOCATE_ATTRIBUTE)\
    return 1;                                                                \
error_allocate:                                                 \
    CLASS##_LAYOUT_##ID(ERR_ALLOCATE_ATTRIBUTE)\
    return 0;                                                                \
}
#define RESULT_ALLOCATOR(CLASS, CLASS_NAME, ATTR_NAME, ID) \
uint8_t allocate_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * const array, uint8_t const index_mode, size_t const description_count, size_t const frame_count) { \
    array->index_mode = index_mode;                                     \
    array->base_count = description_count;                 \
    array->frame_count = frame_count;                                                       \
    size_t const total = description_count * frame_count;                     \
    uint8_t allocated = 0;                                                                \
                                                                \
    CLASS##_LAYOUT_##ID(ALLOCATE_ATTRIBUTE)\
    return 1;                                                                \
error_allocate:                                                 \
    CLASS##_LAYOUT_##ID(ERR_ALLOCATE_ATTRIBUTE)\
    return 0;                                                                \
}

#define FREE_ATTRIBUTE(TYPE, NAME, ...) free(array->NAME##_array);
#define DESCRIPTION_DE_ALLOCATOR(CLASS, CLASS_NAME, ATTR_NAME, ID) \
void free_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * array) { \
    array->count = 0;                                           \
    CLASS##_LAYOUT_##ID(FREE_ATTRIBUTE)                                                                   \
}

#define RESULT_DE_ALLOCATOR(CLASS, CLASS_NAME, ATTR_NAME, ID) \
void free_##ATTR_NAME##s_##ID(CLASS_NAME##ID##Array * array) { \
    array->index_mode = -1;                                     \
    array->base_count = 0;                 \
    array->frame_count = 0;                                           \
    CLASS##_LAYOUT_##ID(FREE_ATTRIBUTE)                                                                   \
}

#define SIMPLE_GET_VIEW_ATTRIBUTE(TYPE, NAME, ...) *(void**)attribute = (void*)array->NAME##_array; attribute += sizeof(void*); *(size_t*)attribute = sizeof(TYPE); attribute += sizeof(size_t);
#define GET_VIEW_ATTRIBUTE(TYPE, NAME, _GARBAGE, PARENT_TYPE, PARENT_NAME, ...) *(void**)attribute = (void*)&(array->PARENT_NAME##_array->NAME); attribute += sizeof(void*); *(size_t*)attribute = sizeof(PARENT_TYPE); attribute += sizeof(size_t);
#define GET_VIEW_BODY(TYPE, NAME, LAYOUT, ...) LAYOUT(GET_VIEW_ATTRIBUTE, TYPE, NAME)

#define DESCRIPTION_VIEW_GETTER(CLASS, CLASS_NAME, ATTR_NAME, ID) \
void get_##ATTR_NAME##_##ID##_array_view(CLASS_NAME##ID##Array const * const array, CLASS_NAME##ArrayView * const out_view) {\
    out_view->count = array->count;                                                                 \
    void * attribute = (void*) out_view + offsetof(CLASS_NAME##ArrayView, count) + sizeof(out_view->count);                                                        \
    CLASS##_LAYOUT_##ID(GET_VIEW_BODY)\
}
#define RESULT_VIEW_GETTER(CLASS, CLASS_NAME, ATTR_NAME, ID) \
void get_##ATTR_NAME##_##ID##_array_view(CLASS_NAME##ID##Array const * const array, CLASS_NAME##ArrayView * const out_view) {\
    out_view->index_mode = array->index_mode;                                     \
    out_view->base_count = array->base_count;                 \
    out_view->frame_count = array->frame_count;                                                                \
    void * attribute = (void*) out_view + offsetof(CLASS_NAME##ArrayView, frame_count) + sizeof(out_view->frame_count);                                                        \
    CLASS##_LAYOUT_##ID(GET_VIEW_BODY)\
}

#define DESCRIPTION_GET_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) \
TYPE get_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * const view, size_t const index) {          \
    return *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * index);\
}
#define DESCRIPTION_SET_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) \
void set_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * const view, size_t const index, TYPE const value) { \
    *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * index) = value;\
}

#define RESULT_INDEX index * (view->index_mode * view->frame_count + 1 - view->index_mode) + frame * ((1-view->index_mode) * view->base_count + view->index_mode)
#define RESULT_GET_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) \
TYPE get_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * const view, size_t const index, size_t const frame) { \
    size_t const actual_index = RESULT_INDEX; \
    return *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * actual_index);\
}

#define RESULT_SET_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) \
void set_##ATTR_NAME##_##NAME(CLASS_NAME##ArrayView const * const view, size_t const index, size_t const frame, TYPE const value) { \
    size_t const actual_index = RESULT_INDEX; \
    *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * actual_index) = value;\
}

#define GET_GENERIC_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) output->NAME = *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * actual_index);
#define SET_GENERIC_ATTRIBUTE(TYPE, NAME, _GARBAGE, CLASS, CLASS_NAME, ATTR_NAME, ...) *(TYPE*)(view->NAME##_ptr + view->NAME##_offset * actual_index) = input->NAME;

#define DESCRIPTION_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
void get_##ATTR_NAME(CLASS_NAME##ArrayView const * const view, size_t const index, CLASS_NAME * const output) { \
    size_t const actual_index = index;                         \
    CLASS##_LAYOUT_0(GET_GENERIC_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME) \
} \
void set_##ATTR_NAME(CLASS_NAME##ArrayView const * const view, size_t index, CLASS_NAME const * const input) {        \
    size_t const actual_index = index;                         \
    CLASS##_LAYOUT_0(SET_GENERIC_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME) \
}

#define RESULT_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
void get_##ATTR_NAME(CLASS_NAME##ArrayView const * const view, size_t const index, size_t const frame, CLASS_NAME * const output) { \
    size_t const actual_index = RESULT_INDEX;                         \
    CLASS##_LAYOUT_0(GET_GENERIC_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME) \
} \
void set_##ATTR_NAME(CLASS_NAME##ArrayView const * const view, size_t const index, size_t const frame, CLASS_NAME const * const input) {        \
    size_t const actual_index = RESULT_INDEX;                         \
    CLASS##_LAYOUT_0(SET_GENERIC_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME) \
}


#define INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE) \
CLASS(TYPE##_ALLOCATOR, CLASS, CLASS_NAME, ATTR_NAME) \
CLASS(TYPE##_DE_ALLOCATOR, CLASS, CLASS_NAME, ATTR_NAME) \
CLASS(TYPE##_VIEW_GETTER, CLASS, CLASS_NAME, ATTR_NAME)  \
CLASS##_LAYOUT_0(TYPE##_GET_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME)\
CLASS##_LAYOUT_0(TYPE##_SET_ATTRIBUTE, CLASS, CLASS_NAME, ATTR_NAME)\
TYPE##_INTERFACE(CLASS, CLASS_NAME, ATTR_NAME, TYPE)


typedef struct System {
    SolidDescriptionArrayView solids;
    JointDescriptionArrayView joints;
    SolidResultArrayView solid_results;
    JointResultArrayView joint_results;
} System;

#endif //SYSTEM_H