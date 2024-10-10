#ifndef BASE_LAYOUTS_H
#define BASE_LAYOUTS_H

#define STRUCT_ATTRIBUTE(TYPE, NAME, ...) TYPE NAME;

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


#endif //BASE_LAYOUTS_H
