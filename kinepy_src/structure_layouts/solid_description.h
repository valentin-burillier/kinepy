#ifndef SOLID_DESCRIPTION_H
#define SOLID_DESCRIPTION_H

#include "base_layouts.h"

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


#endif //SOLID_DESCRIPTION_H
