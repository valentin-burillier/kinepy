#ifndef SOLID_RESULT_H
#define SOLID_RESULT_H

#include "base_layouts.h"

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


#endif //SOLID_RESULT_H
