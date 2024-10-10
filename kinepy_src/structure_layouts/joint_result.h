#ifndef JOINT_RESULT_H
#define JOINT_RESULT_H

#include "base_layouts.h"

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

#endif //JOINT_RESULT_H
