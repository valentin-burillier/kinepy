#ifndef JOINT_DESCRIPTION_H
#define JOINT_DESCRIPTION_H

#include "base_layouts.h"

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

#endif //JOINT_DESCRIPTION_H
