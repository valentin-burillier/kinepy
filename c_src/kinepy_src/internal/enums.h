#ifndef ENUMS_H
#define ENUMS_H

typedef enum {
    KINEPY_SUCCESS,
    KINEPY_FAILURE,
    KINEPY_INVALID_INPUT,
    KINEPY_INVALID_INPUT_NEGATIVE_INERTIAL_VALUE,
    KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS,
    KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX,
    KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE,
    KINEPY_INVALID_INPUT_WRONG_RELATION_TYPE,
    KINEPY_INVALID_INPUT_UNMATCHED_RELATION_TYPE_AND_JOINT_TYPES,
    KINEPY_NO_GRAPH_FOUND,
    KINEPY_INVALID_CONFIGURATION_HYPERSTATIC_SYSTEM,
    KINEPY_INVALID_CONFIGURATION_HYPOSTATIC_SYSTEM,
    KINEPY_INVALID_CONFIGURATION_HYPERSTATIC_RELATION_SCHEME,
    KINEPY_INVALID_CONFIGURATION_GEAR_RELATION_WITH_NO_COMMON_EQ,
    KINEPY_MALLOC_FAILED
} KpResult;

#endif //ENUMS_H
