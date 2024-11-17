#include "configuration.h"
#include "helper_functions.h"
#include "internal/enums.h"

uint32_t kp_configure_joint(Configuration * const config, uint32_t const joint_index, uint8_t const type, uint32_t const solid1, uint32_t const solid2) {
    if (!is_existing_joint(config, joint_index) || !is_existing_solid(config, solid1) || !is_existing_solid(config, solid2)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    if (solid1 == solid2) {
        return KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS;
    }
    if (!is_existing_joint_type(type)) {
        return KINEPY_INVALID_INPUT_WRONG_JOINT_TYPE;
    }
    typeof(config->joints) joint = config->joints + joint_index;
    joint->solid1 = solid1;
    joint->solid2 = solid2;
    joint->type = type;

    return KINEPY_SUCCESS;
}

uint32_t kp_configure_relation(Configuration * const config, uint32_t relation_index, uint8_t const type, uint32_t const joint1, uint32_t const joint2) {
    if (joint1 == joint2) {
        return KINEPY_INVALID_INPUT_IDENTICAL_OBJECTS;
    }
    if (!is_existing_relation(config, relation_index) || !is_existing_joint(config, joint1) && !is_existing_joint(config, joint2)) {
        return KINEPY_INVALID_INPUT_WRONG_OBJECT_INDEX;
    }
    if (!is_existing_relation_type(type)) {
        return KINEPY_INVALID_INPUT_WRONG_RELATION_TYPE;
    }
    if (!is_valid_relation(config, type, joint1, joint2)) {
        return KINEPY_INVALID_INPUT_UNMATCHED_RELATION_TYPE_AND_JOINT_TYPES;
    }

    typeof(config->relations) relation = config->relations + relation_index;
    relation->joint1 = joint1;
    relation->joint2 = joint2;
    relation->type = type;

    return KINEPY_SUCCESS;
}