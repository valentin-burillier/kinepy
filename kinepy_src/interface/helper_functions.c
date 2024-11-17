#include "helper_functions.h"

uint8_t is_existing_solid(Configuration const * const config, uint32_t const solid_index) {
    return solid_index < config->solid_count;
}


uint8_t is_existing_joint(Configuration const * const config, uint32_t const joint_index) {
    return joint_index < config->joint_count;
}
uint8_t is_existing_joint_type(uint8_t type) {
    return JOINT_TYPE_EMPTY < type && type < JOINT_TYPE_COUNT;
}
uint8_t is_existing_relation(Configuration const * config, uint32_t relation_index) {
    return relation_index < config->relation_count;
}
uint8_t is_existing_relation_type(uint8_t type) {
    return RELATION_TYPE_EMPTY < type && type < RELATION_TYPE_COUNT;
}

uint8_t is_valid_relation(Configuration const * const config, uint8_t type, uint32_t joint1, uint32_t joint2) {
    switch (type) {
        case RELATION_TYPE_GEAR:
            return config->joints[joint1].type == JOINT_TYPE_REVOLUTE && config->joints[joint2].type == JOINT_TYPE_REVOLUTE;
        case RELATION_TYPE_GEAR_RACK:
            return config->joints[joint1].type == JOINT_TYPE_REVOLUTE && config->joints[joint2].type == JOINT_TYPE_PRISMATIC;
        default:
            return 1;
    }
}