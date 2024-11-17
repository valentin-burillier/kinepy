#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "stdint.h"

typedef enum {
    JOINT_TYPE_EMPTY,
    JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_PRISMATIC,
    JOINT_TYPE_COUNT
} JointType;

typedef enum {
    RELATION_TYPE_EMPTY,
    RELATION_TYPE_GEAR,
    RELATION_TYPE_GEAR_RACK,
    RELATION_TYPE_DISTANT,
    RELATION_TYPE_EFFORTLESS,
    RELATION_TYPE_COUNT
} RelationType;

typedef struct {
    uint32_t solid_count;
    uint32_t joint_count;
    uint32_t relation_count;
    struct {
        uint8_t type;
        uint32_t solid1;
        uint32_t solid2;
    } * joints;
    struct {
        uint8_t type;
        uint32_t joint1;
        uint32_t joint2;
    } * relations;
} Configuration;

uint32_t kp_configure_joint(Configuration * config, uint32_t joint_index, uint8_t type, uint32_t solid1, uint32_t solid2);
uint32_t kp_configure_relation(Configuration * config, uint32_t relation_index, uint8_t type, uint32_t joint1, uint32_t joint2);

#endif //CONFIGURATION_H
