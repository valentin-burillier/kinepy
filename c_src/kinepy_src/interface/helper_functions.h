#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include "configuration.h"

uint8_t is_existing_solid(KpConfiguration const *  config, uint32_t solid_index);
uint8_t is_existing_joint(KpConfiguration const * config, uint32_t joint_index);
uint8_t is_existing_joint_type(uint8_t type);
uint8_t is_existing_relation(KpConfiguration const * config, uint32_t relation_index);
uint8_t is_existing_relation_type(uint8_t type);
uint8_t is_valid_relation(KpConfiguration const * config, uint8_t type, uint32_t joint1, uint32_t joint2);

#endif //HELPER_FUNCTIONS_H
