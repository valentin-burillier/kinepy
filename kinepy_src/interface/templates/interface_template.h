#ifndef INTERFACE_INTERFACE_H
#include "stdint.h"


uint32_t allocate_solid_descriptions(System * system, uint32_t obj_count);
void free_solid_descriptions(System * system);
void get_solid_description(System const * system, uint32_t obj_index, SolidDescription * output);
uint32_t set_solid_description(System * system, uint32_t obj_index, SolidDescription const * input);

/**
 * Allows to update constraints of a revolute joint without the need redetermine the computation order
 * @param system
 * @param obj_index
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 */
uint32_t update_revolute_constraints(System * system, uint32_t obj_index, float_type x1, float_type y1, float_type x2, float_type y2);

/**
 * Allows to update constraints of a prismatic joint without the need redetermine the computation order
 * @param system
 * @param obj_index
 * @param alpha1
 * @param distance1
 * @param alpha2
 * @param distance2
 */
uint32_t update_prismatic_constraints(System * system, uint32_t obj_index, float_type alpha1, float_type distance1, float_type alpha2, float_type distance2);

uint32_t allocate_joint_descriptions(System * system, uint32_t obj_count);
void free_joint_descriptions(System * system);
void get_joint_description(System const * system, uint32_t obj_index, JointDescription * output);
uint32_t set_joint_description(System * system, uint32_t obj_index, JointDescription const * input);

/**
 * Allows to update relation parameters without the need redetermine the computation order
 * @param system
 * @param obj_count
 * @param ratio
 * @param v0
 */
void update_relation_parameters(System * system, uint32_t obj_count, float_type ratio, float_type v0);


uint32_t allocate_relation_descriptions(System * system, uint32_t obj_count);
void free_relation_descriptions(System * system);
void get_relation_description(System const * system, uint32_t obj_index, RelationDescription * output);
uint32_t set_relation_description(System * system, uint32_t obj_index, RelationDescription const * input);

#endif