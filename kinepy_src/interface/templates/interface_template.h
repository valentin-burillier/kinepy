#ifndef INTERFACE_INTERFACE_H
#include "structs_template.h"
#include "stdint.h"


/**
 * Allocates all memory needed to hold system configuration and parameters
 * @param system
 * @param solid_count
 * @param joint_count
 * @param relation_count
 * @return
 */
uint32_t allocate_system(System * system, uint32_t solid_count, uint32_t joint_count, uint32_t relation_count);
void free_system(System * system);

uint32_t update_solid_physics(System * system, uint32_t solid_index, float_type mass, float_type moment_of_inertia, float_type gx, float_type gy);

/**
 * Allows to update constraints of a revolute joint
 * There is no need to redetermine computation orders after this function
 * @param system
 * @param obj_index
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 */
uint32_t update_revolute_constraints(System * system, uint32_t joint_index, float_type x1, float_type y1, float_type x2, float_type y2);

/**
 * Allows to update constraints of a prismatic joint
 * There is no need to redetermine computation orders after this function
 * @param system
 * @param obj_index
 * @param alpha1
 * @param distance1
 * @param alpha2
 * @param distance2
 */
uint32_t update_prismatic_constraints(System * system, uint32_t joint_index, float_type alpha1, float_type distance1, float_type alpha2, float_type distance2);

/**
 * Allows to update relation parameters without the need redetermine the computation order
 * @param system
 * @param obj_count
 * @param ratio
 * @param v0
 */
void update_relation_parameters(System * system, uint32_t relation_index, float_type ratio, float_type v0);


uint32_t allocate_result(KpConfiguration const * config, uint32_t frame_count, Result * result);
void free_result(Result * result);

#endif