#ifndef GRAPH_GRAPH_INTERFACE_H

#include "interface/configuration.h"
#include "public_structs.h"

uint32_t kp_determine_computation_order(Configuration const * config, ResolutionMode * resolution_mode);
void kp_clear_resolution_steps(ResolutionMode * const resolution_mode);

#define GRAPH_GRAPH_INTERFACE_H
#endif //GRAPH_GRAPH_INTERFACE_H
