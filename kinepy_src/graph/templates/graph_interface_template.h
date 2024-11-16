#ifndef GRAPH_GRAPH_INTERFACE_H
#include "stdint.h"
#include "interface/templates/structs_template.h"
#include "graph/public_structs.h"

uint32_t determine_computation_order(System const * system, ResolutionMode * resolution_mode);

#endif