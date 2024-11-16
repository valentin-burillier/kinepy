#include "graph_interface_template.h"

uint32_t determine_computation_order(System const * system, ResolutionMode * resolution_mode) {
    return internal_determine_computation_order((system_internal*) system, resolution_mode);
}
