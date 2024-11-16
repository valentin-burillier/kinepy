#ifndef GRAPH_GRAPH_INTERFACE_H

#include "internal/define_names.h"

#define float_type float
#define type_suffix _s
#include "templates/graph_interface_template.h"
#undef float_type
#undef type_suffix

#define float_type double
#define type_suffix _d
#include "templates/graph_interface_template.h"
#undef float_type
#undef type_suffix

#include "internal/undef_names.h"

void kp_clear_resolution_steps(ResolutionMode * const resolution_mode);

#define GRAPH_GRAPH_INTERFACE_H
#endif //GRAPH_GRAPH_INTERFACE_H
