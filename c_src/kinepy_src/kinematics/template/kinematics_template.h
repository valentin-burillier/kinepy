#ifndef KINEMATICS_KINEMATICS_H

#include "interface/templates/structs_template.h"
#include "graph/public_structs.h"

typedef struct {
    uint32_t solid_index;
    float_type px;
    float_type py;
} PointDescription;

void solve_graph_rrr(System const * system, ResolutionStep const * step, Result * result, uint32_t start_index, uint32_t end_index);
void solve_graph_rrp(System const * system, ResolutionStep const * step, Result * result, uint32_t start_index, uint32_t end_index);
void solve_graph_ppr(System const * system, ResolutionStep const * step, Result * result, uint32_t start_index, uint32_t end_index);

#endif // KINEMATICS_KINEMATICS_H
