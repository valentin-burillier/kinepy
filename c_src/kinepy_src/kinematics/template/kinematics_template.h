#ifndef KINEMATICS_KINEMATICS_H

#include "interface/templates/structs_template.h"
#include "graph/public_structs.h"
#include "graph/graph_data.h"


typedef struct {
    uint32_t solid_index;
    float_type px;
    float_type py;
} PointDescription;


typedef void (*GraphSolver)(System const *, ResolutionStep const *, Result *, uint32_t, uint32_t);

void reset_results(KpConfiguration const * configuration, Result * result, uint32_t start_index, uint32_t end_index);

void solve_graph_rrr(System const * system, ResolutionStep const * step, Result * result, uint32_t start_index, uint32_t end_index);
void solve_graph_rrp(System const * system, ResolutionStep const * step, Result * result, uint32_t start_index, uint32_t end_index);
void solve_graph_ppr(System const * system, ResolutionStep const * step, Result * result, uint32_t start_index, uint32_t end_index);


extern GraphSolver graph_solvers[ISOSTATIC_GRAPH_COUNT];

#endif // KINEMATICS_KINEMATICS_H
