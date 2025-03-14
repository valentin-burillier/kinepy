#ifndef TEST_GRAPH_H
#define TEST_GRAPH_H

#include "graph/private_structs.h"
#include "graph/public_structs.h"
#include "graph/graph_data.h"
#include "internal/enums.h"
#include "graph/graph_interface.h"

int compare_degrees(JointDegree d1, JointDegree d2);
uint32_t find_isomorphism(Graph const * graph, uint32_t * result_graph_index, uint32_t ** result_isomorphism);
uint32_t merge_graph(Graph * graph, uint32_t const * group_to_merge, uint32_t group_size);


#endif //TEST_GRAPH_H
