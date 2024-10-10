#include "gtest/gtest.h"
#include <vector>
extern "C" {
    #include "graphs.h"
}

void isostatic_to_user(JointType const * const iso_graph, uint32_t const size, std::vector<GraphNode>& result) {
    for (int index = 0; index < size; ++index) {
        result[index].type = iso_graph[index];
        result[index].joint_index = -1;
    }
}

TEST(FindIsomorphism, Identity) {
    std::vector<GraphNode> user_graph(adjacency_size(3));
    for (int g = 0; g < ISOSTATIC_GRAPH_COUNT; ++g) {
        IsostaticGraphInfo const & target = ISOSTATIC_GRAPHS[g];
        isostatic_to_user(target.adjacency, adjacency_size(target.vertex_count), user_graph);

        uint32_t result_graph;
        uint32_t * result_isomorphism;
        result_graph = find_isomorphism(user_graph.data(), target.degrees, target.vertex_count, &result_isomorphism);
        std::vector<uint32_t> ri;


        EXPECT_EQ(result_graph, g);

        std::vector<uint32_t> expected(target.vertex_count);
        for (int i = 0; i < expected.size(); ++i) expected[i] = i;
        if (result_isomorphism) {
            EXPECT_TRUE(std::equal(result_isomorphism, result_isomorphism + target.vertex_count,
                                   expected.begin()));

        }
        free(result_isomorphism);
    }
}