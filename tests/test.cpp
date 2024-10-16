#include "gtest/gtest.h"
#include <vector>
#include <cstdlib>
#include <cmath>

extern "C" {
    #include "graphs.h"
}

void isostatic_to_user(JointType const * const iso_graph, uint32_t const size, std::vector<GraphNode>& result) {
    for (int index = 0; index < size; ++index) {
        result[index].type = iso_graph[index];
        result[index].joint_index = -1;
    }
}
void isostatic_isomorphism(IsostaticGraphInfo const * const src, std::vector<uint32_t> const & isomorphism, std::vector<GraphNode>& result_graph, std::vector<JointDegree>& result_degree) {
    for (int x = 0; x < src->vertex_count; ++x) {
        uint32_t new_x = isomorphism[x];
        result_degree[new_x] = src->degrees[x];
        for (int y = x+1; y < src->vertex_count; ++y) {
            uint32_t new_y = isomorphism[y];
            result_graph[graph_index(new_x, new_y, src->vertex_count)] = (GraphNode){.type=src->adjacency[certain_order_graph_index(x, y, src->vertex_count)], .joint_index=0};
        }
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
            EXPECT_TRUE(std::equal(result_isomorphism, result_isomorphism + target.vertex_count, expected.begin()));
        }
        free(result_isomorphism);
    }
}


TEST(FindIsomorphism, Shuffled) {
    for (int g = 0; g < ISOSTATIC_GRAPH_COUNT && ISOSTATIC_GRAPHS[g].vertex_count == 3; ++g) {
        IsostaticGraphInfo const &target = ISOSTATIC_GRAPHS[g];
        for (auto const &isomorphism: std::vector<std::vector<uint32_t>>{{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}) {
            std::vector<GraphNode> user_graph(adjacency_size(3));
            std::vector<JointDegree> user_degree(3);
            isostatic_isomorphism(&target, isomorphism, user_graph, user_degree);

            uint32_t result_graph;
            uint32_t *result_isomorphism;
            result_graph = find_isomorphism(user_graph.data(), user_degree.data(), target.vertex_count, &result_isomorphism);

            EXPECT_EQ(result_graph, g);
            free(result_isomorphism);
        }
    }
}

TEST(MergeGraph, simple) {
    /*
            0
           / \
          R0  R1
         /     \
        1       2
        |\      |
        | R3    |
        |  \    |
        R2  5   R4
        |  / \  |
        | R5  R8|
        |/     \|
        3       4
         \     /
          R6  R7
           \ /
            6
     */
    GraphNode const base_graph[] = {
        /* 0 */ {.type=JOINT_TYPE_REVOLUTE, .joint_index=0}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=1}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1},
        /* 1 */ {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=2}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=3}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1},
        /* 2 */ {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=4}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1},
        /* 3 */ {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=5}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=7},
        /* 4 */ {.type=JOINT_TYPE_REVOLUTE, .joint_index=8}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=6},
        /* 5 */ {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}
    };
    /*
            0
           / \
          R0  R1
         /     \
        1       2
        |  \    |
        |   R8  R4
        |    \  |
        R6      3
         \     /
          \   R7
           \ /
            4
     */
    GraphNode const target_graph[] = {
        /* 0 */ {.type=JOINT_TYPE_REVOLUTE, .joint_index=0}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=1}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1},
        /* 1 */ {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=8}, {.type=JOINT_TYPE_REVOLUTE, .joint_index=7},
        /* 2 */ {.type=JOINT_TYPE_REVOLUTE, .joint_index=4}, {.type=JOINT_TYPE_EMPTY, .joint_index=(uint32_t)-1},
        /* 3 */ {.type=JOINT_TYPE_REVOLUTE, .joint_index=6}
    };

    std::vector<uint32_t> merge_group{{5, 1, 3}};

    GraphNode const * new_graph = merge_graph(base_graph, 7, merge_group.data(), 3);

    for (int index = 0; index < adjacency_size(5); index++) {
        EXPECT_EQ(target_graph[index].joint_index, new_graph[index].joint_index);
        EXPECT_EQ(target_graph[index].type, new_graph[index].type);
    }
}