import unittest
import kinepy.strategy.algorithm as algo
import kinepy.strategy.graph_data as graph_data


def data_graph_to_user_graph(adj: graph_data.Adjacency) -> algo.JointGraph:
    result_graph: algo.JointGraph = [[algo.JointGraphNode(v) for v in line] for line in adj]
    return result_graph


def apply_isomorphism(graph: algo.JointGraph, iso: algo.Isomorphism) -> algo.JointGraph:
    result = [[algo.JointGraphNode(algo.NodeType.EMPTY) for _ in graph] for _ in graph]

    for x, line in enumerate(graph):
        new_x = iso[x]
        for y, node in enumerate(line):
            new_y = iso[y]
            result[new_x][new_y] = node
    return result


JGN = algo.JointGraphNode

example_graph: algo.JointGraph = [
    [JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 0), JGN(algo.NodeType.REVOLUTE, 1), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY)      ],
    [JGN(algo.NodeType.REVOLUTE, 0), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 2), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 3), JGN(algo.NodeType.EMPTY)      ],
    [JGN(algo.NodeType.REVOLUTE, 1), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 4), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY)      ],
    [JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 2), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 5), JGN(algo.NodeType.REVOLUTE, 7)],
    [JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 4), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 8), JGN(algo.NodeType.REVOLUTE, 6)],
    [JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 3), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 5), JGN(algo.NodeType.REVOLUTE, 8), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY)      ],
    [JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 7), JGN(algo.NodeType.REVOLUTE, 6), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY)      ]
]
r"""
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
      R7  R6     
       \ /     
        6     
"""


example_graph_merge0: algo.JointGraph = [
    [JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 0), JGN(algo.NodeType.REVOLUTE, 1), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY)      ],
    [JGN(algo.NodeType.REVOLUTE, 0), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 8), JGN(algo.NodeType.REVOLUTE, 7)],
    [JGN(algo.NodeType.REVOLUTE, 1), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 4), JGN(algo.NodeType.EMPTY)      ],
    [JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 8), JGN(algo.NodeType.REVOLUTE, 4), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 6)],
    [JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 7), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 6), JGN(algo.NodeType.EMPTY)      ]
]
r"""
            0
           / \
          R0  R1
         /     \
        1       2
       / \     /
      R7  R8  R4
     /     \ /
    4 ----- 3
        R6      
"""
example_eqs_merge0: algo.Eq = (0,), (1, 5, 3), (2,), (4,), (6,)
example_solid_to_eq_merge0: algo.EqMapping = 0, 1, 2, 1, 3, 1, 4

example_graph_merge1: algo.JointGraph = [
    [JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 0), JGN(algo.NodeType.REVOLUTE, 1)],
    [JGN(algo.NodeType.REVOLUTE, 0), JGN(algo.NodeType.EMPTY),       JGN(algo.NodeType.REVOLUTE, 4)],
    [JGN(algo.NodeType.REVOLUTE, 1), JGN(algo.NodeType.REVOLUTE, 4), JGN(algo.NodeType.EMPTY),     ]
]
r"""
        0
       / \
      R0  R1
     /     \
    1 ----- 2
        R4
"""
example_eqs_merge1: algo.Eq = (0,), (1, 5, 3, 4, 6), (2,)
example_solid_to_eq_merge1: algo.EqMapping = 0, 1, 2, 1, 1, 1, 1


def make_isomorphisms(size):
    if not size:
        yield ()
        return

    for _sub_iso in make_isomorphisms(size-1):
        for position in range(size):
            yield _sub_iso[:position] + (size-1,) + _sub_iso[position:]


class StrategyTests(unittest.TestCase):
    def test_find_isomorphism_identity(self) -> None:
        """
        Try to identify each registered graph when simply converted as user graph
        """
        for index, adj in enumerate(graph_data.ADJACENCY):
            graph = data_graph_to_user_graph(adj)
            iso = algo.find_isomorphism(graph)
            self.assertFalse(iso is None, f'{index}')
            graph_index, isomorphism = iso
            self.assertEqual(graph_index, index)

    def test_find_isomorphism_shuffled(self) -> None:
        """
        Try to identify each registered dyad and triad when shuffled by any isomorphism
        """
        graph_isomorphisms: tuple[algo.Isomorphism, ...] = 3 * (tuple(make_isomorphisms(3)),) + 10 * (tuple(make_isomorphisms(5)),)

        for index, (adj, iso_group) in enumerate(zip(graph_data.ADJACENCY, graph_isomorphisms)):
            graph = data_graph_to_user_graph(adj)
            for target_iso in iso_group:
                _graph = apply_isomorphism(graph, target_iso)
                iso = algo.find_isomorphism(_graph)
                self.assertFalse(iso is None, f'{index} : {target_iso}')
                graph_index, isomorphism = iso
                self.assertEqual(graph_index, index)

    def test_merge(self) -> None:
        eqs0 = tuple((i,) for i in range(len(example_graph)))
        merged_graph, merged_eqs, merged_mapping = algo.merge(example_graph, eqs0, (1, 3, 5))
        self.assertEqual(merged_mapping, example_solid_to_eq_merge0)
        self.assertEqual(merged_graph, example_graph_merge0)

        merged_graph, merged_eqs, merged_mapping = algo.merge(example_graph_merge0, example_eqs_merge0, (4, 3, 1))
        self.assertEqual(merged_mapping, example_solid_to_eq_merge1)
        self.assertEqual(merged_graph, example_graph_merge1)

        merged_graph, merged_eqs, merged_mapping = algo.merge(example_graph_merge1, example_eqs_merge1, (1, 2, 0))
        self.assertEqual(merged_mapping, (0,) * 7)
        self.assertEqual(merged_graph, [[JGN(algo.NodeType.EMPTY)]])


if __name__ == '__main__':
    unittest.main()
