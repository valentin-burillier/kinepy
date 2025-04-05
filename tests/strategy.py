import unittest
from kinepy.objects.config import Config
import kinepy.exceptions as kp_exs
import kinepy.strategy.algorithm as algo
import kinepy.strategy.graph_data as graph_data
import numpy as np


def data_graph_to_user_graph(adj: graph_data.Adjacency) -> algo.JointGraph:
    result_graph: algo.JointGraph = [[algo.JointGraphNode(v) for v in line] for line in adj]
    return result_graph


def apply_isomorphism(graph: algo.JointGraph, iso: algo.Isomorphism) -> algo.JointGraph:
    result = [[algo.JointGraphNode(algo.JointType.EMPTY) for _ in graph] for _ in graph]

    for x, line in enumerate(graph):
        new_x = iso[x]
        for y, node in enumerate(line):
            new_y = iso[y]
            result[new_x][new_y] = node
    return result


JGN = algo.JointGraphNode

example_graph: algo.JointGraph = [
    [JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 0), JGN(algo.JointType.REVOLUTE, 1), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY)],
    [JGN(algo.JointType.REVOLUTE, 0), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 2), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 3), JGN(algo.JointType.EMPTY)],
    [JGN(algo.JointType.REVOLUTE, 1), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 4), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY)],
    [JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 2), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 5), JGN(algo.JointType.REVOLUTE, 7)],
    [JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 4), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 8), JGN(algo.JointType.REVOLUTE, 6)],
    [JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 3), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 5), JGN(algo.JointType.REVOLUTE, 8), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY)],
    [JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 7), JGN(algo.JointType.REVOLUTE, 6), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY)]
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
    [JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 0), JGN(algo.JointType.REVOLUTE, 1), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY)],
    [JGN(algo.JointType.REVOLUTE, 0), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 8), JGN(algo.JointType.REVOLUTE, 7)],
    [JGN(algo.JointType.REVOLUTE, 1), JGN(algo.JointType.EMPTY), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 4), JGN(algo.JointType.EMPTY)],
    [JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 8), JGN(algo.JointType.REVOLUTE, 4), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 6)],
    [JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 7), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 6), JGN(algo.JointType.EMPTY)]
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
    [JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 0), JGN(algo.JointType.REVOLUTE, 1)],
    [JGN(algo.JointType.REVOLUTE, 0), JGN(algo.JointType.EMPTY), JGN(algo.JointType.REVOLUTE, 4)],
    [JGN(algo.JointType.REVOLUTE, 1), JGN(algo.JointType.REVOLUTE, 4), JGN(algo.JointType.EMPTY), ]
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


class GraphOperationsTests(unittest.TestCase):
    def test_find_isomorphism_identity(self) -> None:
        """
        Try to identify each registered graph when simply converted as user graph
        """
        for g in graph_data.Graphs:
            graph = data_graph_to_user_graph(g.adjacency)
            iso = algo.find_isomorphism(graph)
            self.assertFalse(iso is None, f'{g}')
            graph_index, isomorphism = iso
            self.assertEqual(graph_index, g)

    def test_find_isomorphism_shuffled(self) -> None:
        """
        Try to identify each registered dyad and triad when shuffled by any isomorphism
        """
        graph_isomorphisms: tuple[algo.Isomorphism, ...] = 3 * (tuple(make_isomorphisms(3)),) + 10 * (tuple(make_isomorphisms(5)),)

        for g, iso_group in zip(graph_data.Graphs, graph_isomorphisms):
            graph = data_graph_to_user_graph(g.adjacency)
            for target_iso in iso_group:
                _graph = apply_isomorphism(graph, target_iso)
                iso = algo.find_isomorphism(_graph)
                self.assertFalse(iso is None, f'{g} : {target_iso}')
                graph_index, isomorphism = iso
                # isomorphism cannot be compared to target_iso since some nodes can be interchanged for some graph e.g.: in RRR every node is equivalent to the other
                self.assertEqual(graph_index, g)

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
        self.assertEqual(merged_graph, [[JGN(algo.JointType.EMPTY)]])

    def test_bad_configurations(self):
        strategy = []

        conf = Config()
        conf.add_solids(np.zeros([2, 4]))
        conf.add_joints(
            np.array([
                [graph_data.R.value, 0, 1],
                [graph_data.R.value, 0, 2],
            ]),
            np.zeros((2, 4))
        )

        # not enough constraints after inputs
        self.assertRaises(kp_exs.SystemConfigurationError, algo.determine_computation_order, conf, conf.piloted_joints, strategy)

        # input on solved joint
        conf.add_joints(
            np.array([
                [graph_data.R.value, 1, 2],
            ]),
            np.zeros((1, 4))
        )
        self.assertRaises(kp_exs.SystemConfigurationError, algo.determine_computation_order, conf, np.array([0]), strategy)

        # 1 solid too many
        conf.add_solids(np.zeros([1, 4]))
        self.assertRaises(kp_exs.SystemConfigurationError, algo.determine_computation_order, conf, conf.piloted_joints, strategy)

    def test_std_graphs(self) -> None:
        strategy = []

        conf = Config()
        conf.add_solids(np.zeros([2, 4]))
        conf.add_joints(
            np.array([
                [graph_data.R.value, 0, 1],
                [graph_data.R.value, 0, 2],
                [graph_data.R.value, 1, 2],
            ]),
            np.zeros((3, 4))
        )

        algo.determine_computation_order(conf, conf.piloted_joints, strategy)
        self.assertEqual(len(strategy), 1)
        self.assertTrue(isinstance(strategy[0], algo.GraphStep))

        step: algo.GraphStep = strategy[0]
        self.assertEqual(graph_data.Graphs(step._graph_index), graph_data.Graphs.gRRR)

        conf = Config()
        conf.add_solids(np.zeros([2, 4]))
        conf.add_joints(
            np.array([
                [graph_data.R.value, 0, 1],
                [graph_data.R.value, 0, 2],
                [graph_data.P.value, 1, 2],
            ]),
            np.zeros((3, 4))
        )

        algo.determine_computation_order(conf, conf.piloted_joints, strategy)
        self.assertEqual(len(strategy), 1)
        self.assertTrue(isinstance(strategy[0], algo.GraphStep))

        step: algo.GraphStep = strategy[0]
        self.assertEqual(graph_data.Graphs(step._graph_index), graph_data.Graphs.gRRP)

        conf = Config()
        conf.add_solids(np.zeros([2, 4]))
        conf.add_joints(
            np.array([
                [graph_data.P.value, 0, 1],
                [graph_data.P.value, 0, 2],
                [graph_data.R.value, 1, 2],
            ]),
            np.zeros((3, 4))
        )

        algo.determine_computation_order(conf, conf.piloted_joints, strategy)
        self.assertEqual(len(strategy), 1)
        self.assertTrue(isinstance(strategy[0], algo.GraphStep))

        step: algo.GraphStep = strategy[0]
        self.assertEqual(graph_data.Graphs(step._graph_index), graph_data.Graphs.gPPR)


if __name__ == '__main__':
    unittest.main()
