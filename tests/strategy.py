import unittest
import numpy as np

import kinepy.objects.config as cfg
import kinepy.strategy.types as types
import kinepy.strategy.graph_data as gd
import kinepy.strategy.algorithm as algo
import kinepy.exceptions as ex


def data_graph_to_user_graph(adj: gd.Adjacency) -> types.JointGraph:
    result_graph: types.JointGraph = [[types.JointGraphNode(v) for v in line] for line in adj]
    return result_graph


def apply_isomorphism(graph: types.JointGraph, iso: types.Isomorphism) -> types.JointGraph:
    result = [[types.JointGraphNode(cfg.Joints.Type.EMPTY) for _ in graph] for _ in graph]

    for x, line in enumerate(graph):
        new_x = iso[x]
        for y, node in enumerate(line):
            new_y = iso[y]
            result[new_x][new_y] = node
    return result


E, P, R = cfg.Joints.Type.EMPTY, cfg.Joints.Type.PRISMATIC, cfg.Joints.Type.REVOLUTE

_example_graph = [
    [(E, -1), (R, 0), (R, 1), (E, -1), (E, -1), (E, -1), (E, -1)],
    [(R, 0), (E, -1), (E, -1), (R, 2), (E, -1), (R, 3), (E, -1)],
    [(R, 1), (E, -1), (E, -1), (E, -1), (R, 4), (E, -1), (E, -1)],
    [(E, -1), (R, 2), (E, -1), (E, -1), (E, -1), (R, 5), (R, 7)],
    [(E, -1), (E, -1), (R, 4), (E, -1), (E, -1), (R, 8), (R, 6)],
    [(E, -1), (R, 3), (E, -1), (R, 5), (R, 8), (E, -1), (E, -1)],
    [(E, -1), (E, -1), (E, -1), (R, 7), (R, 6), (E, -1), (E, -1)]
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
example_graph: types.JointGraph = [[types.JointGraphNode(T, I) for T, I in line] for line in _example_graph]

_example_graph_merge0 = [
    [(E, -1), (R, 0), (R, 1), (E, -1), (E, -1)],
    [(R, 0), (E, -1), (E, -1), (R, 8), (R, 7)],
    [(R, 1), (E, -1), (E, -1), (R, 4), (E, -1)],
    [(E, -1), (R, 8), (R, 4), (E, -1), (R, 6)],
    [(E, -1), (R, 7), (E, -1), (R, 6), (E, -1)]
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
example_graph_merge0: types.JointGraph = [[types.JointGraphNode(T, I) for T, I in line] for line in _example_graph_merge0]
example_eqs_merge0: types.Eq = (0,), (1, 5, 3), (2,), (4,), (6,)
example_solid_to_eq_merge0: types.EqMapping = 0, 1, 2, 1, 3, 1, 4


_example_graph_merge1 = [
    [(E, -1), (R, 0), (R, 1)],
    [(R, 0), (E, -1), (R, 4)],
    [(R, 1), (R, 4), (E, -1), ]
]
r"""
        0
       / \
      R0  R1
     /     \
    1 ----- 2
        R4
"""
example_graph_merge1: types.JointGraph = [[types.JointGraphNode(T, I) for T, I in line] for line in _example_graph_merge1]
example_eqs_merge1: types.Eq = (0,), (1, 5, 3, 4, 6), (2,)
example_solid_to_eq_merge1: types.EqMapping = 0, 1, 2, 1, 1, 1, 1


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
        for g in gd.Graphs:
            graph = data_graph_to_user_graph(g.adjacency)
            iso = algo.find_isomorphism(graph)
            self.assertFalse(iso is None, f'{g}')
            graph_index, isomorphism = iso
            self.assertEqual(graph_index, g)

    def test_find_isomorphism_shuffled(self) -> None:
        """
        Try to identify each registered dyad and triad when shuffled by any isomorphism
        """
        graph_isomorphisms: tuple[types.Isomorphism, ...] = 3 * (tuple(make_isomorphisms(3)),) + 10 * (tuple(make_isomorphisms(5)),)

        for g, iso_group in zip(gd.Graphs, graph_isomorphisms):
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
        self.assertEqual(merged_graph, [[types.JointGraphNode(E, -1)]])

    def test_bad_configurations(self):
        strategy = []

        conf = cfg.Config()
        conf.solids.reserve(2)
        joints = conf.joints.reserve(2)
        conf.joints.type_[joints] = R, R
        conf.joints.solids[joints] = (
            (0, 1),
            (0, 2)
        )

        # not enough constraints after inputs
        
        piloted_joints = np.arange(conf.joints.count)[(conf.joints.state & 1) == 1]
        self.assertRaises(ex.SystemConfigurationError, algo.determine_computation_order, conf, piloted_joints, strategy)

        # input on solved joint
        joints = conf.joints.reserve(1)
        conf.joints.type_[joints] = R
        conf.joints.solids[joints] = (
            (1, 2),
        )
        self.assertRaises(ex.SystemConfigurationError, algo.determine_computation_order, conf, np.array([0]), strategy)

        # 1 solid too many
        conf.solids.reserve(1)
        piloted_joints = np.arange(conf.joints.count)[(conf.joints.state & 1) == 1]
        self.assertRaises(ex.SystemConfigurationError, algo.determine_computation_order, conf, piloted_joints, strategy)

    def test_std_graphs(self) -> None:
        strategy = []

        conf = cfg.Config()
        conf.solids.reserve(2)
        joints = conf.joints.reserve(3)
        conf.joints.type_[joints] = R, R, R
        conf.joints.solids[joints] = (
            (0, 1),
            (0, 2),
            (1, 2)
        )
        piloted_joints = np.arange(conf.joints.count)[(conf.joints.state & 1) == 1]
        algo.determine_computation_order(conf, piloted_joints, strategy)
        conf.declarations.append((gd.Graphs.gRRR, 1, 2, 0))
        algo.apply_declarations(conf)
        self.assertEqual(len(strategy), 1)
        self.assertTrue(isinstance(strategy[0], types.GraphStep))

        step: types.GraphStep = strategy[0]
        self.assertEqual(gd.Graphs(step.graph_index), gd.Graphs.gRRR)

        conf = cfg.Config()
        conf.solids.reserve(2)
        joints = conf.joints.reserve(3)
        conf.joints.type_[joints] = R, R, P
        conf.joints.solids[joints] = (
            (0, 1),
            (0, 2),
            (1, 2)
        )
        
        piloted_joints = np.arange(conf.joints.count)[(conf.joints.state & 1) == 1]
        algo.determine_computation_order(conf, piloted_joints, strategy)
        conf.declarations.append((gd.Graphs.gRRP, 2))
        algo.apply_declarations(conf)

        self.assertEqual(len(strategy), 1)
        self.assertTrue(isinstance(strategy[0], types.GraphStep))

        step: types.GraphStep = strategy[0]
        self.assertEqual(gd.Graphs(step.graph_index), gd.Graphs.gRRP)

        conf = cfg.Config()
        conf.solids.reserve(2)
        joints = conf.joints.reserve(3)
        conf.joints.type_[joints] = P, P, R
        conf.joints.solids[joints] = (
            (0, 1),
            (0, 2),
            (1, 2)
        )
        piloted_joints = np.arange(conf.joints.count)[(conf.joints.state & 1) == 1]
        algo.determine_computation_order(conf, piloted_joints, strategy)
        self.assertEqual(len(strategy), 1)
        self.assertTrue(isinstance(strategy[0], types.GraphStep))

        step: types.GraphStep = strategy[0]
        self.assertEqual(gd.Graphs(step.graph_index), gd.Graphs.gPPR)


if __name__ == '__main__':
    unittest.main()
