from typing import TypeAlias, Self
from collections.abc import Generator, Callable
from kinepy.strategy.graph_data import JointType, Graphs
from kinepy.units import _PhysicsEnum
import numpy as np
import kinepy.math.kinematics as kin
from kinepy.objects.config import Config


# region Strategy Internal types

class JointFlags:
    SOLVED_BIT = 1 << 0

    # joint value is certified to be computed for joints that are solved: -by inputs; -by relations.
    # relations may have to compute the joint values for those that are not available yet, otherwise these computations are not necessary and will depend on user queries
    COMPUTED_BIT = 1 << 1

    # joint value is certified to be continuous for all prismatic joints, and for revolute joints that are solved: -before inputs; -by inputs; -by relations.
    # when driven by a revolute joint, relations may have to compute the continuous version of its angle if not already available
    CONTINUOUS_BIT = 1 << 2

    READY_FOR_USER = SOLVED_BIT | COMPUTED_BIT | CONTINUOUS_BIT


class JointGraphNode:

    node_type: JointType
    joint_index: int = -1

    def __init__(self, joint_type: JointType, joint_index: int = -1):
        self.node_type = joint_type
        self.joint_index = joint_index

    def set(self, joint_index: int, joint_type: JointType):
        self.node_type = joint_type
        self.joint_index = joint_index

    def __repr__(self):
        return self.node_type.__repr__()

    def __eq__(self, other: Self):
        return self.node_type == other.node_type and self.joint_index == other.joint_index


class RelationGraphNode:
    is_1_to_2: bool
    relation: int
    solved = False
    pair: None | Self = None
    common_eq = -1

    def __init__(self, is_1_to_2: bool, relation: int):
        self.is_1_to_2, self.relation = is_1_to_2, relation


JointGraph: TypeAlias = list[list[JointGraphNode]]
RelationGraph: TypeAlias = list[list[RelationGraphNode]]
Degrees: TypeAlias = tuple[tuple[int, int], ...]
Eq: TypeAlias = tuple[tuple[int, ...], ...]
EqMapping: TypeAlias = tuple[int, ...]
Isomorphism: TypeAlias = tuple[int, ...]

# endregion Strategy Internal types


class ResolutionStep:
    def solve_kinematics(self, config: Config):
        pass

    def solve_dynamics(self, config: Config):
        pass


class GraphStep(ResolutionStep):
    def __init__(self, graph: Graphs, edges: tuple[tuple[int, bool], ...], eqs: Eq):
        ResolutionStep.__init__(self)
        self.solution_index = 0
        self._graph_index = graph.value
        self._edges = edges
        self._eqs = eqs

    kinematics = (
        kin.Graph.solve_rrr,
        kin.Graph.solve_rrp,
        kin.Graph.solve_ppr
    )

    def get_joints(self) -> Generator[int, None, None]:
        return (j for j, _ in self._edges)

    def solve_kinematics(self, config: Config):
        self.kinematics[self._graph_index](config, self._edges, self._eqs, self.solution_index)


class JointStep(ResolutionStep):
    function: Callable[[np.ndarray, np.ndarray, int, int, np.ndarray, np.ndarray, tuple[int, ...], tuple[int, ...]], None]

    def __init__(self, joint_type: JointType, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...]):
        ResolutionStep.__init__(self)
        self.joint = joint
        self.eq1, self.eq2 = eq1, eq2
        self.joint_type = joint_type
        self.s1, self.s2 = s1, s2

    function_chooser = {
        JointType.REVOLUTE: kin.JointInput.solve_revolute,
        JointType.PRISMATIC: kin.JointInput.solve_prismatic
    }

    def solve_kinematics(self, config):
        self.function_chooser[self.joint_type](config, self.s1, self.s2, self.joint, self.eq1, self.eq2)


class RelationStep(ResolutionStep):
    relation: int
    is_1_to_2: bool
    eq1: tuple[int]
    eq2: tuple[int]

    def __init__(self, relation: int, is_1_to_2: bool, eq1: tuple[int], eq2: tuple[int]):
        self.relation = relation
        self.is_1_to_2 = is_1_to_2
        self.eq1 = eq1
        self.eq2 = eq2

        self.formula = self.relation_formulae[is_1_to_2]
        self.function = JointStep.function_chooser[(relation.j2 if is_1_to_2 else relation.j1).get_input_physics()]

        self.target = relation.j2 if is_1_to_2 else relation.j1
        self.source = relation.j1 if is_1_to_2 else relation.j2

    relation_formulae = (
        lambda value, r, v0: (value - v0) / r,
        lambda value, r, v0: value * r + v0
    )

    def solve_kinematics(self, solid_values: np.ndarray, joint_values: np.ndarray, kinematic_inputs: np.ndarray):
        value = self.formula(joint_values[self.source._index, ...], self.relation._r, self.relation._v0)
        joint_values[self.target._index, ...] = value

        s1 = self.target.s1._index
        s2 = self.target.s2._index

        self.function(solid_values, value, s1, s2, self.target._p1, self.target._p2, self.eq1, self.eq2)


class JointValueComputationStep(ResolutionStep):
    joint: int
    flags: int
    value_function: Callable[[Config, int, int, int], None]
    continuity_function: Callable[[Config, int], None]

    def __init__(self, joint: int, _type: JointType, flags: int, s1: int, s2: int):
        self.s1, self.s2 = s1, s2
        self.joint = joint
        self.flags = flags

        if self.flags & JointFlags.COMPUTED_BIT:
            self.value_function = kin.JointValueComputation.do_not_compute_value
        else:
            self.value_function = self.value_chooser[_type]

        if self.flags & JointFlags.CONTINUOUS_BIT:
            self.continuity_function = kin.JointValueComputation.do_not_compute_continuity
        else:
            self.continuity_function = self.continuity_chooser[_type]

    value_chooser = {
        JointType.REVOLUTE: kin.JointValueComputation.compute_revolute_value,
        JointType.PRISMATIC: kin.JointValueComputation.compute_prismatic_value
    }

    continuity_chooser = {
        JointType.REVOLUTE: kin.JointValueComputation.compute_revolute_continuity,
        JointType.PRISMATIC: kin.JointValueComputation.do_not_compute_continuity
    }

    def solve_kinematics(self, config: Config):
        self.value_function(config, self.joint, self.s1, self.s2)
        self.continuity_function(config, self.joint)
