from kinepy.objects.joints import Revolute, PrimitiveJoint, Prismatic
from kinepy.objects.relations import Relation
from typing import TypeAlias, Self
from collections.abc import Generator, Callable
from kinepy.strategy.graph_data import NodeType
from kinepy.units import _PhysicsEnum
import numpy as np
import kinepy.math.kinematics as kin

class JointFlags:
    SOLVED_BIT = 1 << 0

    # joint value is certified to be computed for joints that are solved: -by inputs; -by relations.
    # relations may have to compute the joint values for those that are not available yet, otherwise these computations are not necessary and will depend on user queries
    COMPUTED_BIT = 1 << 1

    # joint value is certified to be continuous for all prismatic joints, and for revolute joints that are solved: -before inputs; -by inputs; -by relations.
    # when driven by a revolute joint, relations may have to compute the continuous version of its angle if not already available
    CONTINUOUS_BIT = 1 << 2


class JointGraphNode:

    node_type: NodeType
    joint_index: int = -1

    def __init__(self, joint_type: NodeType, joint_index: int = -1):
        self.node_type = joint_type
        self.joint_index = joint_index

    def set_node_type(self, joint: PrimitiveJoint | None):
        if joint is not None:
            self.joint_index = joint.index
        else:
            self.joint_index = -1
        self.node_type = NodeType.EMPTY if joint is None else NodeType.REVOLUTE if isinstance(joint, Revolute) else NodeType.PRISMATIC

    def __repr__(self):
        return self.node_type.__repr__()

    def __eq__(self, other: Self):
        return self.node_type == other.node_type and self.joint_index == other.joint_index


class RelationGraphNode:
    is_1_to_2: bool
    relation: Relation
    solved = False
    pair: None | Self = None
    common_eq = -1

    def __init__(self, is_1_to_2: bool, relation: Relation):
        self.is_1_to_2, self.relation = is_1_to_2, relation

    def yield_target_joint(self) -> Generator[PrimitiveJoint, None, None]:
        yield self.get_target_joint()

    def get_source_joint(self) -> PrimitiveJoint:
        if self.is_1_to_2:
            return self.relation.j1
        return self.relation.j2

    def get_target_joint(self) -> PrimitiveJoint:
        if self.is_1_to_2:
            return self.relation.j2
        return self.relation.j1


JointGraph: TypeAlias = list[list[JointGraphNode]]
RelationGraph: TypeAlias = list[list[RelationGraphNode]]
Degrees: TypeAlias = tuple[tuple[int, int], ...]
Eq: TypeAlias = tuple[tuple[int, ...], ...]
EqMapping: TypeAlias = tuple[int, ...]
Isomorphism: TypeAlias = tuple[int, ...]


class ResolutionStep:
    def solve_kinematics(self, solid_values: np.ndarray, joint_values: np.ndarray, kinematic_inputs: np.ndarray):
        pass

    def solve_dynamics(self):
        pass


class GraphStep(ResolutionStep):
    def __init__(self, graph_index: int, edges: tuple[tuple[PrimitiveJoint, bool], ...], eqs: Eq):
        ResolutionStep.__init__(self)
        self.solution_index = 0
        self._graph_index = graph_index
        self._edges = edges
        self._eqs = eqs

    def get_joints(self) -> Generator[PrimitiveJoint, None, None]:
        return (j for j, _ in self._edges)


class JointStep(ResolutionStep):
    function: Callable[[np.ndarray, np.ndarray, int, int, np.ndarray, np.ndarray, tuple[int, ...], tuple[int, ...]], None]

    def __init__(self, input_index: int, joint: PrimitiveJoint, eq1: tuple[int, ...], eq2: tuple[int, ...]):
        ResolutionStep.__init__(self)
        self.input_index = input_index
        self.joint = joint
        self.eq1, self.eq2 = eq1, eq2
        self.function = self.function_chooser[joint.get_input_physics()]

    function_chooser = {
        _PhysicsEnum.ANGLE: kin.JointInput.solve_revolute,
        _PhysicsEnum.LENGTH: kin.JointInput.solve_prismatic
    }

    def solve_kinematics(self, solid_values: np.ndarray, joint_values: np.ndarray, kinematic_inputs: np.ndarray):
        value = kinematic_inputs[self.input_index, ...]
        joint_values[self.joint._index, ...] = value

        s1 = self.joint.s1._index
        s2 = self.joint.s2._index

        self.function(solid_values, value, s1, s2, self.joint._p1, self.joint._p2, self.eq1, self.eq2)


class RelationStep(ResolutionStep):
    relation: Relation
    is_1_to_2: bool
    eq1: tuple[int]
    eq2: tuple[int]

    def __init__(self, relation: Relation, is_1_to_2: bool, eq1: tuple[int], eq2: tuple[int]):
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
    joint: PrimitiveJoint
    flags: int
    value_function: Callable[[np.ndarray, int, int, np.ndarray, np.ndarray], np.ndarray]
    continuity_function: Callable[[np.ndarray], None]

    def __init__(self, joint: PrimitiveJoint, flags: int):
        self.joint = joint
        self.flags = flags

        if self.flags & JointFlags.COMPUTED_BIT:
            self.value_function = kin.JointValueComputation.do_not_compute_value
        else:
            self.value_function = self.value_chooser[self.joint.get_input_physics()]

        if self.flags & JointFlags.CONTINUOUS_BIT:
            self.continuity_function = kin.JointValueComputation.do_not_compute_continuity
        else:
            self.continuity_function = self.continuity_chooser[self.joint.get_input_physics()]

    value_chooser = {
        _PhysicsEnum.ANGLE: kin.JointValueComputation.compute_revolute_value,
        _PhysicsEnum.LENGTH: kin.JointValueComputation.compute_prismatic_value
    }

    continuity_chooser = {
        _PhysicsEnum.ANGLE: kin.JointValueComputation.compute_revolute_value,
        _PhysicsEnum.LENGTH: kin.JointValueComputation.do_not_compute_continuity
    }

    def solve_kinematics(self, solid_values: np.ndarray, joint_values: np.ndarray, kinematic_inputs: np.ndarray):
        joint_values[self.joint._index, ...] = self.value_function(solid_values, self.joint.s1, self.joint.s2, self.joint._p1, self.joint._p2)
        self.continuity_function(joint_values[self.joint._index, ...])
