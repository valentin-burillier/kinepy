from kinepy.objects.joints import Revolute, PrimitiveJoint, Prismatic
from kinepy.objects.relations import Relation
from typing import TypeAlias, Self, Generator
from kinepy.strategy.graph_data import NodeType





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
    def solve_kinematics(self):
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
    def __init__(self, input_index: int, joint: PrimitiveJoint, eq1: tuple[int, ...], eq2: tuple[int, ...]):
        ResolutionStep.__init__(self)
        self._input_index = input_index
        self._joint = joint
        self._eq1, self._eq2 = eq1, eq2


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


class JointValueComputationStep(ResolutionStep):
    joint: PrimitiveJoint
    flags: int

    def __init__(self, joint: PrimitiveJoint, flags: int):
        self.joint = joint
        self.flags = flags


class JointFlags:
    SOLVED_BIT = 1 << 0

    # joint value is certified to be computed for joints that are solved: -by inputs; -by relations.
    # relations may have to compute the joint values for those that are not available yet, otherwise these computations are not necessary and will depend on user queries
    COMPUTED_BIT = 1 << 1

    # joint value is certified to be continuous for all prismatic joints, and for revolute joints that are solved: -before inputs; -by inputs; -by relations.
    # when driven by a revolute joint, relations may have to compute the continuous version of its angle if not already available
    CONTINUOUS_BIT = 1 << 2
