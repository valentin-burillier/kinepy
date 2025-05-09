from typing import TypeAlias, Self
from collections.abc import Generator, Callable
from kinepy.strategy.graph_data import JointType, Graphs, RelationType
import kinepy.math.kinematics as kin
import kinepy.math.dynamics as dyn
from kinepy.objects.config import OldConfig


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
    def solve_kinematics(self, config: OldConfig):
        pass

    def solve_dynamics(self, config: OldConfig):
        pass


class GraphStep(ResolutionStep):
    def __init__(self, graph: Graphs, edges: tuple[tuple[int, bool], ...], eqs: Eq):
        ResolutionStep.__init__(self)
        self.solution_index = 0
        self._graph_index = graph
        self._edges = edges
        self._eqs = eqs
        self._zero_holder = 0
        for i, eq in enumerate(eqs):
            if 0 in eq:
                self._zero_holder = i

    kinematics = (
        kin.Graph.solve_rrr,
        kin.Graph.solve_rrp,
        kin.Graph.solve_ppr
    )

    dynamics = (
        dyn.Graph.solve_rrr,
        dyn.Graph.solve_rrp,
        dyn.Graph.solve_ppr
    )

    @property
    def solution_count(self) -> int:
        return self._graph_index.solutions

    def get_joints(self) -> Generator[int, None, None]:
        return (j for j, _ in self._edges)

    def solve_kinematics(self, config: OldConfig):
        self.kinematics[self._graph_index.value](config, self._edges, self._eqs, self.solution_index)

    def solve_dynamics(self, config: OldConfig):
        self.dynamics[self._graph_index.value](config, self._edges, self._eqs, self._zero_holder)


class JointStep(ResolutionStep):

    def __init__(self, joint_type: JointType, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...]):
        ResolutionStep.__init__(self)
        self.joint = joint
        self.eq1, self.eq2 = eq1, eq2
        self.joint_type = joint_type.simple()
        self.s1, self.s2 = s1, s2
        self.zero_holder = 0 in eq2

    kinematics_chooser = {
        JointType.REVOLUTE: kin.JointInput.solve_revolute,
        JointType.PRISMATIC: kin.JointInput.solve_prismatic
    }

    dynamics_chooser = {
        JointType.REVOLUTE: dyn.JointInput.solve_revolute,
        JointType.PRISMATIC: dyn.JointInput.solve_prismatic
    }

    def solve_kinematics(self, config: OldConfig):
        self.kinematics_chooser[self.joint_type](config, self.s1, self.s2, self.joint, self.eq1, self.eq2)

    def solve_dynamics(self, config: OldConfig):
        self.dynamics_chooser[self.joint_type](config, self.s1, self.s2, self.joint, self.eq1, self.eq2, self.zero_holder)


class RelationStep(ResolutionStep):
    relation: int
    is_1_to_2: bool
    eq1: tuple[int]
    eq2: tuple[int]

    def __init__(self, relation: int, rtype: int, is_1_to_2: bool, source: int, target: int, target_type: int, eq1: tuple[int], eq2: tuple[int]):
        self.relation = relation
        self.relation_type = RelationType(rtype)
        self.target_type = JointType(target_type).simple().value
        self.is_1_to_2 = is_1_to_2
        self.source = source
        self.target = target
        self.eq1 = eq1
        self.eq2 = eq2
        self.zero_holder = 0 in eq2

    kinematics_chooser = {
        RelationType.GEAR_RACK: kin.Relation.solve_standard_relation,
        RelationType.GEAR: kin.Relation.solve_standard_relation,
        RelationType.DISTANT: kin.Relation.solve_standard_relation,
        RelationType.EFFORTLESS: kin.Relation.solve_standard_relation,
        RelationType.BELT: kin.Relation.solve_belt,
    }

    def solve_kinematics(self, config: OldConfig):
        self.kinematics_chooser[self.relation_type](config, self.relation, self.source, self.target, self.target_type, self.eq1, self.eq2, self.is_1_to_2)

    dynamics_chooser = {
        RelationType.EFFORTLESS: dyn.Relation.solve_effortless_relation,
        RelationType.DISTANT: dyn.Relation.solve_distant_relation,
        RelationType.GEAR: dyn.Relation.solve_gear_pair,
        RelationType.GEAR_RACK: dyn.Relation.solve_gear_rack,
        RelationType.BELT: dyn.Relation.solve_belt
    }

    def solve_dynamics(self, config: OldConfig):
        self.dynamics_chooser[self.relation_type](config, self.relation, self.source, self.target, self.target_type, self.eq1, self.eq2, self.is_1_to_2, self.zero_holder)


class JointValueComputationStep(ResolutionStep):
    joint: int
    flags: int
    value_function: Callable[[OldConfig, int, int, int], None]
    continuity_function: Callable[[OldConfig, int], None]

    def __init__(self, joint: int, _type: JointType, flags: int, s1: int, s2: int):
        _type = _type.simple()
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

    def solve_kinematics(self, config: OldConfig):
        self.value_function(config, self.joint, self.s1, self.s2)
        self.continuity_function(config, self.joint)

    def solve_dynamics(self, config: OldConfig):
        """Nothing to do"""
