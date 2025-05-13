import kinepy.objects.config as cfg
import kinepy.strategy.graph_data as gd
import kinepy.math.kinematics as kin
import kinepy.math.dynamics as dyn

import typing
import collections.abc

# region Strategy Internal types


class JointFlags:
    SOLVED_BIT = 1 << 0

    # joint value is certified to be computed for joints that are solved: -by inputs; -by relations.
    # relations may have to compute the joint values for those that are not available yet, otherwise these computations are not necessary and will depend on user queries
    COMPUTED_BIT = 1 << 1

    # joint value is certified to be continuous for all prismatic joints, and for revolute joints that are solved: -before inputs; -by inputs; -by relations.
    # when driven by a revolute joint, relations may have to compute the continuous version of its angle if not already available
    CONTINUOUS_BIT = 1 << 2

    RELATION_READY = CONTINUOUS_BIT | COMPUTED_BIT
    READY_FOR_USER = SOLVED_BIT | COMPUTED_BIT | CONTINUOUS_BIT

    @classmethod
    def relation_ready(cls, value: int) -> bool:
        return bool(value & cls.COMPUTED_BIT and value & cls.CONTINUOUS_BIT)

    @classmethod
    def solved_joint(cls, type_: int, value_is_computed: bool, certain_continuity: bool):
        return cls.SOLVED_BIT | (certain_continuity or cfg.Joints.Type(type_).primitive() == cfg.Joints.Type.PRISMATIC) * cls.CONTINUOUS_BIT | value_is_computed * cls.COMPUTED_BIT


class JointGraphNode:

    node_type: cfg.Joints.Type
    joint_index: int = -1

    def __init__(self, joint_type: cfg.Joints.Type, joint_index: int = -1):
        self.node_type = joint_type
        self.joint_index = joint_index

    def set(self, joint_index: int, joint_type: cfg.Joints.Type):
        self.node_type = joint_type
        self.joint_index = joint_index

    def __repr__(self):
        return self.node_type.__repr__()

    def __eq__(self, other: typing.Self):
        return self.node_type == other.node_type and self.joint_index == other.joint_index


class RelationGraphNode:
    is_1_to_2: bool
    relation: int
    solved = False
    pair: None | typing.Self = None
    common_eq = -1

    def __init__(self, is_1_to_2: bool, relation: int):
        self.is_1_to_2, self.relation = is_1_to_2, relation


type JointGraph = list[list[JointGraphNode]]
type RelationGraph = list[list[RelationGraphNode]]
type Degrees = tuple[tuple[int, int], ...]
type Eq = tuple[tuple[int, ...], ...]
type EqMapping = tuple[int, ...]
type Isomorphism = tuple[int, ...]
type StrategyJointState = tuple[JointGraph, Eq, EqMapping]

# endregion Strategy Internal types


class ResolutionStep:
    def solve_kinematics(self, config: cfg.Config):
        pass

    def solve_dynamics(self, config: cfg.Config):
        pass


class GraphStep(ResolutionStep):
    def __init__(self, graph: gd.Graphs, edges: tuple[tuple[int, bool], ...], eqs: Eq):
        ResolutionStep.__init__(self)
        self.graph_index = graph

        self.solution_index = 0

        self.edges = edges
        self._eqs = eqs
        self._zero_holder = 0
        for i, eq in enumerate(eqs):
            if 0 in eq:
                self._zero_holder = i
            
        self.__default_choice()

    def __default_choice(self):
        if self.graph_index == gd.Graphs.gRRP:
            self.solution_index = self.edges[2][1]


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
        return self.graph_index.solutions

    def get_joints(self) -> collections.abc.Iterable[int]:
        return (j for j, _ in self.edges)

    def solve_kinematics(self, config: cfg.Config):
        self.kinematics[self.graph_index.value](config, self.edges, self._eqs, self.solution_index)

    def solve_dynamics(self, config: cfg.Config):
        self.dynamics[self.graph_index.value](config, self.edges, self._eqs, self._zero_holder)

    def __match_rrr(self, joints: list[int, ...]):
        return all(map(lambda x: x[0] in joints, self.edges))

    def __match_rrp(self, joints: list[int, ...]):
        return joints[0] == self.edges[2][0]

    __matchers = {
        gd.Graphs.gRRR: __match_rrr,
        gd.Graphs.gRRP: __match_rrp
    }

    def match(self, graph: gd.Graphs, joints: list[int, ...]) -> bool:
        if graph != self.graph_index:
            return False
        return self.__matchers[graph](self, joints)

    def __apply_rrr(self, joints):
        j0 = self.edges[0][0]

        i = 0
        while i < 3 and j0 != joints[i]:
            i += 1
        assert i < 3, "Declaration matched but j0 is not in joints"

        _joints = joints[i:] + joints[:i]
        _my_joints = list(map(lambda x: x[0], self.edges))

        self.solution_index = _joints != _my_joints

    def __apply_rrp(self, joints):
        self.solution_index = not self.edges[2][1]

    __apply_declaration = {
        gd.Graphs.gRRR: __apply_rrr,
        gd.Graphs.gRRP: __apply_rrp
    }

    def apply_declaration(self, joints: list[int, ...]):
        return self.__apply_declaration[self.graph_index](self, joints)



class JointStep(ResolutionStep):

    def __init__(self, joint_type: cfg.Joints.Type, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...]):
        ResolutionStep.__init__(self)
        self.joint = joint
        self.eq1, self.eq2 = eq1, eq2
        self.joint_type = joint_type.primitive()
        self.s1, self.s2 = s1, s2
        self.zero_holder = 0 in eq2

    kinematics_chooser = {
        cfg.Joints.Type.REVOLUTE: kin.JointInput.solve_revolute,
        cfg.Joints.Type.PRISMATIC: kin.JointInput.solve_prismatic
    }

    dynamics_chooser = {
        cfg.Joints.Type.REVOLUTE: dyn.JointInput.solve_revolute,
        cfg.Joints.Type.PRISMATIC: dyn.JointInput.solve_prismatic
    }

    def solve_kinematics(self, config: cfg.Config):
        self.kinematics_chooser[self.joint_type](config, self.s1, self.s2, self.joint, self.eq1, self.eq2)

    def solve_dynamics(self, config: cfg.Config):
        self.dynamics_chooser[self.joint_type](config, self.s1, self.s2, self.joint, self.eq1, self.eq2, self.zero_holder)


class RelationStep(ResolutionStep):
    relation: int
    is_1_to_2: bool
    eq1: tuple[int]
    eq2: tuple[int]

    def __init__(self, relation: int, rtype: int, is_1_to_2: bool, source: int, target: int, target_type: int, eq1: tuple[int], eq2: tuple[int]):
        self.relation = relation
        self.relation_type = cfg.Relations.Type(rtype)
        self.target_type = cfg.Joints.Type(target_type).primitive()
        self.is_1_to_2 = is_1_to_2
        self.source = source
        self.target = target
        self.eq1 = eq1
        self.eq2 = eq2
        self.zero_holder = 0 in eq2

    kinematics_chooser = {
        cfg.Relations.Type.GEAR_RACK: kin.Relation.solve_standard_relation,
        cfg.Relations.Type.GEAR_PAIR: kin.Relation.solve_standard_relation,
        cfg.Relations.Type.DISTANT: kin.Relation.solve_standard_relation,
        cfg.Relations.Type.EFFORTLESS: kin.Relation.solve_standard_relation,
        cfg.Relations.Type.BELT: kin.Relation.solve_belt,
    }

    def solve_kinematics(self, config: cfg.Config):
        self.kinematics_chooser[self.relation_type](config, self.relation, self.source, self.target, self.target_type, self.eq1, self.eq2, self.is_1_to_2)

    dynamics_chooser = {
        cfg.Relations.Type.EFFORTLESS: dyn.Relation.solve_effortless_relation,
        cfg.Relations.Type.DISTANT: dyn.Relation.solve_distant_relation,
        cfg.Relations.Type.GEAR_PAIR: dyn.Relation.solve_gear_pair,
        cfg.Relations.Type.GEAR_RACK: dyn.Relation.solve_gear_rack,
        cfg.Relations.Type.BELT: dyn.Relation.solve_belt
    }

    def solve_dynamics(self, config: cfg.Config):
        self.dynamics_chooser[self.relation_type](config, self.relation, self.source, self.target, self.target_type, self.eq1, self.eq2, self.is_1_to_2, self.zero_holder)


class JointValueComputationStep(ResolutionStep):
    joint: int
    flags: int

    def __init__(self, joint: int, _type: cfg.Joints.Type, flags: int, s1: int, s2: int):
        _type = _type.primitive()
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
        cfg.Joints.Type.REVOLUTE: kin.JointValueComputation.compute_revolute_value,
        cfg.Joints.Type.PRISMATIC: kin.JointValueComputation.compute_prismatic_value
    }

    continuity_chooser = {
        cfg.Joints.Type.REVOLUTE: kin.JointValueComputation.compute_revolute_continuity,
        cfg.Joints.Type.PRISMATIC: kin.JointValueComputation.do_not_compute_continuity
    }

    def solve_kinematics(self, config: cfg.Config):
        self.value_function(config, self.joint, self.s1, self.s2)
        self.continuity_function(config, self.joint)

    def solve_dynamics(self, config: cfg.Config):
        """Nothing to do"""
