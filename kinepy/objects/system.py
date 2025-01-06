from kinepy.objects.solid import Solid
from kinepy.objects.relations import Relation, Gear, GearRack, DistantRelation, EffortlessRelation
from kinepy.objects.joints import Joint, Revolute, Prismatic, GhostHolder, PinSlot, Translation, ThreeDOF, PrimitiveJoint
import kinepy.strategy as strategy
from kinepy.units import Physics
import numpy as np


@Physics.class_
class System:
    def __init__(self):
        self._solids: list[Solid] = [Solid(self, 'Ground', 0)]
        self._joints: list[Joint] = []
        self._relations: list[Relation] = []
        self._blocked = []
        self._piloted = []

        self._dynamic_strategy = []
        self._kinematic_strategy = []

    def get_ground(self) -> Solid:
        return self._solids[0]

    def _check_solids(self, *solids: Solid, kw_solids=()):
        for solid in solids + kw_solids:
            if solid.system is not self or solid.index >= len(self._solids) or self._solids[solid.index] is not solid:
                raise ValueError(f"Solid: {solid.name} does not belong to this system")
        return True

    def _check_joints(self, *joints: Joint, kw_joints=()):
        for joint in joints + kw_joints:
            if joint.system is not self or (not isinstance(joint, GhostHolder) and (joint.index >= len(self._joints) or self._joints[joint.index] is not joint)):
                raise ValueError(f"Joint {joint} does not belong to this system")
        return True

    def add_solid(self, name='', mass: Physics.MASS = 0., moment_of_inertia: Physics.MOMENT_OF_INERTIA = 0., g: Physics.POINT = (0., 0.)) -> Solid:
        name = name or f'Solid {len(self._solids)}'
        self._solids.append(Solid(self, name, len(self._solids), mass, moment_of_inertia, g))
        return self._solids[-1]

    def add_prismatic(self, s1: Solid, s2: Solid, alpha1: Physics.ANGLE = 0.0, distance1: Physics.LENGTH = 0.0, alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> Prismatic:
        self._check_solids(s1, s2)
        result = Prismatic(self, len(self._joints), s1, s2, alpha1, distance1, alpha2, distance2)
        self._joints.append(result)
        return result

    def add_revolute(self, s1: Solid, s2: Solid, p1: Physics.POINT = (0.0, 0.0), p2: Physics.POINT = (0.0, 0.0)) -> Revolute:
        self._check_solids(s1, s2)
        result = Revolute(self, len(self._joints), s1, s2, p1, p2)
        self._joints.append(result)
        return result

    def add_pin_slot(self, s1: Solid, s2: Solid, p1: Physics.POINT = (0.0, 0.0), alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> PinSlot:
        self._check_solids(s1, s2)
        ghost_solid = Solid(self, f'GhostSolid {len(self._solids)}', len(self._solids))
        self._solids.append(ghost_solid)
        ghost_joints = (
            Revolute(self, len(self._joints), s1, ghost_solid, p1),
            Prismatic(self, len(self._joints)+1, ghost_solid, s2, alpha2, 0.0, alpha2, distance2)
        )
        self._joints.extend(ghost_joints)
        return PinSlot(self, -1, ghost_solid, ghost_joints, s1, s2)

    def add_translation(self, s1: Solid, s2: Solid, alpha1: Physics.ANGLE = 0.0, distance1: Physics.LENGTH = 0.0, alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> Translation:
        self._check_solids(s1, s2)
        ghost_solid = Solid(f'GhostSolid {len(self._solids)}', len(self._solids))
        self._solids.append(ghost_solid)
        ghost_joints = (
            Prismatic(self, len(self._joints), s1, ghost_solid, alpha1, distance1, alpha1, 0.0),
            Prismatic(self, len(self._joints)+1, ghost_solid, s2, alpha2, 0.0, alpha2, distance2)
        )
        self._joints.extend(ghost_joints)
        return Translation(self, -1, ghost_solid, ghost_joints, s1, s2)

    def add_3dof_to_ground(self, s: Solid, p: Physics.POINT = (0., 0.)) -> ThreeDOF:
        self._check_solids(s)
        ghost_solids = (
            Solid(self, f'GhostSolid {len(self._solids)}', len(self._solids)),
            Solid(self, f'GhostSolid {len(self._solids)+1}', len(self._solids)+1)
        )
        self._solids.extend(ghost_solids)
        ghost_joints = (
            Prismatic(self, len(self._joints), self.get_ground(), ghost_solids[0]),
            Prismatic(self, len(self._joints)+1, ghost_solids[0], ghost_solids[1], alpha1=np.pi * 0.5, alpha2=np.pi * 0.5),
            Revolute(self, len(self._joints)+2, ghost_solids[1], s, p2=p)
        )
        self._joints.extend(ghost_joints)
        return ThreeDOF(self, -1, ghost_solids, ghost_joints, self.get_ground(), s)

    def add_gear(self, j1: Revolute, j2: Revolute, r: Physics.DIMENSIONLESS = 0.0, v0: Physics.ANGLE = 0.0) -> Gear:
        self._check_joints(j1, j2)
        self._relations.append(g := Gear(self, len(self._relations), j1, j2, r, v0))
        return g

    def add_gear_rack(self, j1: Revolute, j2: Prismatic, r: Physics.LENGTH = 0.0, v0: Physics.LENGTH = 0.0) -> GearRack:
        self._check_joints(j1, j2)
        self._relations.append(g := GearRack(self, len(self._relations), j1, j2, r, v0))
        return g

    def add_distant_relation(self, j1: PrimitiveJoint, j2: PrimitiveJoint, r: Physics.scalar_type = 0.0, v0: Physics.scalar_type = 0.0) -> DistantRelation:
        self._check_joints(j1, j2)
        rel = DistantRelation(self, len(self._relations), j1, j2)
        rel.r, rel.v0 = r, v0
        self._relations.append(rel)
        return rel

    def add_effortless_relation(self, j1: PrimitiveJoint, j2: PrimitiveJoint, r: Physics.scalar_type = 0.0, v0: Physics.scalar_type = 0.0) -> EffortlessRelation:
        self._check_joints(j1, j2)
        rel = EffortlessRelation(self, len(self._relations), j1, j2)
        rel.r, rel.v0 = r, v0
        self._relations.append(rel)
        return rel

    def pilot(self, *joints: Joint):
        self._check_joints(kw_joints=joints)
        self._piloted.extend(joints)

    def block(self, *joints: Joint):
        self._check_joints(kw_joints=joints)
        self._blocked.extend(joints)

    def _hyper_statism_value(self, joint_input: list[Joint]) -> int:
        return 2 * len(self._joints) - 3 * (len(self._solids) - 1) + len(joint_input) + len(self._relations)

    def _determine_computation_order(self, input_joints, strategy_output):
        strategy.determine_computation_order(len(self._solids), self._joints, self._relations, input_joints, strategy_output)

    def determine_computation_order(self):
        if self._blocked:
            self._determine_computation_order(self._blocked, self._dynamic_strategy)
        self._determine_computation_order(self._joints, self._kinematic_strategy)

    def solve_kinematics(self):
        pass

    def solve_dynamics(self):
        pass
