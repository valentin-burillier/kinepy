from kinepy.objects.solid import Solid
from kinepy.objects.relations import Relation, GearPair, GearRack, DistantRelation, EffortlessRelation
from kinepy.objects.joints import Joint, Revolute, Prismatic, GhostHolder, PinSlot, Translation, ThreeDOF, PrimitiveJoint
import kinepy.exceptions as ex
import kinepy.strategy as strategy
from kinepy.units import Physics
import numpy as np
from collections.abc import Iterable


@Physics.class_
class System:
    def __init__(self):
        self._solids: list[Solid] = [Solid(self, 'Ground', 0)]
        self._joints: list[PrimitiveJoint] = []
        self._relations: list[Relation] = []
        self._blocked = []
        self._piloted = []

        self._kinematic_inputs: np.ndarray = np.array([], dtype=np.float64)
        self._kinematic_strategy: list[strategy.ResolutionStep] = []

        self._solid_values: np.ndarray = np.array([], dtype=np.float64)
        self._joint_values: np.ndarray = np.array([], dtype=np.float64)

        self._dynamic_strategy: list[strategy.ResolutionStep] = []

    def get_ground(self) -> Solid:
        return self._solids[0]

    def _check_solids(self, *solids: Solid, kw_solids: tuple[Solid, ...] = ()):
        for solid in solids + kw_solids:
            if solid.system is not self or solid.index >= len(self._solids) or self._solids[solid.index] is not solid:
                raise ex.UnrelatedObjectsError(f"Solid \"{solid}\" does not belong to this system")

    def _check_joints(self, *joints: Joint, kw_joints: tuple[Joint, ...] = ()):
        test: int
        for joint in joints + kw_joints:
            if joint.system is not self or (not isinstance(joint, GhostHolder) and (joint.index >= len(self._joints) or self._joints[joint.index] is not joint)):
                raise ex.UnrelatedObjectsError(f"Joint \"{joint}\" does not belong to this system")

    def add_solid(self, name='', mass: Physics.MASS = 0., moment_of_inertia: Physics.MOMENT_OF_INERTIA = 0., g: Physics.POINT = (0., 0.)) -> Solid:
        name = name or f'Solid {len(self._solids)}'
        self._solids.append(Solid(self, name, len(self._solids), mass, moment_of_inertia, g))
        return self._solids[-1]

    def add_prismatic(self, s1: Solid, s2: Solid, alpha1: Physics.ANGLE = 0.0, distance1: Physics.LENGTH = 0.0, alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> Prismatic:
        print(self._check_solids(s1, s2))
        self._check_solids(s1, s2)
        if s1 is s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")
        result = Prismatic(self, len(self._joints), s1, s2, alpha1, distance1, alpha2, distance2)
        self._joints.append(result)
        return result

    def add_revolute(self, s1: Solid, s2: Solid, p1: Physics.POINT = (0.0, 0.0), p2: Physics.POINT = (0.0, 0.0)) -> Revolute:
        self._check_solids(s1, s2)
        if s1 is s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")
        result = Revolute(self, len(self._joints), s1, s2, p1, p2)
        self._joints.append(result)
        return result

    def add_pin_slot(self, s1: Solid, s2: Solid, p1: Physics.POINT = (0.0, 0.0), alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> PinSlot:
        self._check_solids(s1, s2)
        if s1 is s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")
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
        if s1 is s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")
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

    @staticmethod
    def _check_gear_solids(j1: PrimitiveJoint, j2: PrimitiveJoint, g1: None | Solid, g2: None | Solid):
        if g1 is not None:
            if g1 is not j1.s1 or g1 is not j1.s2:
                raise ex.UnrelatedObjectsError(f"{j1} does not constrain {g1}")
        if g2 is not None:
            if g2 is not j2.s1 or g2 is not j2.s2:
                raise ex.UnrelatedObjectsError(f"{j2} does not constrain {g2}")

    def add_gear_pair(self, j1: Revolute, j2: Revolute, r: Physics.DIMENSIONLESS = 0.0, v0: Physics.ANGLE = 0.0, pressure_angle: Physics.ANGLE = np.pi / 6, pinion1: None | Solid = None, pinion2: None | Solid = None) -> GearPair:
        self._check_joints(j1, j2)
        if j1 is j2:
            raise ex.ConstraintOnSameObjectError(f"Joint arguments are identical ({j1})")
        self._check_gear_solids(j1, j2, pinion1, pinion2)
        self._relations.append(g := GearPair(self, len(self._relations), j1, j2, r, v0, pressure_angle, pinion1, pinion2))
        return g

    def add_gear_rack(self, j1: Revolute, j2: Prismatic, r: Physics.LENGTH = 0.0, v0: Physics.LENGTH = 0.0, pressure_angle: Physics.ANGLE = np.pi/6, pinion: None | Solid = None, rack: None | Solid = None) -> GearRack:
        self._check_joints(j1, j2)
        if j1 is j2:
            raise ex.ConstraintOnSameObjectError(f"Joint arguments are identical ({j1})")
        self._check_gear_solids(j1, j2, pinion, rack)
        self._relations.append(g := GearRack(self, len(self._relations), j1, j2, r, v0, pressure_angle, pinion, rack))
        return g

    def add_distant_relation(self, j1: PrimitiveJoint, j2: PrimitiveJoint, r: Physics.scalar_type = 0.0, v0: Physics.scalar_type = 0.0) -> DistantRelation:
        self._check_joints(j1, j2)
        if j1 is j2:
            raise ex.ConstraintOnSameObjectError(f"Joint arguments are identical ({j1})")
        rel = DistantRelation(self, len(self._relations), j1, j2)
        # Manages units once configured
        rel.r, rel.v0 = r, v0
        self._relations.append(rel)
        return rel

    def add_effortless_relation(self, j1: PrimitiveJoint, j2: PrimitiveJoint, r: Physics.scalar_type = 0.0, v0: Physics.scalar_type = 0.0) -> EffortlessRelation:
        self._check_joints(j1, j2)
        if j1 is j2:
            raise ex.ConstraintOnSameObjectError(f"Joint arguments are identical ({j1})")
        rel = EffortlessRelation(self, len(self._relations), j1, j2)
        # Manages units once configured
        rel.r, rel.v0 = r, v0
        self._relations.append(rel)
        return rel

    def pilot(self, *joints: Joint, kw_joints: Iterable[Joint] = ()) -> None:
        total_joints = joints + tuple(kw_joints)
        self._check_joints(kw_joints=total_joints)
        self._piloted.extend(total_joints)

    def block(self, *joints: Joint, kw_joints: Iterable[Joint] = ()) -> None:
        total_joints = joints + tuple(kw_joints)
        self._check_joints(kw_joints=total_joints)
        self._blocked.extend(total_joints)

    def _hyper_statism_value(self, joint_input: list[Joint]) -> int:
        return 2 * len(self._joints) - 3 * (len(self._solids) - 1) + len(joint_input) + len(self._relations)

    def _determine_computation_order(self, input_joints, strategy_output: list[strategy.ResolutionStep]):
        h = self._hyper_statism_value(input_joints)
        if h > 0:
            raise ex.OverDeterminationError(f"System has {h} constraints in excess")
        if h < 0:
            raise ex.UnderDeterminationError(f"System is lacking {-h} constraints")
        strategy.determine_computation_order(len(self._solids), self._joints, self._relations, input_joints, strategy_output)

    def determine_computation_order(self):
        if self._blocked:
            self._determine_computation_order(self._blocked, self._dynamic_strategy)
        self._determine_computation_order(self._joints, self._kinematic_strategy)

    def show_input_order(self):
        pass

    def solve_kinematics(self, input_values: np.ndarray) -> None:
        if not self._kinematic_strategy:
            return

        for j in self._joints:
            # convert prismatic alpha/distances into points
            j._set_points()

        # save input values
        self._kinematic_inputs = input_values.copy()
        for index, joint in enumerate(j for joint in self._joints for j in joint.get_all_joints()):
            joint: PrimitiveJoint
            self._kinematic_inputs[index, :] *= Physics._get_unit_value(joint.get_input_physics())

        frame_count = self._kinematic_inputs.shape[1]
        self._solid_values.reshape((len(self._solids), 4, frame_count))
        # shapes: m, 4, n <-  1, 4, 1
        self._solid_values[:] = ((0.,), (0., ), (1.0,), (0.,)),

        self._joint_values.reshape((len(self._joints), frame_count))
        self._joint_values[:] = 0.0

        for step in self._kinematic_strategy:
            step.solve_kinematics(self._solid_values, self._joint_values, self._kinematic_inputs)

        # TODO: replace everything in ground's frame of reference

    def solve_dynamics(self, simulation_duration: Physics.TIME = 1.0) -> None:
        dynamics_strategy = self._dynamic_strategy or self._kinematic_strategy
        if not dynamics_strategy:
            return

        # TODO: compute inertia
        # TODO: compute interactions

        for step in reversed(dynamics_strategy):
            step.solve_dynamics()
