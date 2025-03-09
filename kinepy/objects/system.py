from kinepy.objects.config import Config, np
import kinepy.units as u
from kinepy.objects.joints_solid import Solid, Prismatic, Revolute, PinSlot, Translation, CompositeJoint
from kinepy.strategy.graph_data import JointType
import kinepy.exceptions as ex
import kinepy.strategy as strategy

@u.UnitSystem.class_
class System:
    def __init__(self):
        self.__config = Config()
        self._ground = CompositeJoint.GSolid(self.__config, 0, 'Ground')

        self._kinematic_strategy = []
        self._dynamic_strategy = []
        self._final_joint_states = []

    @property
    def ground(self) -> Solid:
        return self._ground

    def add_solid(self, name='', mass: u.Mass.phy = 0.0, moment_of_inertia: u.MomentOfInertia.phy = 0.0, g: u.Length.point = (0.0, 0.0)) -> Solid:
        index = self.__config.solid_physics.shape[0]
        self.__config.add_solids(np.r_[mass, moment_of_inertia, g][np.newaxis, :])
        return Solid(self.__config, index, name)

    def _check_solids(self, *solids: Solid, kw_solids: tuple[Solid, ...] = ()):
        for solid in solids + kw_solids:
            if not solid.check_against(self.__config, self.__config.solid_physics):
                raise ex.UnrelatedObjectsError(f"Solid \"{solid}\" does not belong to this system")

    def add_prismatic(self, s1: Solid, s2: Solid, alpha1: u.Angle.phy = 0.0, distance1: u.Length.phy = 0.0, alpha2: u.Angle.phy = 0.0, distance2: u.Length.phy = 0.0) -> Prismatic:
        self._check_solids(s1, s2)
        if s1 is s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")
        index = self.__config.joint_config.shape[0]
        self.__config.add_joints(np.array([[JointType.PRISMATIC.value, s1._index, s2._index]], int), np.array([[np.cos(alpha1) * distance1, np.sin(alpha1) * distance1, np.cos(alpha2) * distance2, np.sin(alpha2) * distance2]]))
        return Prismatic(self.__config, index, s1, s2)

    def add_revolute(self, s1: Solid, s2: Solid, p1: u.Length.point = (0.0, 0.0), p2: u.Length.point = (0.0, 0.0)) -> Revolute:
        self._check_solids(s1, s2)
        if s1 is s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")
        index = self.__config.joint_config.shape[0]
        self.__config.add_joints(np.array([[JointType.REVOLUTE.value, s1._index, s2._index]], int), np.r_[p1, p2][np.newaxis, :])
        return Revolute(self.__config, index, s1, s2)

    def add_pin_slot(self, s1: Solid, s2: Solid, p1: u.Length.point = (0.0, 0.0), alpha2: u.Angle.phy = 0.0, distance2: u.Length.phy = 0.0) -> PinSlot:
        self._check_solids(s1, s2)
        if s1 is s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")

        s_ghost_index = self.__config.solid_physics.shape[0]
        self.__config.add_solids(np.zeros((1, 4)))

        j_ghost_index = self.__config.joint_config.shape[0]
        self.__config.add_joints(
            np.array([[JointType.REVOLUTE.value, s1._index, s_ghost_index], [JointType.PRISMATIC.value, s_ghost_index, s2._index]]),
            np.array((np.r_[p1, 0, 0], [alpha2, 0, alpha2, distance2]))
        )

        ghost_solid = CompositeJoint.GSolid(self.__config, s_ghost_index, f'GhostSolid {s_ghost_index}')
        ghost_joints = (
            PinSlot.Angle(self.__config, j_ghost_index, s1, ghost_solid, f"<¨PinSlot: {s2.name}/{s1.name} .angle>"),
            PinSlot.Sliding(self.__config, j_ghost_index + 1, ghost_solid, s2, f"<PinSlot: {s2.name}/{s1.name} .sliding>")
        )
        return PinSlot(ghost_joints, (ghost_solid,))

    def add_translation(self, s1: Solid, s2: Solid, alpha1: u.Angle.phy = 0.0, distance1: u.Length.phy = 0.0, alpha2: u.Angle.phy = 0.0, distance2: u.Length.phy = 0.0, diff_angle: u.Angle.phy = 0.0) -> Translation:
        self._check_solids(s1, s2)
        if s1 is s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")

        s_ghost_index = self.__config.solid_physics.shape[0]
        self.__config.add_solids(np.zeros((1, 4)))

        j_ghost_index = self.__config.joint_config.shape[0]
        self.__config.add_joints(
            np.array([[JointType.PRISMATIC.value, s1._index, s_ghost_index], [JointType.PRISMATIC.value, s_ghost_index, s2._index]]),
            np.array(([alpha1, distance1, alpha1, 0], [alpha2, 0, alpha2 + diff_angle, distance2]))
        )

        ghost_solid = CompositeJoint.GSolid(self.__config, s_ghost_index, f'GhostSolid {s_ghost_index}')
        ghost_joints = (
            Translation.AxleX(self.__config, j_ghost_index, s1, ghost_solid, f"<¨Translation: {s2.name}/{s1.name} .angle>"),
            Translation.AxleY(self.__config, j_ghost_index + 1, ghost_solid, s2, f"<Translation: {s2.name}/{s1.name} .sliding>")
        )
        return Translation(ghost_joints, (ghost_solid,))

    def determine_computation_order(self):
        if self.__config.working_joints.size:
            self._determine_computation_order(self.__config.working_joints, self._dynamic_strategy)
        self._determine_computation_order(self.__config.piloted_joints, self._kinematic_strategy)

    def _determine_computation_order(self, input_joints, strategy_output: list[strategy.ResolutionStep]):
        h = self._hyper_statism_value(input_joints)
        if h > 0:
            raise ex.OverDeterminationError(f"System has {h} constraints in excess")
        if h < 0:
            raise ex.UnderDeterminationError(f"System is lacking {-h} constraints")
        strategy.determine_computation_order(self.__config, input_joints, self._final_joint_states, strategy_output)

    def _hyper_statism_value(self, joint_input: np.ndarray[int]) -> int:
        return 2 * self.__config.joint_config.shape[0] - 3 * (self.__config.solid_physics.shape[0] - 1) + len(joint_input) + self.__config.relation_config.shape[0]
