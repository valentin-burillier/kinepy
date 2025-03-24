from kinepy.objects.config import Config, np
import kinepy.units as u
from kinepy.objects.joints_solid import Solid, Prismatic, Revolute, PinSlot, Translation, TranslationAxleX, TranslationAxleY, PinSlotAngle, PinSlotSliding, GhostSolid
from kinepy.strategy.graph_data import JointType, RelationType
import kinepy.exceptions as ex
import kinepy.strategy as strategy
import kinepy.math.kinematics as kin
import kinepy.math.dynamics as dyn
from kinepy.objects.interaction import Interaction
from kinepy.objects.relations import GearRack, GearPair, Belt, Distant, Effortless

@u.UnitSystem.class_
class System:
    def __init__(self):
        self.__config = Config()
        self._ground = GhostSolid(self.__config, 0, 'Ground')

        self._kinematic_strategy: list[strategy.ResolutionStep] = []
        self._dynamic_strategy: list[strategy.ResolutionStep] = []

        self._interactions: list[Interaction] = []

    @property
    def ground(self) -> Solid:
        return self._ground

    def add_solid(self, name='', mass: u.Mass.phy = 0.0, moment_of_inertia: u.MomentOfInertia.phy = 0.0, g: u.Length.point = (0.0, 0.0)) -> Solid:
        index = self.__config.solid_physics.shape[0]
        self.__config.add_solids(np.r_[mass, moment_of_inertia, g][np.newaxis, :])
        return Solid(self.__config, index, name)

    def _check_solids_ownership(self, *solids: Solid, kw_solids: tuple[Solid, ...] = ()):
        for solid in solids + kw_solids:
            if not solid.check_against(self.__config, self.__config.solid_physics):
                raise ex.UnrelatedObjectsError(f"Solid \"{solid}\" does not belong to this system")

    def _check_solids(self, s1: Solid, s2: Solid):
        self._check_solids_ownership(s1, s2)
        if s1 == s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")

    def add_prismatic(self, s1: Solid, s2: Solid, alpha1: u.Angle.phy = 0.0, distance1: u.Length.phy = 0.0, alpha2: u.Angle.phy = 0.0, distance2: u.Length.phy = 0.0) -> Prismatic:
        self._check_solids(s1, s2)
        index = self.__config.joint_config.shape[0]
        self.__config.add_joints(np.array([[JointType.PRISMATIC.value, s1._index, s2._index]], int), np.array([[alpha1, distance1, alpha2,  distance2]]))
        return Prismatic(self.__config, index, s1, s2)

    def add_revolute(self, s1: Solid, s2: Solid, p1: u.Length.point = (0.0, 0.0), p2: u.Length.point = (0.0, 0.0)) -> Revolute:
        self._check_solids(s1, s2)
        index = self.__config.joint_config.shape[0]
        self.__config.add_joints(np.array([[JointType.REVOLUTE.value, s1._index, s2._index]], int), np.r_[p1, p2][np.newaxis, :])
        return Revolute(self.__config, index, s1, s2)

    def add_pin_slot(self, s1: Solid, s2: Solid, alpha1: u.Angle.phy = 0.0, distance1: u.Length.phy = 0.0, p2: u.Length.point = (0.0, 0.0)) -> PinSlot:
        self._check_solids(s1, s2)

        s_ghost_index = self.__config.solid_physics.shape[0]
        self.__config.add_solids(np.zeros((1, 4)))

        j_ghost_index = self.__config.joint_config.shape[0]
        self.__config.add_joints(
            np.array([[JointType.PRISMATIC.value, s1._index, s_ghost_index], [JointType.REVOLUTE.value, s_ghost_index, s2._index]]),
            np.array(([alpha1, distance1, alpha1, 0], np.r_[0, 0, p2]))
        )

        ghost_solid = GhostSolid(self.__config, s_ghost_index, f'GhostSolid {s_ghost_index}')
        ghost_joints = (
            PinSlotSliding(self.__config, j_ghost_index, s1, ghost_solid, f"<PinSlot: {s2.name}/{s1.name} .sliding>"),
            PinSlotAngle(self.__config, j_ghost_index+1, ghost_solid, s2, f"<¨PinSlot: {s2.name}/{s1.name} .angle>")
        )
        return PinSlot(ghost_joints, (ghost_solid,))

    def add_translation(self, s1: Solid, s2: Solid, alpha1: u.Angle.phy = 0.0, distance1: u.Length.phy = 0.0, alpha2: u.Angle.phy = 0.0, distance2: u.Length.phy = 0.0, diff_angle: u.Angle.phy = 0.0) -> Translation:
        self._check_solids(s1, s2)

        s_ghost_index = self.__config.solid_physics.shape[0]
        self.__config.add_solids(np.zeros((1, 4)))

        j_ghost_index = self.__config.joint_config.shape[0]
        self.__config.add_joints(
            np.array([[JointType.PRISMATIC.value, s1._index, s_ghost_index], [JointType.PRISMATIC.value, s_ghost_index, s2._index]]),
            np.array(([alpha1, distance1, alpha1, 0], [alpha2, 0, alpha2 + diff_angle, distance2]))
        )

        ghost_solid = GhostSolid(self.__config, s_ghost_index, f'GhostSolid {s_ghost_index}')
        ghost_joints = (
            TranslationAxleX(self.__config, j_ghost_index, s1, ghost_solid, f"<¨Translation: {s2.name}/{s1.name} .angle>"),
            TranslationAxleY(self.__config, j_ghost_index + 1, ghost_solid, s2, f"<Translation: {s2.name}/{s1.name} .sliding>")
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
        strategy.determine_computation_order(self.__config, input_joints, strategy_output)

    def _hyper_statism_value(self, joint_input: np.ndarray[int]) -> int:
        return 2 * self.__config.joint_config.shape[0] - 3 * (self.__config.solid_physics.shape[0] - 1) + len(joint_input) + self.__config.relation_config.shape[0]

    def set_frame_count(self, frame_cnt: int, frame_time: u.Time.phy = 0.0):
        self.__config.allocate_results(frame_cnt, frame_time)

    def solve_kinematics(self):
        kin.System.set_up(self.__config)

        for step in self._kinematic_strategy:
            step.solve_kinematics(self.__config)

        kin.System.clean_up(self.__config)

    def get_steps_with_multiple_solutions(self) -> tuple[strategy.GraphStep, ...]:
        return tuple(step for step in self._kinematic_strategy if isinstance(step, strategy.GraphStep) and step.solution_count > 1)

    def solve_dynamics(self):
        dyn.System.set_up(self.__config)

        for inter in self._interactions:
            inter.register_actions()
        _strategy = self._dynamic_strategy or self._kinematic_strategy
        for step in _strategy[::-1]:
            step.solve_dynamics(self.__config)

        dyn.System.clean_up(self.__config)

    def add_interaction(self, interaction: Interaction):
        self._interactions.append(interaction)
        interaction._config = self.__config

    def add_gear_pair(self, j1: Revolute, j2: Revolute, v0: u.Angle.phy = 0.0, r: u.Dimensionless.phy = -1.0, pressure_angle: u.Angle.phy = np.pi / 9, gear1: Solid | None = None, gear2: Solid | None = None):
        index = self.__config.relation_config.shape[0]
        self.__config.add_relations(
            np.array([[RelationType.GEAR.value, j1._index, j2._index, -1 if gear1 is None else gear1._index, -1 if gear2 else gear2._index]]),
            np.array([[v0, r, pressure_angle, 0.0]])
        )
        return GearPair(self.__config, index, j1, j2, gear1, gear2)

    def add_gear_rack(self, j1: Revolute, j2: Prismatic, v0: u.Length.phy = 0.0, r: u.Length.phy = 1.0, pressure_angle: u.Angle.phy = np.pi / 9, gear1: Solid | None = None, gear2: Solid | None = None):
        index = self.__config.relation_config.shape[0]
        self.__config.add_relations(
            np.array([[RelationType.GEAR_RACK.value, j1._index, j2._index, -1 if gear1 is None else gear1._index, -1 if gear2 else gear2._index]]),
            np.array([[v0, r, pressure_angle, 0.0]])
        )
        return GearRack(self.__config, index, j1, j2, gear1, gear2)

    def add_belt(self, j1: Revolute, j2: Revolute, v0: u.Angle.phy = 0.0, r1: u.Length.phy = 1.0, r2: u.Length.phy = 1.0, t0: u.Force.phy = 0.0, shaft1: Solid | None = None, shaft2: Solid | None = None):
        index = self.__config.relation_config.shape[0]
        self.__config.add_relations(
            np.array([[RelationType.BELT.value, j1._index, j2._index, -1 if shaft1 is None else shaft1._index, -1 if shaft2 else shaft2._index]]),
            np.array([[v0, r1, r2, t0]])
        )
        return Belt(self.__config, index, j1, j2, shaft1, shaft2)

    def add_distant_relation(self, j1: Revolute, j2: Revolute, v0: u.Angle.phy = 0.0, r: u.Dimensionless.phy = 1.0):
        index = self.__config.relation_config.shape[0]
        self.__config.add_relations(
            np.array([[RelationType.DISTANT.value, j1._index, j2._index, -1, -1]]),
            np.array([[v0, r, 0.0, 0.0]])
        )
        return Distant(self.__config, index, j1, j2)

    def add_effortless_relation(self, j1: Revolute, j2: Revolute, v0: u.Angle.phy = 0.0, r: u.Dimensionless.phy = 1.0):
        index = self.__config.relation_config.shape[0]
        self.__config.add_relations(
            np.array([[RelationType.EFFORTLESS.value, j1._index, j2._index, -1, -1]]),
            np.array([[v0, r, 0.0, 0.0]])
        )
        return Effortless(self.__config, index, j1, j2)
