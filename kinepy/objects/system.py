from kinepy.objects.config import Config, np, ConfigState, ActionMode
import kinepy.units as u
from kinepy.objects.joints_solid import Solid, Prismatic, Revolute, PinSlot, Translation, PrimitiveJoint, CompositeType
from kinepy.strategy.graph_data import JointType, RelationType
import kinepy.exceptions as ex
import kinepy.strategy as strategy
import kinepy.math.kinematics as kin
import kinepy.math.dynamics as dyn
from kinepy.objects.interaction import Interaction, Gravity, Inertia, LinearSpring, TwistingSpring
from kinepy.objects.relations import GearRack, GearPair, Belt, Distant, Effortless
from kinepy.gui.new_gui import GUI


@u.UnitSystem.class_
class System:
    def __init__(self):
        self.__config = Config()

        self._kinematic_strategy: list[strategy.ResolutionStep] = []
        self._dynamic_strategy: list[strategy.ResolutionStep] = []

        self._interactions: list[Interaction] = []

    # region Solid

    @property
    def ground(self) -> Solid:
        return Solid(self.__config, 0)

    def add_solid(self, name='', mass: u.Mass.phy = 0.0, moment_of_inertia: u.MomentOfInertia.phy = 0.0, g: u.Length.point = (0.0, 0.0)) -> Solid:
        index = self.__config.solid_config.shape[0]
        self.__config.add_solids(
            [name or f'Solid-{index}'],
            np.r_[mass, moment_of_inertia, g][np.newaxis, :]
        )
        return Solid(self.__config, index)

    def _check_solids_ownership(self, *solids: Solid, kw_solids: tuple[Solid, ...] = ()):
        for solid in solids + kw_solids:
            if not solid.check_against(self.__config):
                raise ex.UnrelatedObjectsError(f"Solid \"{solid}\" does not belong to this system")

    def _check_solids(self, s1: Solid, s2: Solid):
        self._check_solids_ownership(s1, s2)
        if s1 == s2:
            raise ex.ConstraintOnSameObjectError(f"Solid arguments are identical ({s1})")

    # endregion Solid

    # region Joint

    def add_prismatic(self, s1: Solid, s2: Solid, alpha1: u.Angle.phy = 0.0, distance1: u.Length.phy = 0.0, alpha2: u.Angle.phy = 0.0, distance2: u.Length.phy = 0.0, name='') -> Prismatic:
        self._check_solids(s1, s2)
        index = self.__config.joint_config.shape[0]
        self.__config.add_joints([name or f'Prismatic-{index}({s2.name}/{s1.name})'], np.array([[JointType.PRISMATIC.value, s1._index, s2._index]], int), np.array([[alpha1, distance1, alpha2,  distance2]]))
        return Prismatic(self.__config, index)

    def add_revolute(self, s1: Solid, s2: Solid, p1: u.Length.point = (0.0, 0.0), p2: u.Length.point = (0.0, 0.0), name='') -> Revolute:
        self._check_solids(s1, s2)
        index = self.__config.joint_config.shape[0]
        self.__config.add_joints([name or f'Revolute-{index}({s2.name}/{s1.name})'], np.array([[JointType.REVOLUTE.value, s1._index, s2._index]], int), np.r_[p1, p2][np.newaxis, :])
        return Revolute(self.__config, index)

    def add_pin_slot(self, s1: Solid, s2: Solid, alpha1: u.Angle.phy = 0.0, distance1: u.Length.phy = 0.0, p2: u.Length.point = (0.0, 0.0)) -> PinSlot:
        self._check_solids(s1, s2)

        s_ghost_index = self.__config.solid_physics.shape[0]
        self.__config.add_solids([f'GhostSolid {s_ghost_index}'], np.zeros((1, 4)))

        j_ghost_index = self.__config.joint_config.shape[0]
        self.__config.add_joints(
            [f"<PinSlot: {s2.name}/{s1.name} .sliding>", f"<¨PinSlot: {s2.name}/{s1.name} .angle>"],
            np.array([[JointType.X.value, s1._index, s_ghost_index], [JointType.GHOST_ANGLE.value, s_ghost_index, s2._index]]),
            np.array(([alpha1, distance1, alpha1, 0], np.r_[0, 0, p2]))
        )

        cj_index = self.__config.composite_joint_config.shape[0]
        self.__config.add_composite(CompositeType.PIN_SLOT.value, (j_ghost_index, j_ghost_index+1), (s_ghost_index,))
        return PinSlot(self.__config, cj_index)

    def add_translation(self, s1: Solid, s2: Solid, alpha1: u.Angle.phy = 0.0, distance1: u.Length.phy = 0.0, alpha2: u.Angle.phy = 0.0, distance2: u.Length.phy = 0.0, diff_angle: u.Angle.phy = 0.0) -> Translation:
        self._check_solids(s1, s2)

        s_ghost_index = self.__config.solid_physics.shape[0]
        self.__config.add_solids([f'GhostSolid {s_ghost_index}'], np.zeros((1, 4)))

        j_ghost_index = self.__config.joint_config.shape[0]
        self.__config.add_joints(
            [f"<¨Translation: {s2.name}/{s1.name} .x>", f"<Translation: {s2.name}/{s1.name} .y>"],
            np.array([[JointType.X.value, s1._index, s_ghost_index], [JointType.Y.value, s_ghost_index, s2._index]]),
            np.array(([alpha1, distance1, alpha1, 0], [alpha2, 0, alpha2 + diff_angle, distance2]))
        )
        cj_index = self.__config.composite_joint_config.shape[0]
        self.__config.add_composite(CompositeType.PIN_SLOT.value, (j_ghost_index, j_ghost_index + 1), (s_ghost_index,))

        return Translation(self.__config, cj_index)

    # endregion Joint

    def determine_computation_order(self):
        self.__config.state = ConfigState.STRATEGY_OK
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

    def set_sim_parameters(self, frame_cnt: int, total_time: u.Time.phy = 0.0):
        assert self.__config.state >= ConfigState.STRATEGY_OK, "Call `System.determine_computation_order` before allocating resources"
        for inter in self._interactions:
            inter._claim_resources()
        self.__config.allocate_results(frame_cnt, total_time / (frame_cnt - 1) if frame_cnt > 1 else total_time)

    def solve_kinematics(self):
        assert self.__config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters` before solving kinematics"
        self.__config.state = ConfigState.KINEMATICS_OK

        kin.System.set_up(self.__config)

        for step in self._kinematic_strategy:
            step.solve_kinematics(self.__config)

        kin.System.clean_up(self.__config)

    def get_steps_with_multiple_solutions(self) -> tuple[strategy.GraphStep, ...]:
        return tuple(step for step in self._kinematic_strategy if isinstance(step, strategy.GraphStep) and step.solution_count > 1)

    def solve_dynamics(self):
        assert self.__config.state >= ConfigState.KINEMATICS_OK, "Call `System.solve_kinematics` before solving dynamics"
        self.__config.state = ConfigState.DYNAMICS_OK

        dyn.System.set_up(self.__config)

        for inter in self._interactions:
            inter._set_actions()

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
            np.array([[RelationType.GEAR.value, j1._index, j2._index, -1 if gear1 is None else gear1._index, -1 if gear2 is None else gear2._index]]),
            np.array([[v0, r, pressure_angle, 0.0]])
        )
        return GearPair(self.__config, index)

    def add_gear_rack(self, j1: Revolute, j2: Prismatic, v0: u.Length.phy = 0.0, r: u.Length.phy = 1.0, pressure_angle: u.Angle.phy = np.pi / 9, gear1: Solid | None = None, gear2: Solid | None = None):
        index = self.__config.relation_config.shape[0]
        self.__config.add_relations(
            np.array([[RelationType.GEAR_RACK.value, j1._index, j2._index, -1 if gear1 is None else gear1._index, -1 if gear2 is None else gear2._index]]),
            np.array([[v0, r, pressure_angle, 0.0]])
        )
        return GearRack(self.__config, index)

    def add_belt(self, j1: Revolute, j2: Revolute, v0: u.Angle.phy = 0.0, r1: u.Length.phy = 1.0, r2: u.Length.phy = 1.0, t0: u.Force.phy = 0.0, shaft1: Solid | None = None, shaft2: Solid | None = None):
        index = self.__config.relation_config.shape[0]
        self.__config.add_relations(
            np.array([[RelationType.BELT.value, j1._index, j2._index, -1 if shaft1 is None else shaft1._index, -1 if shaft2 is None else shaft2._index]]),
            np.array([[v0, r1, r2, t0]])
        )
        return Belt(self.__config, index)

    def add_distant_relation(self, j1: PrimitiveJoint, j2: PrimitiveJoint, v0: u.Angle.phy = 0.0, r: u.Dimensionless.phy = 1.0):
        index = self.__config.relation_config.shape[0]
        self.__config.add_relations(
            np.array([[RelationType.DISTANT.value, j1._index, j2._index, -1, -1]]),
            np.array([[v0, r, 0.0, 0.0]])
        )
        return Distant(self.__config, index)

    def add_effortless_relation(self, j1: PrimitiveJoint, j2: PrimitiveJoint, v0: u.Angle.phy = 0.0, r: u.Dimensionless.phy = 1.0):
        index = self.__config.relation_config.shape[0]
        self.__config.add_relations(
            np.array([[RelationType.EFFORTLESS.value, j1._index, j2._index, -1, -1]]),
            np.array([[v0, r, 0.0, 0.0]])
        )
        return Effortless(self.__config, index)

    def add_gravity(self, g: u.Acceleration.point = (0, -u.Acceleration.G.value)) -> Gravity:
        self._interactions.append(gravity := Gravity(self.__config, dict(), g))
        gravity._claim_resources()
        return gravity

    def add_inertia(self) -> Inertia:
        self._interactions.append(inertia := Inertia(self.__config, dict()))
        inertia._claim_resources()
        return inertia

    def add_linear_spring(self, s1: Solid, s2: Solid, p1: u.Length.point = (0.0, 0.0), p2: u.Length.point = (0.0, 0.0), k: u.SpringConstant.phy = 0.0, l0: u.Length.phy = 0.0) -> LinearSpring:
        _action_index = self.__config.action_config.shape[0]
        self.__config.add_actions(
            np.array([
                [s1._index, ActionMode.NO_INDIRECTION, 0],
                [s2._index, ActionMode.NO_INDIRECTION, 0]
            ]),
            np.array([p1, p2])
        )
        ls = LinearSpring(self.__config, {s1._index: _action_index, s2._index: _action_index+1}, s1, s2, p1, p2, k, l0)
        self._interactions.append(ls)
        return ls

    def add_twisting_spring(self,  r: Revolute, k: u.Torque.phy = 0.0, a0: u.Angle.phy = 0.0) -> TwistingSpring:
        _action_index = self.__config.action_config.shape[0]
        self.__config.add_actions(
            np.array([
                [r._s1, ActionMode.JOINT_POINT, r._index],
                [r._s2, ActionMode.JOINT_POINT, r._index]
            ]),
            np.zeros((2, 2))
        )
        ts = TwistingSpring(self.__config, {r._s1: _action_index, r._s2: _action_index+1}, r, k, a0)
        self._interactions.append(ts)
        return ts

    def kinematic_diagram(self) -> GUI:
        return GUI(self.__config)
