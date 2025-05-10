import kinepy.objects.config as cfg
import kinepy.objects.joints_solid as jo_so
import kinepy.objects.relations as rel
import kinepy.objects.interaction as inter
import kinepy.objects.action as act
import kinepy.math.kinematics as kin
import kinepy.math.dynamics as dyn
import kinepy.strategy as strategy
import kinepy.exceptions as ex

import numpy as np
import functools


class System:
    def __init__(self):
        self.__config = cfg.Config()

    # region Solid

    @property
    def ground(self) -> jo_so.GhostSolid:
        return jo_so.GhostSolid(self.__config, 0)

    def add_solid(self, name='', mass=0.0, moment_of_inertia=0.0, g=(0.0, 0.0)) -> jo_so.Solid:
        self.__config.assert_no_universal()
        self.__config.invalidate_config()

        index = self.__config.solids.reserve(1).start
        self.__config.solids.names[index] = name or f'jo_so.SolidBase-{index}'
        # config
        self.__config.solids.is_ghost[index] = 0
        self.__config.solids.j3dof[index] = -1

        # physics
        self.__config.solids.physics_array[index] = mass, moment_of_inertia, g[0], g[1]

        return jo_so.Solid(self.__config, index)

    # endregion Solid

    # region Joint

    def __add_joint(self, index, _type: cfg.Joints.Type, s1: jo_so.SolidBase, s2: jo_so.SolidBase, physics):
        if s1 == s2:
            raise ex.ConstraintOnSameObjectError()

        self.__config.invalidate_config()
        # config
        self.__config.joints.type_[index] = _type
        self.__config.joints.solids[index] = s1._index, s2._index
        # physics
        self.__config.joints.physics_array[index] = physics

    def add_prismatic(self, s1: jo_so.SolidBase, s2: jo_so.SolidBase, alpha1=0.0, distance1=0.0, alpha2=0.0, distance2=0.0, name='') -> jo_so.Prismatic:
        index = self.__config.joints.reserve(1).start
        self.__config.joints.names[index] = name or f'Prismatic-{index}({s2.name}/{s1.name})'
        self.__add_joint(index, cfg.Joints.Type.PRISMATIC, s1, s2, (alpha1, distance1, alpha2, distance2))
        return jo_so.Prismatic(self.__config, index)

    def add_revolute(self, s1: jo_so.SolidBase, s2: jo_so.SolidBase, p1=(0.0, 0.0), p2=(0.0, 0.0), name='') -> jo_so.Revolute:
        index = self.__config.joints.reserve(1).start
        self.__config.joints.names[index] = name or f'Revolute-{index}({s2.name}/{s1.name})'
        self.__add_joint(index, cfg.Joints.Type.REVOLUTE, s1, s2, (*p1, *p2))
        return jo_so.Revolute(self.__config, index)

    def __add_ghost_solid(self) -> int:
        self.__config.assert_no_universal()
        self.__config.invalidate_config()
        s_ghost_index = self.__config.solids.reserve(1).start
        self.__config.solids.names[s_ghost_index] = f'GhostSolid {s_ghost_index}'
        self.__config.solids.is_ghost[s_ghost_index] = True
        self.__config.solids.physics_array[s_ghost_index] = 0
        return s_ghost_index

    def __add_composite(self, name: str, type_: cfg.Composite.Type, ghost_j: slice, ghost_s: int) -> int:
        cj_index = self.__config.composite_joints.reserve(1).start
        self.__config.composite_joints.names[cj_index] = name
        self.__config.composite_joints.type_[cj_index] = type_
        self.__config.composite_joints.first_ghost_joint[cj_index] = ghost_j.start
        self.__config.composite_joints.first_ghost_solid[cj_index] = ghost_s
        return cj_index

    def add_pin_slot(self, s1: jo_so.SolidBase, s2: jo_so.SolidBase, alpha1=0.0, distance1=0.0, p2=(0.0, 0.0)) -> jo_so.PinSlot:
        s_ghost_index = self.__add_ghost_solid()

        j_ghost_indices = self.__config.joints.reserve(2)
        self.__config.joints.names[j_ghost_indices] = f'<PinSlot: {s2.name}/{s1.name} .sliding>', f'<¨PinSlot: {s2.name}/{s1.name} .angle>'
        self.__config.joints.type_[j_ghost_indices] = cfg.Joints.Type.X, cfg.Joints.Type.GHOST_ANGLE
        self.__config.joints.solids[j_ghost_indices] = (s1._index, s_ghost_index), (s_ghost_index, s2._index)
        self.__config.joints.physics_array[j_ghost_indices] = (alpha1, distance1, alpha1, 0), (0, 0, *p2)

        return jo_so.PinSlot(self.__config, self.__add_composite(f'PinSlot: {s2.name}/{s1.name}', cfg.Composite.Type.PIN_SLOT, j_ghost_indices, s_ghost_index))

    def add_translation(self, s1: jo_so.SolidBase, s2: jo_so.SolidBase, alpha1=0.0, distance1=0, alpha2=0.0, distance2=0.0, diff_angle=0.0) -> jo_so.Translation:
        s_ghost_index = self.__add_ghost_solid()

        j_ghost_indices = self.__config.joints.reserve(2)
        self.__config.joints.names[j_ghost_indices] = f'<Translation: {s2.name}/{s1.name} .x>', f'<¨Translation: {s2.name}/{s1.name} .y>'
        self.__config.joints.type_[j_ghost_indices] = cfg.Joints.Type.X, cfg.Joints.Type.Y
        self.__config.joints.solids[j_ghost_indices] = (s1._index, s_ghost_index), (s_ghost_index, s2._index)
        self.__config.joints.physics_array[j_ghost_indices] = (alpha1, distance1, alpha1, 0), (alpha2, 0, alpha2 + diff_angle, distance2)

        return jo_so.Translation(self.__config, self.__add_composite(f'Translation: {s2.name}/{s1.name}', cfg.Composite.Type.TRANSLATION, j_ghost_indices, s_ghost_index))

    # endregion Joint

    # region Relation

    def add_gear_pair(self, j1: jo_so.Revolute, j2: jo_so.Revolute, v0=0.0, r=-1.0, pressure_angle=np.pi / 9, gear1: jo_so.SolidBase | None = None, gear2: jo_so.SolidBase | None = None) -> rel.GearPair:
        self.__config.invalidate_config()

        index = self.__config.relations.reserve(1).start
        self.__config.relations.names[index] = f"GearPair{index}"
        self.__config.relations.type_[index] = cfg.Relations.Type.GEAR_PAIR
        self.__config.relations.joints[index] = j1._index, j2._index
        self.__config.relations.g1[index] = -1 if gear1 is None else gear1._index
        self.__config.relations.g2[index] = -1 if gear2 is None else gear2._index

        self.__config.relations.v0[index] = v0
        self.__config.relations.r[index] = r
        self.__config.relations.gear_pressure_angle[index] = pressure_angle

        return rel.GearPair(self.__config, index)

    def add_gear_rack(self, j1: jo_so.Revolute, j2: jo_so.Prismatic, v0=0.0, r=1.0, pressure_angle=np.pi / 9, gear1: jo_so.SolidBase | None = None, rack2: jo_so.SolidBase | None = None) -> rel.GearRack:
        self.__config.invalidate_config()

        index = self.__config.relations.reserve(1).start
        self.__config.relations.names[index] = f"GearRack{index}"
        self.__config.relations.type_[index] = cfg.Relations.Type.GEAR_RACK
        self.__config.relations.joints[index] = j1._index, j2._index
        self.__config.relations.g1[index] = -1 if gear1 is None else gear1._index
        self.__config.relations.g2[index] = -1 if rack2 is None else rack2._index

        self.__config.relations.v0[index] = v0
        self.__config.relations.r[index] = r
        self.__config.relations.gear_pressure_angle[index] = pressure_angle

        return rel.GearRack(self.__config, index)

    def add_belt(self, j1: jo_so.Revolute, j2: jo_so.Revolute, v0=0.0, r1=1.0, r2=1.0, t0=0.0, pulley1: jo_so.SolidBase | None = None, pulley2: jo_so.SolidBase | None = None) -> rel.Belt:
        self.__config.invalidate_config()

        index = self.__config.relations.reserve(1).start
        self.__config.relations.names[index] = f"BeltDrive{index}"
        self.__config.relations.type_[index] = cfg.Relations.Type.BELT
        self.__config.relations.joints[index] = j1._index, j2._index
        self.__config.relations.g1[index] = -1 if pulley1 is None else pulley1._index
        self.__config.relations.g2[index] = -1 if pulley2 is None else pulley2._index

        self.__config.relations.v0[index] = v0
        self.__config.relations.belt_r1[index] = r1
        self.__config.relations.belt_r2[index] = r2
        self.__config.relations.belt_t0[index] = t0

        return rel.Belt(self.__config, index)

    def add_distant_relation(self, j1: jo_so.Joint, j2: jo_so.Joint, v0=0.0, r=1.0):
        self.__config.invalidate_config()

        index = self.__config.relations.reserve(1).start
        self.__config.relations.names[index] = f"Distant{index}"
        self.__config.relations.type_[index] = cfg.Relations.Type.DISTANT
        self.__config.relations.joints[index] = j1._index, j2._index

        self.__config.relations.v0[index] = v0
        self.__config.relations.r[index] = r
        return rel.Distant(self.__config, index)

    def add_hydraulic_link(self, p1: jo_so.Prismatic, p2: jo_so.Prismatic, v0=0.0, surface_ratio=1.0) -> rel.Distant:
        # TODO: give it its own class and RelationType
        return self.add_distant_relation(p1, p2, v0, surface_ratio)

    def add_effortless_relation(self, j1: jo_so.Joint, j2: jo_so.Joint, v0=0.0, r=1.0) -> rel.Effortless:
        self.__config.invalidate_config()

        index = self.__config.relations.reserve(1).start
        self.__config.relations.names[index] = f"Effortless{index}"
        self.__config.relations.type_[index] = cfg.Relations.Type.EFFORTLESS
        self.__config.relations.joints[index] = j1._index, j2._index

        self.__config.relations.v0[index] = v0
        self.__config.relations.r[index] = r
        return rel.Effortless(self.__config, index)

    # endregion Relation

    # region Interaction

    def __add_universal_interaction(self, type_: cfg.Interactions.Type) -> int:
        self.__config.invalidate_resources()
        self.__config.has_universal_interaction = True

        actions = self.__config.actions.reserve(self.__config.solids.count)
        self.__config.actions.type_[actions] = cfg.Actions.Type.INTERNAL_INTERACTION
        self.__config.actions.solid[actions] = range(self.__config.solids.count)

        index = self.__config.interactions.reserve(1).start
        self.__config.interactions.type_[index] = type_
        self.__config.interactions.first_action[index] = actions.start
        return index

    def add_gravity(self, g=(0, -9.81)) -> inter.Gravity:
        index = self.__add_universal_interaction(cfg.Interactions.Type.GRAVITY)
        self.__config.interactions.g_fields[index] = g
        return inter.Gravity(self.__config, index)

    def add_inertia(self) -> inter.Inertia:
        index = self.__add_universal_interaction(cfg.Interactions.Type.INERTIA)
        return inter.Inertia(self.__config, index)

    def __add_spring(self, type_: cfg.Interactions.Type, s1: int, s2: int) -> int:
        actions = self.__config.actions.reserve(2)
        self.__config.actions.type_[actions] = cfg.Actions.Type.INTERNAL_INTERACTION
        self.__config.actions.solid[actions] = s1, s2

        index = self.__config.interactions.reserve(1).start
        self.__config.interactions.type_[index] = type_
        self.__config.interactions.first_action[index] = actions.start
        return index

    def add_linear_spring(self, s1: jo_so.SolidBase, s2: jo_so.SolidBase, p1=(0.0, 0.0), p2=(0.0, 0.0), k=0.0, l0=0.0) -> inter.LinearSpring:
        self.__config.invalidate_resources()
        _s1, _s2 = s1._index, s2._index
        index = self.__add_spring(cfg.Interactions.Type.LINEAR_SPRING, _s1, _s2)

        self.__config.interactions.linear_spring_s1[index] = _s1
        self.__config.interactions.linear_spring_s2[index] = _s2
        self.__config.interactions.spring_stiffness[index] = k
        self.__config.interactions.spring_equilibrium_position[index] = l0
        self.__config.interactions.linear_spring_p1[index] = p1
        self.__config.interactions.linear_spring_p2[index] = p2

        return inter.LinearSpring(self.__config, index)

    def add_twisting_spring(self,  r: jo_so.Revolute, k=0.0, a0=0.0) -> inter.TwistingSpring:
        self.__config.invalidate_config()
        index = self.__add_spring(cfg.Interactions.Type.TWISTING_SPRING, r._s1, r._s2)

        self.__config.interactions.twisting_spring_revolute[index] = r._index
        self.__config.interactions.spring_stiffness[index] = k
        self.__config.interactions.spring_equilibrium_position[index] = a0

        return inter.TwistingSpring(self.__config, index)

    # endregion Interaction

    def add_action(self, solid: jo_so.SolidBase, ap=(0.0, 0.0)) -> act.UserAction:
        self.__config.invalidate_resources()
        index = self.__config.actions.reserve(1).start
        self.__config.actions.type_[index] = cfg.Actions.Type.USER
        self.__config.actions.solid[index] = solid._index
        self.__config.actions.user_point[index] = ap

        return act.UserAction(self.__config, index)

    @staticmethod
    def __assert_resource(method):
        @functools.wraps(method)
        def n_method(self, *args, **kwargs):
            assert self.__config.state >= cfg.ConfigState.ALLOCATED_RESOURCES, f"Call `System.set_sim_parameters` before messing with {method.__qualname__}"
            return method(self, *args, **kwargs)
        return n_method

    @staticmethod
    def __assert_kin_ok(method):
        @functools.wraps(method)
        def n_method(self, *args, **kwargs):
            assert self.__config.state >= cfg.ConfigState.KINEMATICS_OK, f"Call `System.solve_kinematics` before messing with {method.__qualname__}"
            return method(self, *args, **kwargs)
        return n_method

    @staticmethod
    def __assert_strategy(method):
        @functools.wraps(method)
        def n_method(self, *args, **kwargs):
            assert self.__config.state >= cfg.ConfigState.STRATEGY_OK, f"Call `System.determine_computation_order` before messing with {method.__qualname__}"
            return method(self, *args, **kwargs)
        return n_method

    def determine_computation_order(self):
        if self.__config.working_joints.size:
            self._determine_computation_order(self.__config.working_joints, self.__config.dynamics_strategy)
        self._determine_computation_order(self.__config.piloted_joints, self.__config.kinematics_strategy)
        self.__config.state = cfg.ConfigState.STRATEGY_OK

    def _determine_computation_order(self, input_joints, strategy_output: list[strategy.ResolutionStep]):
        h = self._hyper_statism_value(input_joints)
        if h > 0:
            raise ex.OverDeterminationError(f"System has {h} constraints in excess")
        if h < 0:
            raise ex.UnderDeterminationError(f"System is lacking {-h} constraints")
        strategy.determine_computation_order(self.__config, input_joints, strategy_output)

    def _hyper_statism_value(self, joint_input: np.ndarray[int]) -> int:
        return 2 * self.__config.joints.count - 3 * (self.__config.solids.count - 1) + len(joint_input) + self.__config.relations.count

    @__assert_strategy
    def set_sim_parameters(self, frame_cnt: int, total_time: float = 0):
        self.__config.allocate_resources(frame_cnt)
        self.__config.frame_time = total_time / (frame_cnt - 1)

    @__assert_resource
    def solve_kinematics(self):
        kin.System.set_up(self.__config)

        for step in self.__config.kinematics_strategy:
            step.solve_kinematics(self.__config)

        kin.System.clean_up(self.__config)

        self.__config.state = cfg.ConfigState.KINEMATICS_OK

    def get_steps_with_multiple_solutions(self) -> tuple[strategy.GraphStep, ...]:
        return tuple(step for step in self.__config.kinematics_strategy if isinstance(step, strategy.GraphStep) and step.solution_count > 1)

    @__assert_kin_ok
    def solve_dynamics(self):
        dyn.System.set_up(self.__config)

        _strategy = self.__config.dynamics_strategy or self.__config.dynamics_strategy
        for step in _strategy[::-1]:
            step.solve_dynamics(self.__config)

        dyn.System.clean_up(self.__config)

        self.__config.state = cfg.ConfigState.DYNAMICS_OK

    # def kinematic_diagram(self) -> GUI:
    #     return GUI(self.__config)
