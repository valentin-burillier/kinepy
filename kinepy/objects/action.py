from kinepy.objects.config import *
from kinepy.strategy.graph_data import JointType
from kinepy.math.geometry import Position
import kinepy.units as u


class Action(ConfigView):

    def __get_ap(self, indirection: int) -> u.point_type:
        return self._config.action_physics[self._index]

    def __get_solid_g(self, indirection: int) -> u.point_type:
        return self._config.solid_physics[indirection, Config.SOLID_CFG_G]

    def __get_joint_point(self, indirection: int) -> u.point_type:
        _solid = self._config.action_config[self._index, Config.ACTION_SOLID]
        _j_type, s1, s2 = self._config.joint_config[indirection]

        if _solid == s1:
            _slice = Config.JOINT_P1
        elif _solid == s2:
            _slice = Config.JOINT_P2
        else:
            raise ValueError("Action applies on a joint its solid is not constrained by")
        if _j_type == JointType.REVOLUTE:
            return self._config.joint_physics[indirection, _slice]
        else:
            angle, dist = self._config.joint_physics[indirection, _slice]
            return np.array([-np.sin(angle) * dist, np.cos(angle) * dist])

    __getter = {
        ActionMode.NO_INDIRECTION: __get_ap,
        ActionMode.SOLID_G: __get_solid_g,
        ActionMode.JOINT_POINT: __get_joint_point
    }

    @property
    def ap(self) -> u.Length.point:
        _type = ActionMode(self._config.action_config[self._index, Config.ACTION_MODE])
        _indirection = self._config.action_config[self._index, Config.ACTION_INDIRECTION]
        return self.__getter[_type](_indirection)

    @ap.setter
    def ap(self, value: u.Length.point):
        _type = ActionMode(self._config.action_config[self._index, Config.ACTION_MODE])
        if _type != ActionMode.NO_INDIRECTION:
            raise ValueError("You cannot modify this value from here")
        self._config.invalidate_dynamics()
        self._config.action_physics[self._index] = np.array(value)

    def set_force(self, value: u.Force.point):
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_frame_count` before setting values"
        self._config.results.action_values[self._index, :, Config.ACTION_DYN_FORCE] = value

    def set_force_locally(self, value: u.Length.point):
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_frame_count` before setting values"
        _solid = self._config.action_config[self._index, Config.ACTION_SOLID]
        self._config.results.action_values[self._index, :, Config.ACTION_DYN_FORCE] = Position.local_point(self._config, _solid, np.array(value))

    def get_force(self) -> u.Force.point:
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_frame_count` before getting values"
        return self._config.results.action_values[self._index, :, Config.ACTION_DYN_FORCE]

    def set_torque(self, value: u.Torque.phy):
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_frame_count` before setting values"
        self._config.results.action_values[self._index, :, Config.ACTION_DYN_TORQUE] = value

    def get_torque(self) -> u.Torque.phy:
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_frame_count` before getting values"
        return self._config.results.action_values[self._index, :, Config.ACTION_DYN_TORQUE]


class InternalAction(Action):
    ap = disable_set(Action.ap)

    def __do_not_set(self, value):
        raise AttributeError("You cannot manually set values of InternalAction")

    def set_force_locally(self, value: u.Length.point):
        return self.__do_not_set(value)

    def set_force(self, value: u.Force.point):
        return self.__do_not_set(value)

    def set_torque(self, value: u.Torque.phy):
        return self.__do_not_set(value)
