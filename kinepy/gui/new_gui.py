import enum
import os

import numpy as np

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import pygame as pg
import kinepy.gui.new_meshes as meshes
import kinepy.math.geometry as geo
from kinepy.objects.config import Config, ConfigState
from kinepy.objects.joints_solid import CompositeType, JointType, PrimitiveJoint
import kinepy.strategy.types as strategy
import time

_icon_path = os.path.join(os.path.dirname(__file__), 'logo.ico')



class _GUIObject:
    def draw(self, surface: pg.Surface, solid_values, frame_index, scale, translation):
        pass

    def update_bbox(self, bbox, solid_values):
        pass

    @staticmethod
    def update_bbox_from_point(bbox, solid_values, point):
        positions = solid_values[0:2] + geo.Orientation.add(solid_values[2:4], point[:, np.newaxis])
        bbox[0:2] = np.minimum(bbox[0:2], np.nanmin(positions, axis=-1))
        bbox[2:4] = np.maximum(bbox[2:4], np.nanmax(positions, axis=-1))


class _Symbol(_GUIObject):
    def __init__(self, point, mesh, mounting_point, grounded: bool):
        self.point = point
        self.mesh = mesh
        self.mesh_mounting_point = mounting_point
        if grounded:
            self.region = -1
        else:
            self.region = 2 * (point[1] > point[0]) + (point[1] > -point[0])

    def update_bbox(self, bbox, solid_values):
        self.update_bbox_from_point(bbox, solid_values, self.point)

    def draw(self, surface: pg.Surface, solid_values, frame_index, scale, translation):
        point = (solid_values[0:2, frame_index] + geo.Orientation.add_s(solid_values[2:4, frame_index], self.point)) * scale + translation
        mesh = geo.Orientation.sub_m(self.mesh, solid_values[2:4, frame_index]) + point
        pg.draw.polygon(surface, (0, 0, 0), mesh, 0)
        pg.draw.polygon(surface, (255, 255, 255), mesh, 3)

    @classmethod
    def from_revolute(cls, r_index, config: Config, mesh, mounting_point):
        s1, s2 = config.joint_config[r_index, Config.JOINT_SOLIDS]
        point = config.joint_physics[r_index, Config.JOINT_P2]

        if not s2:
            # point towards south
            mesh = mesh * (1, -1)
        else:
            # point towards solid ref
            region = 2 * (point[1] > point[0]) + (point[1] > -point[0])
            if region in (1, 2):
                mesh = mesh[:, ::-1]
            if region & 1:
                mesh = mesh * (1, -1)

        return cls(point, mesh, mounting_point, not s2)

    @classmethod
    def from_prismatic(cls, p_index, config: Config, mesh, mounting_point):
        s1, s2 = config.joint_config[p_index, Config.JOINT_SOLIDS]
        angle, dist = config.joint_physics[p_index, Config.JOINT_P2]
        angle = (angle + np.pi) % (2 * np.pi) - np.pi

        if not s2:
            # point towards south
            if abs(angle) < np.pi * 0.5:
                mesh = mesh * (1, -1)
        elif dist > 0:
            # point towards solid ref
            mesh = mesh * (1, -1)

        mesh = geo.Orientation.add(geo.Orientation.from_angle(angle), mesh)

        point = geo.Orientation.from_angle(np.array(angle + np.pi)) * dist
        return cls(point, mesh, mounting_point, not s2)


class _Sliding(_GUIObject):
    def __init__(self, start_point, end_point):
        self.start_point, self.end_point = start_point, end_point

    def update_bbox(self, bbox, solid_values):
        self.update_bbox_from_point(bbox, solid_values, self.start_point)
        self.update_bbox_from_point(bbox, solid_values, self.end_point)

    def draw(self, surface: pg.Surface, solid_values, frame_index, scale, translation):
        start = (solid_values[0:2, frame_index] + geo.Orientation.add_s(solid_values[2:4, frame_index], self.start_point)) * scale + translation
        end = (solid_values[0:2, frame_index] + geo.Orientation.add_s(solid_values[2:4, frame_index], self.end_point)) * scale + translation

        pg.draw.line(surface, (255, 255, 255), start, end, 3)

    @classmethod
    def from_prismatic(cls, p_index, config: Config):
        s1, s2 = config.joint_config[p_index, Config.JOINT_SOLIDS]
        angle, dist = config.joint_physics[p_index, Config.JOINT_P1]
        point = geo.Orientation.from_angle(np.array(angle + np.pi * 0.5)) * dist
        v_dir = geo.Orientation.from_angle(np.array(angle))

        # TODO: change PrimitiveJoint to use PrimitiveJoint.get_value()
        if config.joint_states[p_index] ^ strategy.JointFlags.READY_FOR_USER:
            strategy.JointValueComputationStep(p_index, JointType.PRISMATIC, config.joint_states[p_index], s1, s2).solve_kinematics(config)
            config.joint_states[p_index] = strategy.JointFlags.READY_FOR_USER
        sliding = config.results.joint_values[p_index]
        return cls(point + np.nanmin(sliding) * v_dir, point + np.nanmax(sliding) * v_dir)


class GUI:
    def __init__(self, config: Config):
        self._config = config

        # layer 1: ground markers, tree branches, sliders
        self._solid_objects_1: dict[int, list[_GUIObject]] = {}
        # layer 2: joint symbols, single point symbols
        self._solid_objects_2: dict[int, list[_GUIObject]] = {}

    def _do_nothing(self, index: int):
        pass

    def _add_pin_slot(self, index: int):
        s1, s2 = self._config.get_composite_solids(index)
        p, r, _ = self._config.composite_joint_config[index, Config.COMPOSITE_JOINTS]

        self._solid_objects_1[s1].append(_Sliding.from_prismatic(p, self._config))
        self._solid_objects_2[s2].append(_Symbol.from_revolute(r, self._config, meshes.PIN_SLOT, meshes.PIN_SLOT_MOUNTING_POINT))

    def _add_revolute(self, index: int):
        s1, s2 = self._config.joint_config[index, Config.JOINT_SOLIDS]
        self._solid_objects_2[s2].append(_Symbol.from_revolute(index, self._config, meshes.REVOLUTE, meshes.REVOLUTE_MOUNTING_POINT))

    def _add_prismatic(self, index: int):
        s1, s2 = self._config.joint_config[index, Config.JOINT_SOLIDS]
        self._solid_objects_2[s1].append(_Sliding.from_prismatic(index, self._config))
        self._solid_objects_2[s2].append(_Symbol.from_revolute(index, self._config, meshes.PRISMATIC, meshes.PRISMATIC_MOUNTING_POINT))

    _composite_additions = {
        CompositeType.PIN_SLOT: _add_pin_slot,
        CompositeType.TRANSLATION: _do_nothing,
        CompositeType.J3DOF: _do_nothing
    }

    _joint_additions = {
       JointType.REVOLUTE: _add_revolute,
       JointType.PRISMATIC: _add_prismatic
    }

    def _prepare(self):
        self._solid_objects_1.clear()
        self._solid_objects_2.clear()

        _solid_visibility = [1] * len(self._config.solid_config)
        _joint_visibility = [1] * self._config.joint_config.shape[0]
        _composite_joint_visibility = [1] * self._config.composite_joint_config.shape[0]
        # TODO: add relation visibility
        # TODO: add force visibility

        # hide ghosts
        for _, gj1, gj2, gj3, gs1, gs2 in self._config.composite_joint_config:
            _solid_visibility[gs1] = 0
            if gs2 > 0:
                _solid_visibility[gs2] = 0

            _joint_visibility[gj1] = 0
            _joint_visibility[gj2] = 0
            if gj3 > -1:
                _joint_visibility[gj3] = 0

        # TODO: add user requested hidden joints/solids

        for solid, _ in filter(lambda x: x[1], enumerate(_solid_visibility)):
            self._solid_objects_1[solid] = []
            self._solid_objects_2[solid] = []

        for cj_index, _ in filter(lambda x: x[1], enumerate(_composite_joint_visibility)):
            s1, s2 = self._config.get_composite_solids(cj_index)
            if not _solid_visibility[s1] or not _solid_visibility[s2]:
                continue
            _type = CompositeType(self._config.composite_joint_config[cj_index, Config.COMPOSITE_TYPE])
            self._composite_additions[_type](self, cj_index)

        for j_index, _ in filter(lambda x: x[1], enumerate(_joint_visibility)):
            s1, s2 = self._config.joint_config[j_index, Config.JOINT_SOLIDS]
            if not _solid_visibility[s1] or not _solid_visibility[s2]:
                # for relations
                _joint_visibility[j_index] = 0
                continue
            _type = JointType(self._config.joint_config[j_index, Config.JOINT_TYPE])
            self._joint_additions[_type](self, j_index)

        bbox = np.array([float('inf'), float('inf'), float('-inf'), float('-inf')])
        for solid, _ in filter(lambda x: x[1], enumerate(_solid_visibility)):
            solid_values = self._config.results.solid_values[solid]

            for _dic in self._solid_objects_1, self._solid_objects_2:
                for obj in _dic.get(solid, []):
                    obj.update_bbox(bbox, solid_values)

        return bbox

    @staticmethod
    def get_transform(bbox, screen_size):
        bbox_center = (bbox[0:2] + bbox[2:4]) * 0.5
        bbox_h_extent = (bbox[2:4] - bbox[0:2]) * 0.5
        screen_center = screen_size * np.array(0.5)

        scale = min((screen_center - 2 * meshes.REVOLUTE_RADIUS) / bbox_h_extent)
        translation = screen_center - scale * bbox_center * (1, -1)
        return (scale, -scale), translation

    def show(self):
        assert self._config.state >= ConfigState.KINEMATICS_OK, "Call `System.solve_kinematics` before displaying"
        pg.init()

        window = pg.display.set_mode((800, 800))
        pg.display.set_caption('Kinepy', 'Kinepy')
        _icon = pg.image.load(_icon_path).convert_alpha()
        pg.display.set_icon(_icon)

        bbox = self._prepare()
        scale, translation = self.get_transform(bbox, window.get_size())

        frame_count, frame_time = self._config.results.solid_values.shape[-1], self._config.frame_time
        if not frame_time:
            frame_time = 0.02  # 20ms frames if no time is set
        __frame_index = 0
        self._display(window, __frame_index, scale, translation)
        pg.display.flip()

        __date = time.perf_counter()
        __remaining_time = 0.0

        __running = True
        while __running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    __running = False

            n_date = time.perf_counter()
            __remaining_time += n_date - __date
            __date = n_date

            if __remaining_time >= frame_time:
                time_shift, __remaining_time = divmod(__remaining_time, frame_time)
                __frame_index = (__frame_index + int(time_shift)) % frame_count
                self._display(window, __frame_index, scale, translation)
                pg.display.flip()

        pg.quit()

    def _display(self, surface: pg.Surface, frame_index: int, scale, translation):
        surface.fill((0, 0, 0))
        for solid, obj_list in self._solid_objects_1.items():
            solid_values = self._config.results.solid_values[solid]

            for obj in obj_list:
                obj.draw(surface, solid_values, frame_index, scale, translation)

        for solid, obj_list in self._solid_objects_2.items():
            solid_values = self._config.results.solid_values[solid]

            for obj in obj_list:
                obj.draw(surface, solid_values, frame_index, scale, translation)
