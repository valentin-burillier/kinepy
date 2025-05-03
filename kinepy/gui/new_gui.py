import enum
import os

import numpy as np

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import pygame as pg
import PIL.Image as im
import kinepy.gui.new_meshes as meshes
import kinepy.math.geometry as geo
from kinepy.objects.config import Config, ConfigState
from kinepy.objects.joints_solid import CompositeType, JointType, PrimitiveJoint, Solid
import kinepy.strategy.types as strategy
import time

_icon_path = os.path.join(os.path.dirname(__file__), 'logo.ico')

COLORMAP = (
    (144, 144, 144), (61, 131, 198), (204, 0, 0), (106, 167, 79), (241, 194, 57), (227, 119, 194), (255, 127, 14),
    (148, 103, 189), (145, 220, 3), (26, 190, 207)
)


class GUIParameters:
    background_color = 16, 16, 16
    scale = np.array((1, -1))
    translation = np.zeros((2,))
    figure_size = 800, 800


class _GUIObject:
    def draw(self, surface: pg.Surface, solid_values, frame_index, param: GUIParameters, color):
        pass

    def update_bbox(self, bbox, solid_values):
        pass

    @staticmethod
    def update_bbox_from_point(bbox, solid_values, point):
        positions = solid_values[:, 0:2] + geo.Orientation.add(solid_values[:, 2:4], point)
        bbox[0:2] = np.minimum(bbox[0:2], np.nanmin(positions, axis=0))
        bbox[2:4] = np.maximum(bbox[2:4], np.nanmax(positions, axis=0))

    @staticmethod
    def point_to_screen(solid_values, frame_index, scale, translation, point, grounded=False):
        if not grounded:
            return (solid_values[frame_index, 0:2] + geo.Orientation.add(solid_values[frame_index, 2:4], point)) * scale + translation
        else:
            return point * scale + translation

    def compute_mounting_point(self, scale):
        pass

    def add_solid_structure(self, solid_obj_list):
        pass


class _Symbol(_GUIObject):
    def __init__(self, point, mesh, mounting_point, grounded: bool):
        self.point = point
        self.mesh = mesh
        self.mesh_mounting_point = mounting_point
        self.grounded = grounded
        self.region = 2 * (point[1] > point[0]) + (point[1] > -point[0])
        self._solid_structure = _SolidStructure()

    def update_bbox(self, bbox, solid_values):
        self.update_bbox_from_point(bbox, solid_values, self.point)

    def draw(self, surface: pg.Surface, solid_values, frame_index, param: GUIParameters, color):
        point = self.point_to_screen(solid_values, frame_index, param.scale, param.translation, self.point, self.grounded)
        mesh = geo.Orientation.sub(self.mesh, solid_values[frame_index, 2:4]) + point
        pg.draw.polygon(surface, param.background_color, mesh, 0)
        pg.draw.polygon(surface, color, mesh, 3)

    @classmethod
    def from_revolute(cls, r_index, config: Config, mesh, mounting_point):
        s1, s2 = config.joint_config[r_index, Config.JOINT_SOLIDS]
        point = config.joint_physics[r_index, Config.JOINT_P2]

        if mesh is None:
            return _RevoluteSymbol(point, None, None, not s2)

        if not s2:
            pass
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
            if abs(angle) > np.pi * 0.5:
                mesh = mesh * (1, -1)
        elif dist < 0:
            # point towards solid ref
            mesh = mesh * (1, -1)

        mesh = geo.Orientation.add(mesh, geo.Orientation.from_angle(angle))

        point = geo.Orientation.from_angle(np.array(angle + np.pi)) * dist
        return cls(point, mesh, mounting_point, not s2)

    def add_solid_structure(self, solid_obj_list):
        solid_obj_list.append(self._solid_structure)

    def compute_mounting_point(self, scale):
        m_point = (self.point + self.mesh[self.mesh_mounting_point] / scale)
        self._solid_structure.update(m_point, self.grounded)


class _RevoluteSymbol(_Symbol):
    distant_relative = None

    def draw(self, surface: pg.Surface, solid_values, frame_index, param: GUIParameters, color):
        point = self.point_to_screen(solid_values, frame_index, param.scale, param.translation, self.point, self.grounded)
        pg.draw.circle(surface, param.background_color, point, meshes.REVOLUTE_RADIUS)
        pg.draw.circle(surface, color, point, meshes.REVOLUTE_RADIUS, 3)

    def compute_mounting_point(self, scale):
        self._solid_structure.update(self.point + np.array((0, meshes.REVOLUTE_RADIUS)) / scale * self.grounded, self.grounded)
        if self.distant_relative is not None and self.distant_relative.grounded:
            self.distant_relative.point += np.array((0, meshes.REVOLUTE_RADIUS)) / scale


class _Point(_Symbol):

    def draw(self, surface: pg.Surface, solid_values, frame_index, param: GUIParameters, color):
        point = self.point_to_screen(solid_values, frame_index, param.scale, param.translation, self.point, self.grounded)
        pg.draw.circle(surface, param.background_color, point, meshes.REVOLUTE_RADIUS * 0.5)
        pg.draw.circle(surface, color, point, meshes.REVOLUTE_RADIUS * 0.5, 3)

    def compute_mounting_point(self, scale):
        self._solid_structure.update(self.point + np.array((0, meshes.REVOLUTE_RADIUS)) / scale * self.grounded, self.grounded)


class _Sliding(_GUIObject):
    def __init__(self, start_point, end_point, grounded):
        self.start_point, self.end_point = start_point, end_point
        self.grounded = grounded
        self._solid_structure = _SolidStructure()

    def update_bbox(self, bbox, solid_values):
        self.update_bbox_from_point(bbox, solid_values, self.start_point)
        self.update_bbox_from_point(bbox, solid_values, self.end_point)

    def draw(self, surface: pg.Surface, solid_values, frame_index, param: GUIParameters, color):
        start = self.point_to_screen(solid_values, frame_index, param.scale, param.translation, self.start_point, self.grounded)
        end = self.point_to_screen(solid_values, frame_index, param.scale, param.translation, self.end_point, self.grounded)

        pg.draw.line(surface, color, start, end, 3)

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
        return cls(point + np.nanmin(sliding) * v_dir, point + np.nanmax(sliding) * v_dir, not s1)

    def add_solid_structure(self, solid_obj_list):
        solid_obj_list.append(self._solid_structure)

    def compute_mounting_point(self, scale):
        mid = (self.start_point + self.end_point) * 0.5
        self._solid_structure.update(mid, self.grounded)


class _Trace(_GUIObject):
    def __init__(self, value):
        self.value = value.swapaxes(0, 1)

    def draw(self, surface: pg.Surface, solid_values, frame_index, param: GUIParameters, color):
        pg.draw.lines(surface, color, False, self.value * param.scale + param.translation, 1)


class _SolidStructure(_GUIObject):
    point = np.zeros((2,))
    region = -1
    grounded = False
    points = np.zeros((0, 2), float)

    def update(self, point, grounded):
        self.point = point
        region = 2 * (point[1] > point[0]) + (point[1] > -point[0])
        self.points = np.array((point, point * (region in (1, 2), region in (0, 3)), (0, 0)))
        self.grounded = grounded

    def draw(self, surface: pg.Surface, solid_values, frame_index, param: GUIParameters, color):
        if self.grounded:
            for line in meshes.GROUND.reshape((meshes.GROUND.shape[0] // 2, 2, 2)):
                pg.draw.lines(surface, color, False, self.point * param.scale + param.translation + line, 2)
            return
        if np.all(np.abs(self.points) < 1e-2):
            return
        line = (solid_values[frame_index, 0:2] + geo.Orientation.add(self.points, solid_values[frame_index, 2:4])) * param.scale + param.translation
        pg.draw.lines(surface, color, False, line, 3)

    @classmethod
    def from_revolute(cls, r_index, config: Config):
        s1, s2 = config.joint_config[r_index, Config.JOINT_SOLIDS]
        point = config.joint_physics[r_index, Config.JOINT_P1]

        s = cls()
        s.update(point, not s1)
        return s

class GUIState(enum.Enum):
    STOPPED, RUNNING, PAUSED = range(3)

class GUI:
    def __init__(self, config: Config):
        self._config = config

        # layer 1: ground markers, tree branches, sliders; layer 2: joint symbols, single point symbols
        self._solid_objects: dict[int, tuple[list[_GUIObject], list[_GUIObject]]] = {}

        self._wild_points = []
        self._params = GUIParameters()

    def add_solid_point(self, solid: Solid, point, trace=True):
        solid.check_against(self._config)
        self._wild_points.append((solid._index, np.array(point), trace))

    def _do_nothing(self, index: int):
        pass

    def _add_pin_slot(self, index: int):
        s1, s2 = self._config.get_composite_solids(index)
        p, r, _ = self._config.composite_joint_config[index, Config.COMPOSITE_JOINTS]

        self._solid_objects[s1][0].append(s := _Sliding.from_prismatic(p, self._config))
        s.add_solid_structure(self._solid_objects[s1][0])
        self._solid_objects[s2][1].append(s := _Symbol.from_revolute(r, self._config, meshes.PIN_SLOT, meshes.PIN_SLOT_MOUNTING_POINT))
        s.add_solid_structure(self._solid_objects[s2][0])

    def _add_revolute(self, index: int):
        s1, s2 = self._config.joint_config[index, Config.JOINT_SOLIDS]

        self._solid_objects[s1][0].append(struct := _SolidStructure.from_revolute(index, self._config))
        self._solid_objects[s2][1].append(symbol := _Symbol.from_revolute(index, self._config, None, meshes.REVOLUTE_MOUNTING_POINT))
        symbol.add_solid_structure(self._solid_objects[s2][0])
        symbol.distant_relative = struct

    def _add_prismatic(self, index: int):
        s1, s2 = self._config.joint_config[index, Config.JOINT_SOLIDS]
        self._solid_objects[s1][0].append(s := _Sliding.from_prismatic(index, self._config))
        s.add_solid_structure(self._solid_objects[s1][0])
        self._solid_objects[s2][1].append(s := _Symbol.from_prismatic(index, self._config, meshes.PRISMATIC, meshes.PRISMATIC_MOUNTING_POINT))
        s.add_solid_structure(self._solid_objects[s2][0])

    _composite_additions = {
        CompositeType.PIN_SLOT: _add_pin_slot,
        CompositeType.TRANSLATION: _do_nothing,
        CompositeType.J3DOF: _do_nothing
    }

    _joint_additions = {
       JointType.REVOLUTE: _add_revolute,
       JointType.PRISMATIC: _add_prismatic
    }

    def _prepare(self, win_size):
        self._solid_objects.clear()

        _solid_visibility = [1] * len(self._config.solid_names)
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
            self._solid_objects[solid] = [], []

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

        for solid, point, trace in self._wild_points:
            if not _solid_visibility[solid]:
                continue
            self._solid_objects[solid][1].append(symbol := _Point(point, None, None, not solid))
            symbol.add_solid_structure(self._solid_objects[solid][0])
            if trace:
                # TODO: move this to a background layer
                self._solid_objects[solid][0].append(_Trace(Solid(self._config, solid).get_point(point)))

        bbox = np.array([float('inf'), float('inf'), float('-inf'), float('-inf')])
        for solid, _ in filter(lambda x: x[1], enumerate(_solid_visibility)):
            solid_values = self._config.results.solid_values[solid]

            for obj_l in self._solid_objects.get(solid, ([],)):
                for obj in obj_l:
                    obj.update_bbox(bbox, solid_values)

        scale, translation = self._get_transform(bbox, win_size)

        for s_index, layers in self._solid_objects.items():
            for obj_list in layers:
                for gui_obj in obj_list:
                    gui_obj.compute_mounting_point(scale)

        self._params.scale = np.array(scale)
        self._params.translation = translation

    @staticmethod
    def _get_transform(bbox, screen_size):
        bbox_center = (bbox[0:2] + bbox[2:4]) * 0.5
        bbox_h_extent = (bbox[2:4] - bbox[0:2]) * 0.5
        screen_center = screen_size * np.array(0.5)

        scale = min((screen_center - 2 * meshes.REVOLUTE_RADIUS) / bbox_h_extent)
        translation = screen_center - scale * bbox_center * (1, -1)
        return (scale, -scale), translation

    def show(self):
        assert self._config.state >= ConfigState.KINEMATICS_OK, "Call `System.solve_kinematics` before displaying"
        pg.init()

        window = pg.display.set_mode(self._params.figure_size)
        pg.display.set_caption('Kinepy', 'Kinepy')
        _icon = pg.image.load(_icon_path).convert_alpha()
        pg.display.set_icon(_icon)

        self._prepare(window.get_size())

        frame_count, frame_time = self._config.results.solid_values.shape[1], self._config.frame_time
        if not frame_time:
            frame_time = 0.02  # 20ms frames if no time is set
        __frame_index = 0
        self._display(window, __frame_index)
        pg.display.flip()

        __date = time.perf_counter()
        __remaining_time = 0.0

        __state = GUIState.RUNNING
        while __state != GUIState.STOPPED:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    __state = GUIState.STOPPED
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_SPACE:
                        if __state == GUIState.RUNNING:
                            __state = GUIState.PAUSED
                        elif __state == GUIState.PAUSED:
                            __state = GUIState.RUNNING
                            __date = time.perf_counter()
                    elif __state == GUIState.PAUSED:
                        if event.key == pg.K_LEFT:
                            __frame_index = (__frame_index - 1) % frame_count
                            self._display(window, __frame_index)
                            pg.display.flip()
                        elif event.key == pg.K_RIGHT:
                            __frame_index = (__frame_index + 1) % frame_count
                            self._display(window, __frame_index)
                            pg.display.flip()

            if __state == GUIState.PAUSED:
                continue

            n_date = time.perf_counter()
            __remaining_time += n_date - __date
            __date = n_date

            if __remaining_time >= frame_time:
                time_shift, __remaining_time = divmod(__remaining_time, frame_time)
                __frame_index = (__frame_index + int(time_shift)) % frame_count
                self._display(window, __frame_index)
                pg.display.flip()

        pg.quit()

    def save(self, file_name: str):
        assert self._config.state >= ConfigState.KINEMATICS_OK, "Call `System.solve_kinematics` before saving"
        assert file_name.endswith('.gif'), 'Only gif files are supported'
        surface = pg.Surface(self._params.figure_size)
        self._prepare(surface.get_size())

        video = []

        for frame_index in range(self._config.results.solid_values.shape[-1]):
            self._display(surface, frame_index)
            img_string = pg.image.tostring(surface, 'RGB', False)
            video.append(im.frombytes('RGB', surface.get_size(), img_string))

        base = video[0]
        base.save(file_name, fps=24, save_all=True, append_images=video[1:])

    def _display(self, surface: pg.Surface, frame_index: int):
        surface.fill(self._params.background_color)

        for solid, layers in self._solid_objects.items():
            solid_values = self._config.results.solid_values[solid]
            color = COLORMAP[0] if not solid else COLORMAP[(solid - 1) % (len(COLORMAP) - 1) + 1]

            for obj_list in layers:
                for gui_obj in obj_list:
                    gui_obj.draw(surface, solid_values, frame_index, self._params, color)

