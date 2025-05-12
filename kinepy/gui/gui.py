import enum
import numpy as np
import os
import pygame as pg
import time
import PIL.Image as im

import kinepy.math.geometry as geo
import kinepy.objects.joints_solid as jo_so
import kinepy.objects.config as cfg
import kinepy.gui.meshes as meshes


_icon_path = os.path.join(os.path.dirname(__file__), 'logo.ico')

COLORMAP = (144, 144, 144), (61, 131, 198), (204, 0, 0), (106, 167, 79), (241, 194, 57), (227, 119, 194), (255, 127, 14), (148, 103, 189), (145, 220, 3), (26, 190, 207)


class GUIParameters:
    """
    Parameters that define how the simulation is rendered

    Attributes:
      background_color (tuple[int, int, int]): color that fills the background
      figure_size (tuple[int, int]): size in px of the presenting window
      scale (np.ndarray[(2,)]): [internal] tranformation of simulation positions to screen positions in px/m
      translation (np.ndarray[(2,)]): [internal] position of the simulation origin on the screen in px
    """
    background_color = 16, 16, 16
    figure_size = 800, 800
    scale = np.array((1, -1))
    translation = np.zeros((2,))


class _GUIObject:
    def draw(self, surface: pg.Surface, solid_position: np.ndarray, solid_orientation: np.ndarray, frame_index, param: GUIParameters, color):
        """
        Presents itself to the given surface
        
        @param surface: surface to be drawn to
        @param solid_position: positions to use, corresponds to a solid
        @param solid_orientation: orientations to use, corresponds to a solid
        @param frame_index: represents time
        @param param: gui parameters to use
        @param color: color this object should be drawn with
        """

    def update_bbox(self, bbox: np.ndarray, solid_position: np.ndarray, solid_orientation: np.ndarray):
        """
        Enlarges the total area that is needed to draw the simulation to define the view port

        @param bbox: bottom_left and top_right corners of the total area used by the simulation, modified in place
        @param solid_position: positions to use, corresponds to a solid
        @param solid_orientation: orientations to use, corresponds to a solid
        """

    @staticmethod
    def update_bbox_from_point(bbox, solid_position: np.ndarray, solid_orientation: np.ndarray, point):
        """
        Helper for _GUIObject.update_bbox, uses all positions of a point attached to a solid to update the bbox
        """
        
        # TODO: remove those magic values
        positions = solid_position + geo.Orientation.add(solid_orientation, point)

        # axis 0 is the time axis
        bbox[0:2] = np.minimum(bbox[0:2], np.nanmin(positions, axis=0))
        bbox[2:4] = np.maximum(bbox[2:4], np.nanmax(positions, axis=0))

    @staticmethod
    def point_to_screen(solid_position: np.ndarray, solid_orientation: np.ndarray, frame_index, scale, translation, point, grounded=False):
        """
        @param solid_position: positions to use, corresponds to a solid
        @param solid_orientation: orientations to use, corresponds to a solid
        @param frame_index: represents time
        """
        # TODO: pass GuiParameters instead of scale and translation

        if not grounded:
            return (solid_position[frame_index] + geo.Orientation.add(solid_orientation[frame_index], point)) * scale + translation
        else:
            # we know that orientation in (1, 0) and position is (0, 0), avoids shaking stationnary objects
            return point * scale + translation

    def compute_mounting_point(self, scale):
        """
        Use the computed scale to finish _GUIObject configuration 
        """

    def add_solid_structure(self, solid_obj_list):
        """
        Registers the _GUIObjects that represent the link from this object to the corresponding solid's origin
        """


class _Symbol(_GUIObject):
    """
    Symbol representing a linkage (Revolute, Prismatic, PinSlot) or the ground
    
    Attributes:
        point (np.ndarray): simulation point to be atttached to
        mesh (np.ndarray): symbol mesh to be drawn
        grounded (bool): whether this symbol is attached to ground
        region (int): which direction this point is best pointing at:
            South: 0
            East: 1
            West: 2
            North: 3
    """

    def __init__(self, point, mesh, mounting_point, grounded: bool):
        self.point = point
        self.mesh = mesh
        self.mesh_mounting_point = mounting_point
        self.grounded = grounded
        self.region = 2 * (point[1] > point[0]) + (point[1] > -point[0])
        self._solid_structure = _SolidStructure()

    def update_bbox(self, bbox, solid_position: np.ndarray, solid_orientation: np.ndarray):
        self.update_bbox_from_point(bbox, solid_position, solid_orientation, self.point)

    def draw(self, surface: pg.Surface, solid_position: np.ndarray, solid_orientation: np.ndarray, frame_index, param: GUIParameters, color):
        point = self.point_to_screen(solid_position, solid_orientation, frame_index, param.scale, param.translation, self.point, self.grounded)
        mesh = geo.Orientation.sub(self.mesh, solid_orientation[frame_index]) + point
        pg.draw.polygon(surface, param.background_color, mesh, 0)
        pg.draw.polygon(surface, color, mesh, 3)

    @classmethod
    def from_revolute(cls, r_index, config: cfg.Config, mesh, mounting_point):
        s1, s2 = config.joints.solids[r_index]
        point = config.joints.revolute_p2[r_index]

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
    def from_prismatic(cls, p_index, config: cfg.Config, mesh, mounting_point):
        s1, s2 = config.joints.solids[p_index]
        angle, dist = config.joints.prismatic_angle2[p_index],config.joints.prismatic_distance2[p_index]
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

    def draw(self, surface: pg.Surface, solid_position: np.ndarray, solid_orientation: np.ndarray, frame_index, param: GUIParameters, color):
        point = self.point_to_screen(solid_position, solid_orientation, frame_index, param.scale, param.translation, self.point, self.grounded)
        pg.draw.circle(surface, param.background_color, point, meshes.REVOLUTE_RADIUS)
        pg.draw.circle(surface, color, point, meshes.REVOLUTE_RADIUS, 3)

    def compute_mounting_point(self, scale):
        self._solid_structure.update(self.point + np.array((0, meshes.REVOLUTE_RADIUS)) / scale * self.grounded, self.grounded)
        if self.distant_relative is not None and self.distant_relative.grounded:
            self.distant_relative.point += np.array((0, meshes.REVOLUTE_RADIUS)) / scale


class _Point(_Symbol):

    def draw(self, surface: pg.Surface, solid_position: np.ndarray, solid_orientation: np.ndarray, frame_index, param: GUIParameters, color):
        point = self.point_to_screen(solid_position, solid_orientation, frame_index, param.scale, param.translation, self.point, self.grounded)
        pg.draw.circle(surface, param.background_color, point, meshes.REVOLUTE_RADIUS * 0.5)
        pg.draw.circle(surface, color, point, meshes.REVOLUTE_RADIUS * 0.5, 3)

    def compute_mounting_point(self, scale):
        self._solid_structure.update(self.point + np.array((0, meshes.REVOLUTE_RADIUS)) / scale * self.grounded, self.grounded)


class _Sliding(_GUIObject):
    def __init__(self, start_point, end_point, grounded):
        self.start_point, self.end_point = start_point, end_point
        self.grounded = grounded
        self._solid_structure = _SolidStructure()

    def update_bbox(self, bbox, solid_position: np.ndarray, solid_orientation: np.ndarray):
        self.update_bbox_from_point(bbox, solid_position, solid_orientation, self.start_point)
        self.update_bbox_from_point(bbox, solid_position, solid_orientation, self.end_point)

    def draw(self, surface: pg.Surface, solid_position: np.ndarray, solid_orientation: np.ndarray, frame_index, param: GUIParameters, color):
        start = self.point_to_screen(solid_position, solid_orientation, frame_index, param.scale, param.translation, self.start_point, self.grounded)
        end = self.point_to_screen(solid_position, solid_orientation, frame_index, param.scale, param.translation, self.end_point, self.grounded)

        pg.draw.line(surface, color, start, end, 3)

    @classmethod
    def from_prismatic(cls, p_index, config: cfg.Config):
        s1, s2 = config.joints.solids[p_index]
        angle, dist = config.joints.prismatic_angle1[p_index],config.joints.prismatic_distance1[p_index]
        point = geo.Orientation.from_angle(np.array(angle + np.pi * 0.5)) * dist
        v_dir = geo.Orientation.from_angle(np.array(angle))

        sliding = jo_so.Joint(config, p_index)._get_value()
        return cls(point + np.nanmin(sliding) * v_dir, point + np.nanmax(sliding) * v_dir, not s1)

    def add_solid_structure(self, solid_obj_list):
        solid_obj_list.append(self._solid_structure)

    def compute_mounting_point(self, scale):
        mid = (self.start_point + self.end_point) * 0.5
        self._solid_structure.update(mid, self.grounded)


class _Trace(_GUIObject):
    def __init__(self, value):
        self.value = value

    def draw(self, surface: pg.Surface, solid_position: np.ndarray, solid_orientation: np.ndarray, frame_index, param: GUIParameters, color):
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

    def draw(self, surface: pg.Surface, solid_position: np.ndarray, solid_orientation: np.ndarray, frame_index, param: GUIParameters, color):
        if self.grounded:
            for line in meshes.GROUND.reshape((meshes.GROUND.shape[0] // 2, 2, 2)):
                pg.draw.lines(surface, color, False, self.point * param.scale + param.translation + line, 2)
            return
        if np.all(np.abs(self.points) < 1e-2):
            return
        line = (solid_position[frame_index] + geo.Orientation.add(self.points, solid_orientation[frame_index])) * param.scale + param.translation
        pg.draw.lines(surface, color, False, line, 3)

    @classmethod
    def from_revolute(cls, r_index, config: cfg.Config):
        s1, s2 = config.joints.solids[r_index]
        point = config.joints.revolute_p1[r_index]

        s = cls()
        s.update(point, not s1)
        return s


class GUIState(enum.Enum):
    STOPPED, RUNNING, PAUSED = range(3)


class KeyState:
    def __init__(self):
        self.down: int = 0
        self.date: float = 0.0

    LONG_PRESS_TIME = 0.200

    def long_press(self, date):
        return self.down and date - self.date > KeyState.LONG_PRESS_TIME
    

class GUI:
    def __init__(self, config: cfg.Config):
        self._config = config

        # layer 1: ground markers, tree branches, sliders; layer 2: joint symbols, single point symbols
        self._solid_objects: dict[int, tuple[list[_GUIObject], list[_GUIObject]]] = {}

        self._wild_points = []
        self._params = GUIParameters()

    def add_solid_point(self, solid: jo_so.SolidBase, point, trace=True):
        self._wild_points.append((solid._index, np.array(point), trace))

    def _do_nothing(self, index: int):
        pass

    def _add_pin_slot(self, index: int):
        j1 = self._config.composite_joints.first_ghost_joint[index]
        j2 = j1 + cfg.Composite.Type(self._config.composite_joints.type_[index]).ghost_count
        s1, s2 = self._config.joints.s1[j1], self._config.joints.s2[j2]

        self._solid_objects[s1][0].append(s := _Sliding.from_prismatic(j1, self._config))
        s.add_solid_structure(self._solid_objects[s1][0])
        self._solid_objects[s2][1].append(s := _Symbol.from_revolute(j2, self._config, meshes.PIN_SLOT, meshes.PIN_SLOT_MOUNTING_POINT))
        s.add_solid_structure(self._solid_objects[s2][0])

    def _add_revolute(self, index: int):
        s1, s2 = self._config.joints.solids[index]

        self._solid_objects[s1][0].append(struct := _SolidStructure.from_revolute(index, self._config))
        self._solid_objects[s2][1].append(symbol := _Symbol.from_revolute(index, self._config, None, meshes.REVOLUTE_MOUNTING_POINT))
        symbol.add_solid_structure(self._solid_objects[s2][0])
        symbol.distant_relative = struct

    def _add_prismatic(self, index: int):
        s1, s2 = self._config.joints.solids[index]
        self._solid_objects[s1][0].append(s := _Sliding.from_prismatic(index, self._config))
        s.add_solid_structure(self._solid_objects[s1][0])
        self._solid_objects[s2][1].append(s := _Symbol.from_prismatic(index, self._config, meshes.PRISMATIC, meshes.PRISMATIC_MOUNTING_POINT))
        s.add_solid_structure(self._solid_objects[s2][0])

    """
    Callbacks that add every drawing element corresponding to a CompositeJoint 
    """
    _composite_additions = {
        cfg.Composite.Type.PIN_SLOT: _add_pin_slot,
        cfg.Composite.Type.TRANSLATION: _do_nothing,
        cfg.Composite.Type.J3DOF: _do_nothing
    }

    """
    Callbacks that add every drawing element corresponding to a PrimitiveJoint 
    """
    _joint_additions = {
       cfg.Joints.Type.REVOLUTE: _add_revolute,
       cfg.Joints.Type.PRISMATIC: _add_prismatic
    }

    def _prepare(self, win_size):
        self._solid_objects.clear()

        _solid_visibility = np.array(self._config.solids.is_ghost == 0)
        _solid_visibility[0] = True # Ground might be a ghost, it is "visible"
        _joint_visibility = np.array(self._config.joints.type_ < cfg.Joints.Type.PRIMITIVE_SEPARATOR)
        _composite_joint_visibility = [True] * self._config.composite_joints.count
        # TODO: add relation visibility
        # TODO: add force visibility

        # create empty object layers for each visible solid
        for solid in np.arange(self._config.solids.count)[_solid_visibility]:
            self._solid_objects[solid] = [], []

        for cj_index in np.arange(self._config.composite_joints.count)[_composite_joint_visibility]:
            j1 = self._config.composite_joints.first_ghost_joint[cj_index]
            j2 = j1 + (type_ := cfg.Composite.Type(self._config.composite_joints.type_[cj_index])).ghost_count
            s1, s2 = self._config.joints.s1[j1], self._config.joints.s2[j2]

            if not _solid_visibility[s1] or not _solid_visibility[s2]:
                # any invisible solid completely hides the joint
                continue
            self._composite_additions[type_](self, cj_index)

        for j_index in np.arange(self._config.joints.count)[_joint_visibility]:
            s1, s2 = self._config.joints.solids[j_index]
            if not _solid_visibility[s1] or not _solid_visibility[s2]:
                # any invisible solid completely hides the joint
                _joint_visibility[j_index] = 0
                continue
            _type = cfg.Joints.Type(self._config.joints.type_[j_index])
            self._joint_additions[_type](self, j_index)

        for solid, point, trace in self._wild_points:
            if not _solid_visibility[solid]:
                continue
            self._solid_objects[solid][1].append(symbol := _Point(point, None, None, not solid))
            symbol.add_solid_structure(self._solid_objects[solid][0])
            if trace:
                # TODO: move this to a background layer
                self._solid_objects[solid][0].append(_Trace(geo.Position.point(self._config, solid, point)))

        # compute total region occupied by rendered elements
        bbox = np.array([float('inf'), float('inf'), float('-inf'), float('-inf')])
        for solid, layers in self._solid_objects.items():
            solid_position = self._config.solids.position[solid]
            solid_orientation = self._config.solids.orientation[solid]

            for obj_l in layers:
                for obj in obj_l:
                    obj.update_bbox(bbox, solid_position, solid_orientation)

        scale, translation = self._get_transform(bbox, win_size)

        # place fixed-size object (symbols)
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

        scale = 1 / max(bbox_h_extent / (screen_center - 2 * meshes.REVOLUTE_RADIUS))
        translation = screen_center - scale * bbox_center * (1, -1)
        return (scale, -scale), translation

    def show(self):
        assert self._config.state >= cfg.ConfigState.KINEMATICS_OK, "Call `System.solve_kinematics` before displaying"
        pg.init()

        window = pg.display.set_mode(self._params.figure_size)
        pg.display.set_caption('Kinepy', 'Kinepy')
        _icon = pg.image.load(_icon_path).convert_alpha()
        pg.display.set_icon(_icon)

        self._prepare(window.get_size())

        frame_count, frame_time = self._config.frame_count, self._config.frame_time
        if not frame_time:
            frame_time = 0.02  # 20ms frames if no time is set
        __frame_index = 0
        self._display(window, __frame_index)
        pg.display.flip()

        __date = time.perf_counter()
        
        __sim_remaining_time = 0.0
        __pause_remaining_time = 0.0

        __key_states = {
            pg.K_LEFT: KeyState(),
            pg.K_RIGHT: KeyState()
        }

        __state = GUIState.RUNNING
        while __state != GUIState.STOPPED:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    __state = GUIState.STOPPED
                elif event.type == pg.KEYDOWN:
                    if event.key == pg.K_SPACE:
                        __date = time.perf_counter()
                        if __state == GUIState.RUNNING:
                            __pause_remaining_time = 0.0
                            __state = GUIState.PAUSED
                        elif __state == GUIState.PAUSED:
                            __state = GUIState.RUNNING
                    if event.key in __key_states:
                        __key_states[event.key].down = 1
                        __key_states[event.key].date = time.perf_counter()
                    if __state == GUIState.PAUSED:
                        if event.key not in (pg.K_LEFT, pg.K_RIGHT):
                            continue
                        # first press changes right away
                        __frame_index = (__frame_index + (event.key == pg.K_RIGHT) - (event.key == pg.K_LEFT)) % frame_count
                        self._display(window, __frame_index)
                        pg.display.flip()
                elif event.type == pg.KEYUP and event.key in __key_states:
                    __key_states[event.key].down = 0

            if __state == GUIState.PAUSED:
                n_date = time.perf_counter()
                comp_date = __date
                __date = n_date
                if not (__key_states[pg.K_LEFT].long_press(n_date) ^ __key_states[pg.K_RIGHT].long_press(n_date)):
                    __pause_remaining_time = 0
                    continue
                if __key_states[pg.K_LEFT].long_press(n_date):
                    comp_date = max(__key_states[pg.K_LEFT].date + KeyState.LONG_PRESS_TIME, comp_date)
                if __key_states[pg.K_RIGHT].long_press(n_date):
                    comp_date = max(__key_states[pg.K_RIGHT].date + KeyState.LONG_PRESS_TIME, comp_date)
                __pause_remaining_time += n_date - comp_date

                if __pause_remaining_time < 2 * frame_time:
                    continue
                time_shift, __pause_remaining_time = divmod(__pause_remaining_time, 2 * frame_time)
                __frame_index = (__frame_index + int(time_shift) * (__key_states[pg.K_RIGHT].down - __key_states[pg.K_LEFT].down)) % frame_count
                self._display(window, __frame_index)
                pg.display.flip()
                continue

            n_date = time.perf_counter()
            __sim_remaining_time += n_date - __date
            __date = n_date

            if __sim_remaining_time < frame_time:
                continue
            time_shift, __sim_remaining_time = divmod(__sim_remaining_time, frame_time)
            __frame_index = (__frame_index + int(time_shift)) % frame_count
            self._display(window, __frame_index)
            pg.display.flip()

        pg.quit()

    def save(self, file_name: str):
        assert self._config.state >= cfg.ConfigState.KINEMATICS_OK, "Call `System.solve_kinematics` before saving"
        assert file_name.endswith('.gif'), 'Only gif files are supported'
        surface = pg.Surface(self._params.figure_size)
        self._prepare(surface.get_size())

        video = []

        for frame_index in range(self._config.frame_count):
            self._display(surface, frame_index)
            img_string = pg.image.tostring(surface, 'RGB', False)
            video.append(im.frombytes('RGB', surface.get_size(), img_string))

        base = video[0]
        base.save(file_name, fps=24, save_all=True, append_images=video[1:])

    def _display(self, surface: pg.Surface, frame_index: int):
        surface.fill(self._params.background_color)

        for solid, layers in self._solid_objects.items():
            solid_position = self._config.solids.position[solid]
            solid_orientation = self._config.solids.orientation[solid]
            color = COLORMAP[0] if not solid else COLORMAP[(solid - 1) % (len(COLORMAP) - 1) + 1]

            for obj_list in layers:
                for gui_obj in obj_list:
                    gui_obj.draw(surface, solid_position, solid_orientation, frame_index, self._params, color)

