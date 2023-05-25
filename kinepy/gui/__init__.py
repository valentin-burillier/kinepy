import kinepy.gui.gui_class as _g
import kinepy.units as _units
from PIL.Image import frombytes as _from_bytes
from kinepy.interface.decorators import physics_input_function as _phy, get_object as _get_obj


_GUI_VARIABLES = {
    'frames_of_reference': False,
    'grid': False,
    'graduations': False,
    'figure_size': (640, 480),
    'background_color': (0, 0, 0),
    'animation_time': 5.,
    'system': None,
    'points': [],
    'speeds': [],
    'forces': [],
    'torques': [],
    'joint_efforts': []
}


def _reset():
    for key, var in {
        'frames_of_reference': False,
        'grid': False,
        'graduations': False,
        'figure_size': (640, 480),
        'background_color': (0, 0, 0),
        'animation_time': 5.,
        'system': None,
        'points': [],
        'speeds': [],
        'forces': [],
        'torques': [],
        'joint_efforts': []
    }.items():
        _GUI_VARIABLES[key] = var


def system(s):
    """sets the system to be displayed"""
    _GUI_VARIABLES['system'] = s


def add_point(solid, point, trace=True, speed=False):
    """Adds the point to the plotting list"""
    _GUI_VARIABLES['points'].append((_get_obj(solid), point * _units.SYSTEM[_units.LENGTH], trace))
    if not speed:
        return
    _GUI_VARIABLES['speeds'].append((_get_obj(solid), point * _units.SYSTEM[_units.LENGTH]))


def add_force(solid, force, point):
    """Behaves just like Solid.add_force but adds the point and the force to the plotting list"""
    solid.add_force(force, point)
    f, _, p = solid.external_actions.external[-1]
    _GUI_VARIABLES['forces'].append((_get_obj(solid), p, f))


def add_joint_effort(joint, reverse=False):
    _GUI_VARIABLES['joint_efforts'].append((_get_obj(joint), reverse))


def add_torque(solid, torque):
    """Behaves just like Solid.add_torque but adds the torque to the plotting list"""
    solid.add_torque(torque)
    _, t, _ = solid.external_actions.external[-1]
    _GUI_VARIABLES['torques'].append((_get_obj(solid), t))


def grid(value=True):
    """Set the visibility of the grid: True by default"""
    _GUI_VARIABLES['grid'] = value


def graduation(value=True):
    """Sets the presence of graduations: True by default"""
    _GUI_VARIABLES['graduations'] = value


def frames_of_reference(value=True):
    """Sets the visibility of solid frames of reference: True by default"""
    _GUI_VARIABLES['frames_of_reference'] = value


def figure_size(value=(640, 480)):
    """Sets the size in pixels of the drawing area"""
    _GUI_VARIABLES['figure_size'] = value


@_phy(_units.TIME)
def animation_time(value=1.):
    """sets the duration of the animation in seconds"""
    _GUI_VARIABLES['animation_time'] = value


def light_mode():
    _GUI_VARIABLES['background_color'] = (255, 255, 255)


def dark_mode():
    _GUI_VARIABLES['background_color'] = (0, 0, 0)


def show():
    if _GUI_VARIABLES['system'] is None:
        raise Exception('No system was set to be displayed')
    _g.pg.init()
    _g.GUI(**_GUI_VARIABLES).main_loop()
    _g.pg.quit()
    _reset()


def save(file_name: str):
    if _GUI_VARIABLES['system'] is None:
        raise Exception('No system was set to be displayed')
    if not file_name.endswith('.gif'):
        raise ValueError('Kinepy can only save files as gif, ".gif" has to be the extension')

    images = []
    _g.pg.init()
    gui = _g.GUI(**_GUI_VARIABLES, saving=True)

    w, h = gui.surface.get_size()
    size = w, h - 50
    while gui.camera.animation_state < gui.camera.system.n:
        gui.draw()
        images.append(_from_bytes('RGB', size, _g.pg.image.tostring(gui.surface.subsurface((0, 0) + size), 'RGB', False)))
        gui.camera.animation_state += gui.camera.animation_speed

    # print('duration', 1000. / _g.FPS)
    images[0].save(file_name, append_images=images[1:], save_all=True, optimize=False, duration=1000. / _g.FPS, loop=0)
    _reset()
