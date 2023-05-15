import kinepy.gui.gui_class as _g


_GUI_VARIABLES = {
    'frames_of_reference': False,
    'grid': False,
    'graduations': False,
    'figure_size': (640, 480),
    'background_color': (0, 0, 0),
    'animation_time': 5.,
    'system': None
}


def system(s):
    """sets the system to be displayed"""
    _GUI_VARIABLES['system'] = s


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


def animation_time(value=1.):
    """sets the duration of the animation in seconds"""
    _GUI_VARIABLES['animation_time'] = value


def light_mode():
    _GUI_VARIABLES['background_color'] = (255, 255, 255)


def dark_mode():
    _GUI_VARIABLES['background_color'] = (0, 0, 0)


def show():
    _g.pg.init()
    _g.GUI(**_GUI_VARIABLES).main_loop()
    _g.pg.quit()