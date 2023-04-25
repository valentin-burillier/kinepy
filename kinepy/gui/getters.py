import kinepy.units as units
from kinepy.interface.decorators import get_object

plotting_point_list = []
plotting_force_list = []
plotting_torque_list = []


def gui_get_point(solid, point):
    """Behaves just like Solid.get_point but add the point to the plotting list"""
    plotting_point_list.append((get_object(solid), point * units.SYSTEM[units.LENGTH]))
    return solid.get_point(point)


def gui_add_force(solid, force, point):
    """Behaves just like Solid.add_force but adds the point and the force to the plotting list"""
    solid.add_force(force, point)
    f, _, p = solid.external_actions.external[-1]
    plotting_force_list.append((get_object(solid), p, f))


def gui_add_torque(solid, torque):
    """Behaves just like Solid.add_torque but adds the torque to the plotting list"""
    solid.add_torque(torque)
    _, t, p = solid.external_actions.external[-1]
    plotting_torque_list.append((get_object(solid), t))
