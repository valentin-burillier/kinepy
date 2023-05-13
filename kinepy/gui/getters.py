import kinepy.units as units
from kinepy.interface.decorators import get_object
from kinepy.math.geometry import rot, rvec, unit

plotting_point_list = []
plotting_speed_list = []
plotting_force_list = []
plotting_torque_list = []


def gui_add_point(solid, point):
    """Adds the point to the plotting list"""
    plotting_point_list.append((get_object(solid), point * units.SYSTEM[units.LENGTH]))


def gui_add_speed(solid, point):
    """Adds the point to the plotting list to display the speed of this point"""
    plotting_speed_list.append((get_object(solid), point * units.SYSTEM[units.LENGTH]))


def gui_add_force(solid, force, point):
    """Behaves just like Solid.add_force but adds the point and the force to the plotting list"""
    solid.add_force(force, point)
    f, _, p = solid.external_actions.external[-1]
    plotting_force_list.append((get_object(solid), p, f))


def gui_add_torque(solid, torque):
    """Behaves just like Solid.add_torque but adds the torque to the plotting list"""
    solid.add_torque(torque)
    _, t, _ = solid.external_actions.external[-1]
    plotting_torque_list.append((get_object(solid), t))


def rev_get_points(rev):
    return rev.p1, rev.p2


def pri_get_points(pri):
    return rot(pri.a1) @ (.5 * (min(pri.sliding) + max(pri.sliding)), pri.d1), rot(pri.a2) @ (0., pri.d2)


def pin_slot_get_points(pin):
    return rot(pin.a1) @ (.5 * (min(pin.sliding) + max(pin.sliding)), pin.d1), pin.p2


get_points = {
    1: rev_get_points,
    2: pri_get_points,
    3: pin_slot_get_points
}


def gather_points(system):
    # points is used to determine the window
    points = []
    for s in system.sols:
        points.append(s.origin)
    for s, point in plotting_point_list + plotting_speed_list:
        points.append(s.origin + rvec(s.angle, point))
    for s, point, _ in plotting_force_list:
        points.append(s.origin + rvec(s.angle, point))

    for joint in system.joints:
        if joint.id_ not in get_points:
            continue
        p1, p2 = get_points[joint.id_](joint)
        points.append(joint.s1.origin + rvec(joint.s1.angle, p1))
        points.append(joint.s2.origin + rvec(joint.s2.angle, p2))
    return points


def get_additional_points():
    return plotting_point_list + plotting_speed_list + [li[:2] for li in plotting_force_list]
