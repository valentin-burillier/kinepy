import kinepy.units as units
from kinepy.interface.decorators import get_object
from kinepy.math.geometry import rot, rvec, unit


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


def gather_points(system, points, speeds, forces):
    # points is used to determine the window
    points_ = []
    for s in system.sols:
        points_.append(s.origin)
    for s, point in speeds:
        points_.append(s.origin + rvec(s.angle, point))
    for s, point, _ in forces + points:
        points_.append(s.origin + rvec(s.angle, point))

    for joint in system.joints:
        if joint.id_ not in get_points:
            continue
        p1, p2 = get_points[joint.id_](joint)
        points_.append(joint.s1.origin + rvec(joint.s1.angle, p1))
        points_.append(joint.s2.origin + rvec(joint.s2.angle, p2))
    return points_


# def get_additional_points():
#     return plotting_point_list + plotting_speed_list + [li[:2] for li in plotting_force_list]
