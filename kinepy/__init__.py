from kinepy.solid import *
from kinepy.linkage import Joint, RevoluteJoint, PrismaticJoint, PinSlotJoint, RectangularJoint, J3DOF
from kinepy.interactions import Gravity, Spring
from kinepy.metajoints import Gear, GearRack, DistantRelation, EffortlessRelation
from kinepy.compilation import make_sets, compiler, KINEMATICS, DYNAMICS, BOTH
from kinepy.kinematic import KIN
from kinepy.dynamic import DYN


def solid_checker(f):
    def g(self, s1, s2, *args, **kwargs):
        if isinstance(s1, str):
            s1 = self.named_sols[s1]
        if isinstance(s1, int):
            s1 = self.sols[s1]
        if isinstance(s2, str):
            s2 = self.named_sols[s2]
        if isinstance(s2, int):
            s2 = self.sols[s2]
        return f(self, s1, s2, *args, **kwargs)
    g.__doc__ = f.__doc__
    return g


def joint_checker(f):
    def g(self, j1, j2, *args, **kwargs):
        if isinstance(j1, str):
            j1 = self.named_joints[j1]
        if isinstance(j1, int):
            j1 = self.joints[j1]
        if isinstance(j1, str):
            j2 = self.named_joints[j2]
        if isinstance(j1, int):
            j2 = self.joints[j2]
        return f(self, j1, j2, *args, **kwargs)
    g.__doc__ = f.__doc__
    return g


def single_or_list(post_call=None):
    def decor(f):
        def g(self, *args):
            for arg in args:
                if isinstance(arg, (tuple, list)):
                    for a in arg:
                        f(self, a)
                else:
                    f(self, arg)
            if post_call is not None:
                post_call(self)
        g.__doc__ = f.__doc__
        return g
    return decor


class System:
    """
    Planar mechanism model
    """
    inputs = dt = None
    tot = 0

    def __init__(self, name='Main system'):
        """
        Initialises a System

        Parameters:
         - name: str, name of the system, 'Main system' by default
        """
        self.name = name
        self._unit_system = UnitSystem()
        self.sols = [Solid(self._unit_system, 0, 0., 0., (0., 0.), 'Ground')]
        self.named_sols = {'Ground': 0}
        self.named_joints, self.indices, self.signs, self.tags = {}, {}, {}, {}
        self.kin_instr, self.dyn_instr, self.interactions, self.relations, self.joints, self.blocked, self.piloted = \
            [], [], [], [], [], [], []
        self.kin_sols = self.kin_joints = self.dyn_sols = self.dyn_joints = self.kin_ghosted = self.dyn_ghosted = None

    def add_solid(self, name='', m=0., j=0., g=(0., 0.)):
        """
        Adds a solid to the system

        Parameters:
         - name, str: name of the solid, generic name of type 'Solid {i}' by default
         - m, float: mass of the solid, 0 by default
         - j, float: moment of inertia, 0 by default
         - g, tuple[float, float]: local position of the center of gravity, (0., 0.) by default

        Returns: Solid
        """
        s = Solid(self._unit_system, len(self.sols), j, m, g, name)
        self.named_sols[s.name] = s.rep
        self.sols.append(s)
        return s

    @solid_checker
    def add_revolute(self, s1, s2, p1=(0., 0.), p2=(0., 0.)):
        """
        Adds a revolute joint to the system

        Parameters:
         - s1, int, str or Solid: index, name or object refering to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object refering to the second solid concerned by the joint
         - p1, tuple[float, float]: local position of the joint in s1
         - p2, tuple[float, float]: local position of the joint in s2

        Returns: RevoluteJoint
        """
        p = RevoluteJoint(self._unit_system, len(self.joints), s1, s2, p1, p2)
        print(f'Added linkage {p}')
        self.named_joints[p.name] = p.rep
        self.joints.append(p)
        self.interactions.append(p.interaction)
        return p

    @solid_checker
    def add_prismatic(self, s1, s2, a1=0., d1=0., a2=0., d2=0.):
        """
        Adds a prismatic joint to the system

        Parameters:
         - s1, int, str or Solid: index, name or object refering to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object refering to the second solid concerned by the joint
         - a1, float: local direction of the axis of the joint in s1
         - a2, float: local direction of the axis of the joint in s2
         - d1, float: algebraic distance from the reference of s1 to the axis of the joint
         - d2, float: algebraic distance from the reference of s2 to the axis of the joint

        Returns: PrismaticJoint
        """
        g = PrismaticJoint(self._unit_system, len(self.joints), s1, s2, a1, d1, a2, d2)
        print(f'Added linkage {g}')
        self.named_joints[g.name] = g.rep
        self.joints.append(g)
        self.interactions.append(g.interaction)
        return g

    @solid_checker
    def add_pin_slot(self, s1, s2, a1=0., d1=0., p2=(0., 0.)):
        sp = PinSlotJoint(self._unit_system, len(self.joints), s1, s2, a1, d1, p2)
        print(f'Added linkage {sp}')
        self.named_joints[sp.name] = sp.rep
        self.joints.append(sp)
        self.interactions.append(sp.interaction)
        return sp

    @solid_checker
    def add_rectangle(self, s1, s2, angle=0., a1=0., a2=np.pi * .5, p1=(0., 0.), p2=(0., 0.)):
        t = RectangularJoint(self._unit_system, len(self.joints), s1, s2, angle, a1, a2, p1, p2)
        print(f'Added linkage {t}')
        self.named_joints[t.name] = t.rep
        self.joints.append(t)
        return t

    @solid_checker
    def add_3dof(self, s1, s2, p1=(0., 0.), p2=(0., 0.)):
        _3dof = J3DOF(self._unit_system, len(self.joints), s1, s2, p1, p2)
        print(f'Added linkage {_3dof}')
        self.named_joints[_3dof.name] = _3dof.rep
        self.joints.append(_3dof)
        return _3dof

    def show_input(self):
        j = []
        for joint in self.piloted:
            for m in self.joints[joint].input_mode():
                j.append(m)
        print('Current input order:')
        print(f"({'; '.join(j)})")

    @single_or_list(show_input)
    def pilot(self, joint):
        if isinstance(joint, str):
            joint = self.named_joints[joint]
        if isinstance(joint, int):
            joint = self.joints[joint]

        # Mse à jour de l'entrée pour résolution cinématiue
        pil = []
        for _ in joint.input_mode():
            pil.append(self.tot)
            self.tot += 1
        self.indices[joint.rep] = tuple(pil)
        self.piloted.append(joint.rep)

    @single_or_list()
    def block(self, joint):
        if isinstance(joint, str):
            joint = self.named_joints[joint]
        if isinstance(joint, int):
            joint = self.joints[joint]
        self.blocked.append(joint.rep)

    def reset(self, n):
        for sol in self.sols:
            sol.reset(n)
        for joint in self.joints:
            joint.reset(n)

    def compile(self):
        self.signs, self.tags = {}, {}
        if not self.blocked or set(self.blocked) == set(self.piloted):
            lst, dct, g = make_sets(self, self.piloted)
            self.kin_sols = self.dyn_sols = lst
            self.kin_joints = self.dyn_joints = dct
            self.kin_ghosted = self.dyn_ghosted = g
            self.kin_instr, self.dyn_instr = compiler(self, BOTH)
        else:
            self.kin_sols, self.kin_joints, self.kin_ghosted = make_sets(self, self.piloted)
            self.dyn_sols, self.dyn_joints, self.dyn_ghosted = make_sets(self, self.blocked)
            self.kin_instr, self.dyn_instr = compiler(self), compiler(self, DYNAMICS)
        print('signs =', self.signs)

    def solve_kinematics(self, inputs=None):
        if inputs is not None:
            inputs = np.array(inputs)
            if len(inputs.shape) == 1:
                inputs = inputs[np.newaxis, :]
            self.inputs = inputs

        self.reset(self.inputs.shape[1])
        for instr in self.kin_instr:
            KIN[instr[0]](*instr[1:])

    def solve_statics(self, compute_kine=True, inputs=None):
        if compute_kine:
            self.solve_kinematics(inputs)
        for instr in self.dyn_instr[1:]:
            DYN[instr[0]](*instr[1:])

    @physics_input(TIME, '', '')
    def solve_dynamics(self, t, compute_kine=True, inputs=None):
        if compute_kine:
            self.solve_kinematics(inputs)
        self.dt = t / (self.inputs.shape[1] - 1)
        for instr in self.dyn_instr:
            DYN[instr[0]](*instr[1:])

    def add_gravity(self, g=None):
        af = Gravity(self._unit_system, self, g)
        self.interactions.append(af)
        return af

    @solid_checker
    def add_spring(self, s1, s2, k, l0, p1=(0., 0.), p2=(0, 0.)):
        spr = Spring(self._unit_system, k, l0, s1, s2, p1, p2)
        self.interactions.append(spr)
        return spr

    def __repr__(self):
        return self.name

    @joint_checker
    def add_gear(self, rev1, rev2, r, v0=0., pressure_angle=None):
        if not isinstance(rev1, RevoluteJoint):
            raise TypeError('Joints must be RevoluteJoints')
        if not isinstance(rev2, RevoluteJoint):
            raise TypeError('Joints must be RevoluteJoints')
        g = Gear(self._unit_system, rev1, rev2, r, v0, pressure_angle)
        self.relations.append(g)
        return g

    @joint_checker
    def add_gearrack(self, rev, pri, r, v0=0., pressure_angle=None):
        if not isinstance(rev, RevoluteJoint):
            raise TypeError('Joint 1 must be a RevoluteJoint')
        if not isinstance(pri, PrismaticJoint):
            raise TypeError('Joint 2 must be a PrismaticJoint')
        gr = GearRack(self, rev, pri, r, v0, pressure_angle)
        self.relations.append(gr)
        return gr

    @joint_checker
    def add_effortless_relation(self, j1, j2, r, v0=0.):
        if j1.dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')
        if j2.dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')
        er = EffortlessRelation(self, j1, j2, r, v0)
        self.relations.append(er)
        return er

    @joint_checker
    def add_distant_relation(self, j1, j2, r, v0=0.):
        if j1.dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')
        if j2.dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')
        dr = DistantRelation(self, j1, j2, r, v0)
        self.relations.append(dr)
        return dr
