from kinepy.units import *
import kinepy.units as units
import kinepy.objects as obj
from kinepy.interface.solid import Solid
from kinepy.interface.joints import RevoluteJoint, PrismaticJoint, PinSlotJoint, RectangularJoint, J3DOF
from kinepy.interface.interactions import Gravity, Spring
from kinepy.interface.decorators import physics_input_method, add_joint, solid_checker, multiple_joints, joint_checker, \
    get_object
from kinepy.interface.relations import Gear, GearRack, DistantRelation, EffortlessRelation


class System:
    def __init__(self):
        self._object = obj.System([], [], [], [], [], [])
        self.named_joints, self.named_sols = {}, {}
        self.ground = self.add_solid('Ground', 0., 0., (0., 0.))
        self._compiled = False

    @physics_input_method('', MASS, INERTIA, LENGTH)
    def add_solid(self, name='', m=0., j=0., g=(0., 0.)) -> Solid:
        """
        Adds a solid to the system

        Parameters:
         - m, float: mass of the solid, 0 by default
         - j, float: moment of inertia, 0 by default
         - g, tuple[float, float]: local position of the center of gravity, (0., 0.) by default
         - name, str: name of the solid, generic name of type 'Solid{i}' by default

        Returns: Solid
        """
        name = name if name else f"Solid_{len(self._object.sols)}"
        s = Solid(m, j, g, name)  # noqa
        self._object.sols.append(get_object(s))
        # print(s._object, id(s._object))
        self.named_sols[name] = s
        self._object.interactions.append(s.external_actions)
        self._compiled = False
        return s

    @solid_checker
    @physics_input_method('', '', LENGTH, LENGTH)
    def add_revolute(self, s1, s2, p1=(0., 0.), p2=(0., 0.)) -> RevoluteJoint:
        """
        Adds a revolute joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object referring to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object referring to the second solid concerned by the joint
         - p1, tuple[float, float]: local position of the joint in s1
         - p2, tuple[float, float]: local position of the joint in s2

        Returns: RevoluteJoint
        """
        self._compiled = False
        return add_joint(self, RevoluteJoint, s1, s2, p1, p2)

    @solid_checker
    @physics_input_method('', '', ANGLE, LENGTH, ANGLE, LENGTH)
    def add_prismatic(self, s1, s2, a1=0., d1=0., a2=0., d2=0.) -> PrismaticJoint:
        """
        Adds a prismatic joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object referring to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object referring to the second solid concerned by the joint
         - a1, float: local direction of the axis of the joint in s1
         - a2, float: local direction of the axis of the joint in s2
         - d1, float: algebraic distance from the reference of s1 to the axis of the joint
         - d2, float: algebraic distance from the reference of s2 to the axis of the joint

        Returns: PrismaticJoint
        """
        self._compiled = False
        return add_joint(self, PrismaticJoint, s1, s2, a1, d1, a2, d2)

    @solid_checker
    @physics_input_method('', '', ANGLE, LENGTH, LENGTH)
    def add_pin_slot(self, s1, s2, a1=0., d1=0., p2=(0., 0.)) -> PinSlotJoint:
        """
        Adds a pin slot joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object referring to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object referring to the second solid concerned by the joint
         - a1, float: local direction of the axis of the joint in s1
         - d1, float: algebraic distance from the reference of s1 to the axis of the joint
         - p2, tuple[float, float]: local position of the joint in s2

        Returns: PinSlotJoint
        """
        self._compiled = False
        return add_joint(self, PinSlotJoint, s1, s2, a1, d1, p2)

    @solid_checker
    @physics_input_method('', '', ANGLE, ANGLE, LENGTH, LENGTH)
    def add_rectangle(self, s1, s2, angle=0., a1=0., a2=np.pi * .5, p1=(0., 0.), p2=(0., 0.)) -> RectangularJoint:
        """
        Adds a rectangle joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object referring to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object referring to the second solid concerned by the joint
         - a1, float: local direction of the axis of the joint in s1
         - a2, float: local direction of the axis of the joint in s2
         - p1, tuple[float, float]: local position of the anchor of joint in s1
         - p2, tuple[float, float]: local position of the anchor of joint in s2

        Returns: RectangularJoint
        """
        self._compiled = False
        return add_joint(self, RectangularJoint, s1, s2, angle, a1, a2, p1, p2)

    @solid_checker
    @physics_input_method('', '', LENGTH, LENGTH)
    def add_3dof(self, s1, s2, p1=(0., 0.), p2=(0., 0.)) -> J3DOF:
        """
        Adds a 3-degree of freedom joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object referring to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object referring to the second solid concerned by the joint
         - p1, tuple[float, float]: local position of the anchor of joint in s1
         - p2, tuple[float, float]: local position of the anchor of joint in s2

        Returns: ThreeDegreesOfFreedomJoint
        """
        self._compiled = False
        return add_joint(self, J3DOF, s1, s2, p1, p2)

    @physics_input_method(ACCELERATION)
    def add_gravity(self, g=(0, -G[0])):
        grav = Gravity(g)  # noqa
        self._object.interactions.append(get_object(grav))
        return grav

    @solid_checker
    @physics_input_method('', '', SPRING_CONSTANT, LENGTH, LENGTH, LENGTH)
    def add_spring(self, s1, s2, k, l0, p1=(0., 0.), p2=(0, 0.)):
        spr = Spring(s1, s2, k, l0, p1, p2)  # noqa
        self._object.interactions.append(spr)
        return spr

    @joint_checker
    @physics_input_method('', '', DIMENSIONLESS, ANGLE, ANGLE)
    def add_gear(self, rev1, rev2, r, v0=0., pressure_angle=np.pi / 9):
        if not isinstance(rev1, obj.RevoluteJoint):
            raise TypeError('Joints must be RevoluteJoints')
        if not isinstance(rev2, obj.RevoluteJoint):
            raise TypeError('Joints must be RevoluteJoints')
        g = Gear(rev1, rev2, r, v0, pressure_angle) # noqa
        self._object.relations.append(get_object(g))
        self._compiled = False
        return g

    @joint_checker
    @physics_input_method('', '', LENGTH, LENGTH, ANGLE)
    def add_gear_rack(self, rev, pri, r, v0=0., pressure_angle=np.pi / 9):
        if not isinstance(rev, obj.RevoluteJoint):
            raise TypeError('Joint 1 must be a RevoluteJoint')
        if not isinstance(pri, obj.PrismaticJoint):
            raise TypeError('Joint 2 must be a PrismaticJoint')
        gr = GearRack(rev, pri, r, v0, pressure_angle)  # noqa
        self._object.relations.append(get_object(gr))
        self._compiled = False
        return gr

    @joint_checker
    def add_effortless_relation(self, j1, j2, r, v0=0.):
        if j1.dof != 1 or j2.dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')
        er = EffortlessRelation(self, j1, j2, r, v0)  # noqa
        self._object.relations.append(get_object(er))
        self._compiled = False
        return er

    @joint_checker
    def add_distant_relation(self, j1, j2, r, v0=0.):
        if j1.dof != 1 or j2.dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')
        dr = DistantRelation(self, j1, j2, r, v0)  # noqa
        self._object.relations.append(get_object(dr))
        self._compiled = False
        return dr

    def show_input(self):
        """
        prints the current order of the inputs
        """
        print('Current input order:')
        print(f"({'; '.join(m for joint in self._object.piloted for m in joint.input_mode())})")

    @multiple_joints
    def pilot(self, *joints):
        """sets given joints as kinematic entry points for the mechanism"""
        self._object.piloted = list(joints)
        self._compiled = False
        self.show_input()

    @multiple_joints
    def block(self, *joints):
        """sets given joints as blocked"""
        self._object.blocked = list(joints)
        self._compiled = False

    def compile(self):
        if not self._compiled:
            self._object.compile()
            self._compiled = True
            string = '\n'.join(f'{key}: {value}' for key, value in self._object.signs.items())
            print(f"Current signs :\n{string}")
        
    def change_signs(self, signs):
        if isinstance(signs, (int, float)):
            signs = [signs]
        # vérif si ya que des 1 et -1
        if isinstance(signs, dict):
            if any(x not in (-1, 1, -1., 1.) for x in signs.values()):
                raise ValueError('Signs can be either 1 or -1, int or float')
            for cycle in signs.keys():
                if cycle in self._object.signs:  # sinon marche pas
                    self._object.signs[cycle] = signs[cycle]
                else:
                    print(f'\033[93mWarning : {cycle} is not recognised as a sign key\033[0m')
        else:
            if any(x not in (-1, 1, -1., 1.) for x in signs):
                raise ValueError('Signs can be either 1 or -1, int or float')
            if len(self._object.signs) != len(signs):  # sinon marche pas
                raise ValueError('Number of signs does not match')
            for i, cycle in enumerate(self._object.signs.keys()):
                self._object.signs[cycle] = signs[i]

    def solve_kinematics(self, inputs=None):
        if not self._compiled:
            self.compile()
        if inputs is None:
            return self._object.solve_kinematics(self._object.inputs)
        inputs = np.array(inputs)
        if inputs.ndim == 1:
            inputs = inputs[np.newaxis, :]
        for vec, phy in zip(inputs, (p for joint in self._object.piloted for p in joint.inputs)):
            vec *= units.SYSTEM[phy]
        self._object.solve_kinematics(inputs)

    def solve_statics(self, inputs=None, compute_kine=True):
        if not self._compiled:
            self.compile()
        if compute_kine:
            self.solve_kinematics(inputs)
        self._object.solve_statics()

    @physics_input_method('', TIME, '')
    def solve_dynamics(self, inputs=None, t=10, compute_kine=True):
        if not self._compiled:
            self.compile()
        if compute_kine:
            self.solve_kinematics(inputs)
        self._object.solve_dynamics(t)
    
    def bill_of_materials(self):
        print('N°\t| Names', '-' * 20, sep='\n')
        for i, s in enumerate(self._object.sols):
            print(f'{i}\t| {s}')


