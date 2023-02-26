from kinepy.units import *
import kinepy.objects as obj
from kinepy.interface.solid import Solid
from kinepy.interface.joints import RevoluteJoint, PrismaticJoint, PinslotJoint, RectangularJoint, J3DOF
from kinepy.interface.decorators import physics_input, add_joint, solid_checker, single_or_list, joint_checker, \
    get_object
from kinepy.interface.relations import Gear, GearRack, DistantRelation, EffortlessRelation


class System:
    def __init__(self, name='Main System'):
        self._object = obj.System([], [], [], [], [], [])
        self.name = name
        self._unit_system = UnitSystem()
        self.named_joints, self.named_sols = {}, {}
        self.ground = self.add_solid(0., 0., (0., 0.), 'Ground')

    @physics_input(MASS, INERTIA, LENGTH, '')
    def add_solid(self, m=0., j=0., g=(0., 0.), name=''):
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
        s = Solid(self._unit_system, m, j, g, name)  # noqa
        self._object.sols.append(get_object(s))
        # print(s._object, id(s._object))
        self.named_sols[name] = s
        self._object.interactions.append(s.external_actions)
        return s

    @solid_checker
    @physics_input('', '', LENGTH, LENGTH)
    def add_revolute(self, s1, s2, p1=(0., 0.), p2=(0., 0.)):
        """
        Adds a revolute joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object refering to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object refering to the second solid concerned by the joint
         - p1, tuple[float, float]: local position of the joint in s1
         - p2, tuple[float, float]: local position of the joint in s2

        Returns: RevoluteJoint
        """
        return add_joint(self, RevoluteJoint, s1, s2, p1, p2)

    @solid_checker
    @physics_input('', '', ANGLE, LENGTH, ANGLE, LENGTH)
    def add_prismatic(self, s1, s2, a1=0., d1=0., a2=0., d2=0.):
        """
        Adds a prismatic joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object refering to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object refering to the second solid concerned by the joint
         - a1, float: local direction of the axis of the joint in s1
         - a2, float: local direction of the axis of the joint in s2
         - d1, float: algebraic distance from the reference of s1 to the axis of the joint
         - d2, float: algebraic distance from the reference of s2 to the axis of the joint

        Returns: PrismaticJoint
        """
        return add_joint(self, PrismaticJoint, s1, s2, a1, d1, a2, d2)

    @solid_checker
    @physics_input('', '', ANGLE, LENGTH, LENGTH)
    def add_pin_slot(self, s1, s2, a1=0., d1=0., p2=(0., 0.)):
        """
        Adds a pin slot joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object refering to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object refering to the second solid concerned by the joint
         - a1, float: local direction of the axis of the joint in s1
         - d1, float: algebraic distance from the reference of s1 to the axis of the joint
         - p2, tuple[float, float]: local position of the joint in s2

        Returns: PinSlotJoint
        """
        return add_joint(self, PinslotJoint, s1, s2, a1, d1, p2)

    @solid_checker
    @physics_input('', '', ANGLE, ANGLE, LENGTH, LENGTH)
    def add_rectangle(self, s1, s2, angle=0., a1=0., a2=np.pi * .5, p1=(0., 0.), p2=(0., 0.)):
        """
        Adds a rectangle joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object refering to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object refering to the second solid concerned by the joint
         - a1, float: local direction of the axis of the joint in s1
         - a2, float: local direction of the axis of the joint in s2
         - p1, tuple[float, float]: local position of the anchor of joint in s1
         - p2, tuple[float, float]: local position of the anchor of joint in s2

        Returns: RectangularJoint
        """
        return add_joint(self, RectangularJoint, s1, s2, angle, a1, a2, p1, p2)

    @solid_checker
    @physics_input('', '', LENGTH, LENGTH)
    def add_3dof(self, s1, s2, p1=(0., 0.), p2=(0., 0.)):
        """
        Adds a 3 degrees of freedom joint to the system

        Parameters :
         - s1, int, str or Solid: index, name or object refering to the first solid concerned by the joint
         - s2, int, str or Solid: index, name or object refering to the second solid concerned by the joint
         - p1, tuple[float, float]: local position of the anchor of joint in s1
         - p2, tuple[float, float]: local position of the anchor of joint in s2

        Returns: ThreeDegreesOfFreedomJoint
        """
        return add_joint(self, J3DOF, s1, s2, p1, p2)

    @physics_input(ACCELERATION)
    def add_gravity(self, g=G):
        grav = Gravity(self._unit_system, g)  # noqa
        self._object.interactions.append(grav)
        return grav

    @solid_checker
    @physics_input('', '', SPRING_CONSTANT, LENGTH, LENGTH, LENGTH)
    def add_spring(self, s1, s2, k, l0, p1=(0., 0.), p2=(0, 0.)):
        spr = Spring(self._unit_system, s1, s2, k, l0, p1, p2)  # noqa
        self._object.interactions.append(spr)
        return spr

    @joint_checker
    @physics_input('', '', ADIMENSIONNED, ANGLE, ANGLE)
    def add_gear(self, rev1, rev2, r, v0=0., pressure_angle=np.pi / 9):
        if not isinstance(rev1, obj.RevoluteJoint):
            raise TypeError('Joints must be RevoluteJoints')
        if not isinstance(rev2, obj.RevoluteJoint):
            raise TypeError('Joints must be RevoluteJoints')
        g = Gear(self._unit_system, rev1, rev2, r, v0, pressure_angle) # noqa
        self._object.relations.append(get_object(g))
        return g

    @joint_checker
    @physics_input('', '', LENGTH, LENGTH)
    def add_gearrack(self, rev, pri, r, v0=0., pressure_angle=np.pi / 9):
        if not isinstance(rev, obj.RevoluteJoint):
            raise TypeError('Joint 1 must be a RevoluteJoint')
        if not isinstance(pri, obj.PrismaticJoint):
            raise TypeError('Joint 2 must be a PrismaticJoint')
        gr = GearRack(self, rev, pri, r, v0, pressure_angle)  # noqa
        self._object.relations.append(get_object(gr))
        return gr

    @joint_checker
    def add_effortless_relation(self, j1, j2, r, v0=0.):
        if j1.dof != 1 or j2.dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')
        er = EffortlessRelation(self, j1, j2, r, v0)  # noqa
        self._object.relations.append(get_object(er))
        return er

    @joint_checker
    def add_distant_relation(self, j1, j2, r, v0=0.):
        if j1.dof != 1 or j2.dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')
        dr = DistantRelation(self, j1, j2, r, v0)  # noqa
        self._object.relations.append(get_object(dr))
        return dr

    def show_input(self):
        """
        prints the current order of the inputs
        """
        print('Current input order:')
        print(f"({'; '.join(m for joint in self._object.piloted for m in joint.input_mode())})")

    @single_or_list(show_input)
    def pilot(self, joint):
        self._object.piloted.append(joint)

    @single_or_list()
    def block(self, joint):
        self._object.blocked.append(joint)

    def compile(self):
        self._object.compile()

    def solve_kinematics(self, inputs=None):
        if inputs is None:
            return self._object.solve_kinematics(self._object.inputs)
        inputs = np.array(inputs)
        if inputs.ndim == 1:
            inputs = inputs[np.newaxis, :]
        for vec, phy in zip(inputs, (p for joint in self._object.piloted for p in joint.inputs)):
            vec *= self._unit_system[phy]
        self._object.solve_kinematics(inputs)

    def solve_statics(self, compute_kine=True, inputs=None):
        if compute_kine:
            self.solve_kinematics(inputs)
        self._object.solve_statics()

    def solve_dynamics(self, t, compute_kine=True, inputs=None):
        t /= self._unit_system[TIME]
        if compute_kine:
            self.solve_kinematics(inputs)
        self._object.solve_dynamics(t)
    
    def bill_of_materials(self):
        print('N°\t| Names')
        print('----+------')
        for i, s in enumerate(self._object.sols):
            print(f'{i}\t| {s}')
