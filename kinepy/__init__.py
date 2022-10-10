from kinepy.linkage import *
from kinepy.geometry import make_continuous, det, derivative2_vec, derivative2
from kinepy.solid import Solid
from kinepy.compilation import compiler, DYNAMICS, BOTH
from kinepy.kinematic import kin
from kinepy.dynamic import dyn
from kinepy.interactions import Spring, Gravity
from kinepy.metajoints import DistantRelation, EffortlessRelation, Gear, GearRack


#  ------------------------------------------------ Length units -------------------------------------------------------

MILLIMETER = 1e-3, 'Length'
METER = 1., 'Length'
CENTIMETER = 1e-2, 'Length'
INCH = 2.54e-2, 'Length'

#  ----------------------------------------------- Time units ----------------------------------------------------------

SECOND = 1., 'Time'
MILLISECOND = 1e-3, 'Time'
MINUTE = 60., 'Time'

# ------------------------------------------------ Angle units ---------------------------------------------------------

RADIAN = 1., 'Angle'
DEGREE = np.pi / 180, 'Angle'

# ----------------------------------------------- Acceleration units ---------------------------------------------------

METER_PER_SQUARE_SECOND = 1., 'Acceleration'
G = 9.8067, 'Acceleration'

# ---------------------------------------------- Force units -----------------------------------------------------------

NEWTON = 1., 'Force'
DECANEWTON = 10., 'Force'
MILLINEWTON = 1e-3, 'Force'

# --------------------------------------------- Mass units -------------------------------------------------------------

KILOGRAM = 1., 'Mass'
GRAM = 1e-3, 'Mass'
POUND = 2.20462, 'Mass'

# -------------------------------------------- Torque units ------------------------------------------------------------

NEWTON_METER = 1., 'Torque'
MILLINEWTON_METER = 1e-3, 'Torque'

# -------------------------------------------- Spring constant units ---------------------------------------------------

NEWTON_PER_METER = 1., 'SpringConstant'

# -------------------------------------------- Intertia units ----------------------------------------------------------

KILOGRAM_METER_SQUARED = 1., 'Intertia'


class System:
    def __init__(self, name=''):
        self.piloted, self.blocked = [], []
        self.name = name if name else 'Main system'

        # Dictionnaires des noms
        self.named_sols = {}
        self.named_joints = {}

        self.tot, self.indices = 0, {}

        # Préparation cinématique
        self.input = None
        self.signs = dict()
        self.eqs = None
        self.kin_instr, self.dyn_instr = [], []

        # Préparation dynamique
        self.interactions = []

        # Relation Liaisons
        self.relations = []

        self.units = {
            'Length': 1e-3, 'Time': 1., 'Angle': 1., 'Mass': 1., 'Force': 1., 'Inertia': 1.,
            'Acceleration': 1., 'SpringConstant': 1., 'Torque': 1.
        }
        self.sols, self.joints = [Solid(self, name='Ground')], []

    def set_unit(self, u: tuple[float, str]):
        u_, phy = u
        if phy not in self.units:
            raise ValueError(f'Unknown Physical Quantity: {phy}')
        self.units[phy] = u_

    def add_solid(self, name='', m=0., j=0., g=(0., 0.)):
        s = Solid(self, j, m, g, name, len(self.sols))
        self.named_sols[s.name] = len(self.sols)
        self.sols.append(s)
        return s
    
    def add_revolute(self, s1, s2, p1=(0., 0.), p2=(0., 0.)):
        if isinstance(s1, Solid):
            s1 = s1.rep
        if isinstance(s2, Solid):
            s2 = s2.rep
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
    
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        
        p = RevoluteJoint(self, s1, s2, p1, p2)
        print(f'Added linkage {p}')
        self.named_joints[p.name] = len(self.joints)

        self.joints.append(p)
        self.interactions.append(p.interaction)
        return p

    def add_prismatic(self, s1, s2, a1=0., d1=0., a2=0., d2=0.):
        if isinstance(s1, Solid):
            s1 = s1.rep
        if isinstance(s2, Solid):
            s2 = s2.rep
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
            
        g = PrismaticJoint(self, s1, s2, a1, d1, a2, d2)
        print(f'Added linkage {g}')
        self.named_joints[g.name] = len(self.joints)
        self.joints.append(g)
        self.interactions.append(g.interaction)
        return g
    
    def add_pin_slot(self, s1, s2, a1=0., d1=0., p2=(0., 0.)):
        if isinstance(s1, Solid):
            s1 = s1.rep
        if isinstance(s2, Solid):
            s2 = s2.rep
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
            
        sp = PinSlotJoint(self, s1, s2, a1, d1, p2)
        print(f'Added linkage {sp}')
        self.named_joints[sp.name] = len(self.joints)

        self.joints.append(sp)
        self.interactions.append(sp.interaction)
        return sp
    
    def add_rectangle(self, s1, s2, angle=0., base=(0., np.pi/2)):
        if isinstance(s1, Solid):
            s1 = s1.rep
        if isinstance(s2, Solid):
            s2 = s2.rep
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        
        t = RectangularJoint(self, s1, s2, angle, base)
        print(f'Added linkage {t}')
        self.named_joints[t.name] = len(self.joints)
        self.joints.append(t)
        return t

    def add_3dof(self, s1, s2):
        if isinstance(s1, Solid):
            s1 = s1.rep
        if isinstance(s2, Solid):
            s2 = s2.rep
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]

        _3dof = ThreeDegreesOfFreedomJoint(self, s1, s2)
        print(f'Added linkage {_3dof}')
        self.named_joints[_3dof.name] = len(self.joints)
        self.joints.append(_3dof)
        return _3dof
    
    def pilot(self, joints):
        if isinstance(joints, (tuple, list)):
            # Plusieurs liaisons en entrée
            for j in joints:
                if isinstance(j, Joint):
                    j = j.name
                if isinstance(j, str):
                    # Référence par le nom
                    j = self.named_joints[j]
                self.piloted.append(j)

                # Mse à jour de l'entrée pour résolution cinématiue
                pil = []
                for _ in self.joints[j].input_mode():
                    pil.append(self.tot)
                    self.tot += 1
                self.indices[j] = tuple(pil)

            return self.show_input()
        if isinstance(joints, Joint):
            joints = joints.name
        if isinstance(joints, str):
            # Référence par le nom
            joints = self.named_joints[joints]

        # Mse à jour de l'entrée pour résolution cinématiue
        pil = []
        for _ in self.joints[joints].input_mode():
            pil.append(self.tot)
            self.tot += 1
        self.indices[joints] = tuple(pil)

        self.piloted.append(joints)
        self.show_input()

    def block(self, joints):
        if isinstance(joints, (tuple, list)):
            for j in joints:
                if isinstance(j, Joint):
                    j = j.name
                if isinstance(j, str):
                    # Référence par le nom
                    j = self.named_joints[j]
                self.blocked.append(j)
        else:
            if isinstance(joints, Joint):
                joints = joints.name
            if isinstance(joints, str):
                # Référence par le nom
                joints = self.named_joints[joints]
            self.blocked.append(joints)

    def show_input(self):
        j = []
        for joint in self.piloted:
            for m in self.joints[joint].input_mode():
                j.append(m)
        print('Current input order:')
        print(f"({'; '.join(j)})")
    
    def reset(self, n):
        self.eqs = [(i,) for i in range(len(self.sols))]
        for sol in self.sols:
            sol.reset(n)
        for joint in self.joints:
            joint.reset(n)

    def get_origin(self, sol):
        return self.sols[sol].origin_
    
    def get_ref(self, sol):
        return self.sols[sol].angle_

    def get_point(self, sol, p):
        sol = self.sols[sol]
        return sol.origin_ + mat_mul_n(rot(sol.angle_), p)

    def compile(self):
        if not self.blocked or set(self.blocked) == set(self.piloted):
            self.kin_instr, self.dyn_instr = compiler(self, BOTH)
        else:
            self.kin_instr, self.dyn_instr = compiler(self), compiler(self, DYNAMICS)
        print('signs =', self.signs)
        
    def solve_kinematics(self, inputs=None):
        if inputs is None:
            self.reset(1)
        else:
            if isinstance(inputs, (list, tuple)):
                inputs = np.array(inputs)
            if len(inputs.shape) == 1:
                inputs = inputs[np.newaxis, :]
            self.reset(inputs.shape[1])
            self.input = inputs
            
        for instr in self.kin_instr:
            eq = kin[instr[0]](self, *instr[1:])
            for s in eq:
                self.eqs[s] = eq
        for s in self.sols:
            make_continuous(s.angle_)
            
    def solve_statics(self, compute_kine=True, inputs=None):
        if compute_kine:
            self.solve_kinematics(inputs)
            
        for s in self.sols:
            s.mech_actions = []
            og = self.get_point(s.rep, s.g_)
            f_tot, t_tot = np.array(((0.,), (0.,))), 0.
            for f, t, p in s.external_actions:
                f = f()
                f_tot += f
                t_tot += t() + det(self.get_point(s.rep, p) - og, f)
            s.mech_actions.append(MechanicalAction(f_tot, og, t_tot))
        
        for inter in self.interactions:
            inter.set_ma(self)
        for instr in self.dyn_instr:
            dyn[instr[0]](self, *instr[1:])
            
    def solve_dynamics(self, t, compute_kine=True, inputs=None):
        if compute_kine:
            self.solve_kinematics(inputs)
        dt = t/(self.input.shape[1] - 1) * self.units['Time']
        
        for s in self.sols:
            s.mech_actions = []
            og = self.get_point(s.rep, s.g_)
            s.mech_actions.append(
                MechanicalAction(-s.m_ * derivative2_vec(og, dt), og, -s.j_ * derivative2(s.angle_, dt))
            )
            f_tot, t_tot = np.array(((0.,), (0.,))), 0.
            for f, t, p in s.external_actions:
                f = f()
                f_tot += f
                t_tot += t() + det(self.get_point(s.rep, p) - og, f)
            s.mech_actions.append(MechanicalAction(f_tot, og, t_tot))

        for inter in self.interactions:
            inter.set_ma(self)
        for instr in self.dyn_instr:
            dyn[instr[0]](self, *instr[1:])

    def add_gravity(self, g=None):
        af = Gravity(g * self.units['Acceleration'] if g is not None else (0, -9.81))
        self.interactions.append(af)
        return af

    def add_spring(self, k, l0, s1, s2, p1=(0., 0.), p2=(0, 0.)):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]

        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        spr = Spring(k * self.units['SpringConstant'], l0 * self.units['Length'], s1, s2, np.array(p1) * self.units['Length'], np.array(p2) * self.units['Length'])
        self.interactions.append(spr)
        return spr

    def __repr__(self):
        return self.name

    def add_gear(self, rev1, rev2, r, v0=0.):
        if isinstance(rev1, Joint):
            rev1 = rev1.name
        if isinstance(rev1, str):
            rev1 = self.named_joints[rev1]
        if not isinstance(self.joints[rev1], RevoluteJoint):
            raise TypeError('Joints must be RevoluteJoints')

        if isinstance(rev2, Joint):
            rev2 = rev2.name
        if isinstance(rev2, str):
            rev2 = self.named_joints[rev2]
        if not isinstance(self.joints[rev2], RevoluteJoint):
            raise TypeError('Joints must be RevoluteJoints')

        g = Gear(self, rev1, rev2, r, v0)
        self.relations.append(g)
        return g

    def add_gearrack(self, rev, pri, r, v0=0.):
        if isinstance(rev, Joint):
            rev = rev.name
        if isinstance(rev, str):
            rev = self.named_joints[rev]
        if not isinstance(self.joints[rev], RevoluteJoint):
            raise TypeError('Joint 1 must be a RevoluteJoint')

        if isinstance(pri, Joint):
            pri = pri.name
        if isinstance(pri, str):
            pri = self.named_joints[pri]
        if not isinstance(self.joints[pri], PrismaticJoint):
            raise TypeError('Joint 2 must be a PrismaticJoint')

        gr = GearRack(self, rev, pri, r, v0)
        self.relations.append(gr)
        return gr

    def add_effortless_relation(self, j1, j2, r, v0=0.):
        if isinstance(j1, Joint):
            j1 = j1.name
        if isinstance(j1, str):
            j1 = self.named_joints[j1]
        if self.joints[j1].dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')

        if isinstance(j2, Joint):
            j2 = j2.name
        if isinstance(j2, str):
            j2 = self.named_joints[j2]
        if self.joints[j2].dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')

        er = EffortlessRelation(self, j1, j2, r, v0)
        self.relations.append(er)
        return er

    def add_distant_relation(self, j1, j2, r, v0=0.):
        if isinstance(j1, Joint):
            j1 = j1.name
        if isinstance(j1, str):
            j1 = self.named_joints[j1]
        if self.joints[j1].dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')

        if isinstance(j2, Joint):
            j2 = j2.name
        if isinstance(j2, str):
            j2 = self.named_joints[j2]
        if self.joints[j2].dof != 1:
            raise TypeError('Joints must have 1 degree of freedom')

        dr = DistantRelation(self, j1, j2, r, v0)
        self.relations.append(dr)
        return dr
