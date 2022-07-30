from kinepy.linkage import *
from kinepy.solid import *
from kinepy.compilation import compiler, DYNAMICS, BOTH
from kinepy.kinematic import kin
from kinepy.dynamic import dyn
from kinepy.interactions import Spring, Gravity
import json


class System:
    def __init__(self, sols=(), joints=(), piloted=(), blocked=(), signs=None, name=''):
        self.sols, self.joints = list(sols) if sols else [Solid(name='Ground')], list(joints)
        self.piloted, self.blocked = list(piloted), list(blocked)
        self.name = name if name else 'Main system'

        # Renumérotation des Sols
        for i, s in enumerate(self.sols):
            s.rep = i

        # Dictionnaires des noms
        self.named_sols = {s.name: i for i, s in enumerate(self.sols)}
        self.named_joints = {s.name: i for i, s in enumerate(joints)}

        # Référence dans les liaisons
        for l_ in self.joints:
            l_.system = self

        # Pilotage, mise à jour de l'entrée pour résolution cinématique
        self.tot, self.indices = 0, {}
        for l_ in piloted:
            pil = []
            for _ in self.joints[l_].input_mode():
                pil.append(self.tot)
                self.tot += 1
            self.indices[l_] = tuple(pil)
        self.show_input()

        # Préparation cinématique
        self.input = None
        self.signs = dict() if signs is None else signs
        self.eqs = None
        self.kin_instr, self.dyn_instr = [], []

        # Préparation dynamique
        self.interactions = []

    def add_solid(self, j=0., m=0., g=(0., 0.), name=''):
        s = Solid(j, m, g, name, len(self.sols))
        self.named_sols[s.name] = len(self.sols)
        self.sols.append(s)
        return s
    
    def add_revolute(self, s1, s2, p1=(0., 0.), p2=(0., 0.)):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
    
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        
        p = RevoluteJoint(s1, s2, p1, p2)
        print(f'Added linkage {p}')
        self.named_joints[p.name] = len(self.joints)

        self.joints.append(p)
        p.system = self
        self.interactions.append(p.interaction)
        return p

    def add_prismatic(self, s1, s2, a1=0., d1=0., a2=0., d2=0.):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
            
        g = PrismaticJoint(s1, s2, a1, d1, a2, d2)
        print(f'Added linkage {g}')
        self.named_joints[g.name] = len(self.joints)
        self.joints.append(g)
        g.system = self
        self.interactions.append(g.interaction)
        return g
    
    def add_pin_slot(self, s1, s2, a1=0., d1=0., p2=(0., 0.)):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
            
        sp = PinSlotJoint(s1, s2, a1, d1, p2)
        print(f'Added linkage {sp}')
        self.named_joints[sp.name] = len(self.joints)

        self.joints.append(sp)
        sp.system = self
        self.interactions.append(sp.interaction)
        return sp
    
    def add_rectangle(self, s1, s2, angle=0., base=(0., np.pi/2)):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        
        t = RectangularJoint(s1, s2, angle, base)
        print(f'Added linkage {t}')
        self.named_joints[t.name] = len(self.joints)
        self.joints.append(t)
        t.system = self
        return t

    def add_3dof(self, s1, s2):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]

        _3dof = ThreeDegreesOfFreedomJoint(s1, s2)
        print(f'Added linkage {_3dof}')
        self.named_joints[_3dof.name] = len(self.joints)
        self.joints.append(_3dof)
        _3dof.system = self
        return _3dof

    def add_joint(self, joint: Joint):
        self.named_joints[joint.name] = len(self.joints)
        self.joints.append(joint)
        joint.system = self
        if joint.interaction is not None:
            self.interactions.append(joint.interaction)
    
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
        return self.sols[sol].origin
    
    def get_ref(self, sol):
        return self.sols[sol].angle

    def compile(self):
        if not self.blocked or set(self.blocked) == set(self.piloted):
            self.kin_instr, self.dyn_instr = compiler(self, BOTH)
        else:
            self.kin_instr, self.dyn_instr = compiler(self), compiler(self, DYNAMICS)
        print('signs =', self.signs)
        
    def solve_kinematics(self, inputs):
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
            make_continuous(s.angle)
            
    def solve_statics(self, inputs=None, compute_kine=True):
        if compute_kine:
            self.solve_kinematics(inputs)
        for s in self.sols:
            s.mech_actions = []
            og = s.get_point(s.g)
            f_tot, t_tot = np.array(((0.,), (0.,))), 0.
            for f, t, p in s.external_actions:
                f = f()
                f_tot += f
                t_tot += t() + det(s.get_point(p) - og, f)
            s.mech_actions.append(MechanicalAction(f_tot, og, t_tot))
        
        for inter in self.interactions:
            inter.set_ma(self)
        for instr in self.dyn_instr:
            dyn[instr[0]](self, *instr[1:])
            
    def solve_dynamics(self, t, inputs=None, compute_kine=True):
        if compute_kine:
            self.solve_kinematics(inputs)
        dt = t/(self.input.shape[1] - 1)
        for s in self.sols:
            s.mech_actions = []
            og = s.get_point(s.g)
            s.mech_actions.append(MechanicalAction(-s.m * derivative2_vec(og, dt), og, -s.j * derivative2(s.angle, dt)))
            f_tot, t_tot = np.array(((0.,), (0.,))), 0.
            for f, t, p in s.external_actions:
                f = f()
                f_tot += f
                t_tot += t() + det(s.get_point(p) - og, f)
            s.mech_actions.append(MechanicalAction(f_tot, og, t_tot))

        for inter in self.interactions:
            inter.set_ma(self)
        for instr in self.dyn_instr:
            dyn[instr[0]](self, *instr[1:])

    def add_gravity(self, g=(0, -9.81)):
        af = Gravity(g)
        self.interactions.append(af)
        return af

    def add_spring(self, k, l0, s1, s2, p1=(0, 0), p2=(2, 0)):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]

        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        spr = Spring(k, l0, s1, s2, p1, p2)
        self.interactions.append(spr)
        return spr

    def __repr__(self):
        return self.name
