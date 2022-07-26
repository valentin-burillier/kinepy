from kinepy.linkage import *
from kinepy.solid import *
from kinepy.compilation import compiler, DYNAMICS, BOTH
from kinepy.kinematic import kin
from kinepy.dynamic import *
import json


class System:
    def __init__(self, sols=(), joints=(), piloted=(), blocked=(), signs=None, name=''):
        self.sols, self.joints = list(sols) if sols else [Solid(name='Ground')], list(joints)
        self.piloted, self.blocked = list(piloted), list(blocked)
        self.name = name if name else 'Main system'
        
        # Dictionnaires des noms
        self.named_sols = {s.name: i for i, s in enumerate(self.sols)}
        self.named_joints = {s.name: i for i, s in enumerate(joints)}

        # Référence dans les liaisons
        for l_ in self.joints:
            l_.system = self

        # Pilotage, mise à jour de l'entrée pour résolution cinématiue
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

    def add_solid(self, points=(), named_points=None, j=0., m=0., g=0., name=''):
        s = Solid(points, named_points, j, m, g, name, len(self.sols))
        self.named_sols[s.name] = len(self.sols)
        self.sols.append(s)
        return s
    
    def add_revolute(self, s1, s2, p1=(0., 0.), p2=(0., 0.)):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(p1, (tuple, list)):
            # Nouveau point
            self.sols[s1].points.append(tuple(p1))
            p1 = len(self.sols[s1].points) - 1
    
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        if isinstance(p2, (tuple, list)):
            # Nouveau point
            self.sols[s2].points.append(tuple(p2))
            p2 = len(self.sols[s2].points) - 1
        
        p = RevoluteJoint(s1, s2, p1, p2)
        print(f'Added linkage {p}')
        self.named_joints[p.name] = len(self.joints)

        # Les points de la liaison portent le nom de la liaison
        self.sols[s1].named_points[p.name] = p1
        self.sols[s2].named_points[p.name] = p2

        self.joints.append(p)
        p.system = self
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
        return g
    
    def add_pin_slot(self, s1, s2, a1=0., d1=0., p2=(0., 0.)):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        
        if isinstance(p2, (tuple, list)):
            self.sols[s2].points.append(tuple(p2))
            p2 = len(self.sols[s2].points) - 1
            
        sp = PinSlotJoint(s1, s2, a1, d1, p2)
        print(f'Added linkage {sp}')
        self.named_joints[sp.name] = len(self.joints)

        # Le point de la liaison porte le nom de la liaison
        self.sols[s2].named_points[sp.name] = p2

        self.joints.append(sp)
        sp.system = self
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
    
    def pilot(self, joint):
        if isinstance(joint, (tuple, list)):
            # Plusieurs liaisons en entrée
            for j in joint:
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
        if isinstance(joint, Joint):
            joint = joint.name
        if isinstance(joint, str):
            # Référence par le nom
            joint = self.named_joints[joint]

        # Mse à jour de l'entrée pour résolution cinématiue
        pil = []
        for _ in self.joints[joint].input_mode():
            pil.append(self.tot)
            self.tot += 1
        self.indices[joint] = tuple(pil)

        self.piloted.append(joint)
        self.show_input()

    def block(self, joint):
        if isinstance(joint, (tuple, list)):
            for j in joint:
                if isinstance(j, Joint):
                    j = j.name
                if isinstance(j, str):
                    # Référence par le nom
                    j = self.named_joints[j]
                self.blocked.append(j)
        else:
            if isinstance(joint, Joint):
                joint = joint.name
            if isinstance(joint, str):
                # Référence par le nom
                joint = self.named_joints[joint]
            self.blocked.append(joint)

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
        
    def solve_kinematics(self, input_):
        if isinstance(input_, list):
            input_ = np.array(input_)
        if len(input_.shape) == 1:
            input_ = input_[np.newaxis, :]
        self.reset(input_.shape[1])
        self.input = input_
        for instr in self.kin_instr:
            eq = kin[instr[0]](self, *instr[1:])
            for s in eq:
                self.eqs[s] = eq

    def solve_dynamics(self, dt):
        for s in self.sols:
            og = s.get_point(s.g)
            s.mech_actions['Inertie'] = MechanicalAction(
                -s.m * derivative2_vec(og, dt), og, -s.j * derivative2(s.angle, dt)
            )
        for inter in self.interactions:
            inter.set_am(self)

    def add_acceleration_field(self, g, name='Gravity'):
        self.interactions.append(AccelerationField(g, name))

    def add_spring(self, k, l0, s1, s2, p1=(0, 0), p2=(2, 0)):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(p1, (tuple, list)):
            # Nouveau point
            self.sols[s1].points.append(tuple(p1))
            p1 = len(self.sols[s1].points) - 1

        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        if isinstance(p2, (tuple, list)):
            # Nouveau point
            self.sols[s2].points.append(tuple(p2))
            p2 = len(self.sols[s2].points) - 1

        self.interactions.append(Spring(k, l0, s1, s2, p1, p2))

    def get_data(self):
        return {
            'sols': [s.save() for s in self.sols],
            'joints': [l_.save() for l_ in self.joints],
            'piloted': self.piloted,
            'blocked': self.blocked,
            'signs': self.signs
        }

    def save(self, file):
        with open(file, 'w') as f:
            json.dump(self.get_data(), f)

    @classmethod
    def load(cls, file):
        with open(file) as f:
            data = json.load(f)
        data['sols'] = [Solid.load(s) for s in data['sols']]
        data['joints'] = [class_dict[name].load(d) for name, d in data['joints']]
        return cls(**data)

    def __repr__(self):
        return self.name
