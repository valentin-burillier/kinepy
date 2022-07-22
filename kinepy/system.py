from kinepy.linkage import *
from kinepy.solid import *
from kinepy.compilation import compiler, DYNAMICS, BOTH
from kinepy.kinematic import kin


class System:
    def __init__(self, sols=(), joints=(), piloted=(), blocked=(), signs=None):
        self.sols, self.joints = list(sols) if sols else [Solid(name='Ground')], list(joints)
        self.piloted, self.blocked = list(piloted), list(blocked)

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
            for _ in self.joints[l_].inputMode():
                pil.append(self.tot)
                self.tot += 1
            self.indices[l_] = tuple(pil)
        self.show_input()

        # Préparation cinématique
        self.input = None
        self.signs = dict() if signs is None else signs
        self.eqs = None
        self.kin_instr, self.dyn_instr = [], []
        
    def add_solid(self, points=(), named_points=None, j=0., m=0., g=0., name=''):
        s = Solid(points, named_points, j, m, g, name)
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

    def add_prismatic(self, s1, s2, alpha1=0., d1=0., alpha2=0., d2=0.):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
            
        g = PrismaticJoint(s1, s2, alpha1, d1, alpha2, d2)
        print(f'Added linkage {g}')
        self.named_joints[g.name] = len(self.joints)
        self.joints.append(g)
        g.system = self
        return g
    
    def add_pin_slot(self, s1, s2, alpha1=0., d1=0., p2=(0., 0.)):
        if isinstance(s1, str):
            # Référence par le nom
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            # Référence par le nom
            s2 = self.named_sols[s2]
        
        if isinstance(p2, (tuple, list)):
            self.sols[s2].points.append(tuple(p2))
            p2 = len(self.sols[s2].points) - 1
            
        sp = PinSlotJoint(s1, s2, alpha1, d1, p2)
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

    def add_joint(self, joint):
        self.named_joints[joint.name] = len(self.joints)
        self.joints.append(joint)
        joint.system = self
    
    def pilot(self, joint):
        if isinstance(joint, (tuple, list)):
            # Plusieurs liaisons en entrée
            for j in joint:
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
        elif isinstance(joint, str):
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

    def solve_kinematics(self, input_):
        # pour cinméatique: tout est en (n,)
        self.reset(input_.shape[1])
        self.input = input_
        for instr in self.kin_instr:
            eq = kin[instr[0]](self, *instr[1:])
            for s in eq:
                self.eqs[s] = eq

    def solve_dynamics(self):
        # Après dynamique: tout est en (n-2,)
        pass
