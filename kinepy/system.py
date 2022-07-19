from kinepy.linkage import *
from kinepy.solid import *


class System:
    def __init__(self, sols=(), joints=(), piloted=(), signs=None):
        self.sols, self.joints = list(sols) if sols else [Solid(name='Ground')], list(joints)
        self.piloted, self.signs = list(piloted), dict() if signs is None else signs
        self.named_sols = {s.name: i for i, s in enumerate(self.sols)}
        self.named_joints = {s.name: i for i, s in enumerate(joints)}
        self.tot = 0
        self.indices = {}
        for l_ in self.joints:
            l_.system = self
        for l_ in piloted:
            pil = []
            for _ in self.joints[l_].inputMode():
                pil.append(self.tot)
                self.tot += 1
            self.indices[l_] = tuple(pil)
        self.eqs = None
        
    def add_solid(self, points=(), named_points=None, name=''):
        s = Solid(points, named_points, name)
        self.named_sols[s.name] = len(self.sols)
        self.sols.append(s)
        return s
    
    def add_revolute(self, s1, s2, p1=(0., 0.), p2=(0., 0.)):
        if isinstance(s1, str):
            s1 = self.named_sols[s1]
        if isinstance(p1, (tuple, list)):
            self.sols[s1].points.append(tuple(p1))
            p1 = len(self.sols[s1].points) - 1
    
        if isinstance(s2, str):
            s2 = self.named_sols[s2]
        if isinstance(p2, (tuple, list)):
            self.sols[s2].points.append(tuple(p2))
            p2 = len(self.sols[s2].points) - 1
        
        p = RevoluteJoint(s1, s2, p1, p2)
        print(f'Added linkage {p}')
        self.named_joints[p.name] = len(self.joints)
        self.sols[s1].named_points[p.name] = p1
        self.sols[s2].named_points[p.name] = p2
        self.joints.append(p)
        p.system = self
        return p

    def add_prismatic(self, s1, s2, alpha1=0., d1=0., alpha2=0., d2=0.):
        if isinstance(s1, str):
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            s2 = self.named_sols[s2]
            
        g = PrismaticJoint(s1, s2, alpha1, d1, alpha2, d2)
        print(f'Added linkage {g}')
        self.named_joints[g.name] = len(self.joints)
        self.joints.append(g)
        g.system = self
        return g
    
    def add_slide_curve(self, s1, s2, alpha1=0., d1=0., p2=(0., 0.)):
        if isinstance(s1, str):
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            s2 = self.named_sols[s2]
        
        if isinstance(p2, (tuple, list)):
            self.sols[s1].points.append(tuple(p2))
            p2 = len(self.sols[s1].points) - 1
            
        sp = SlideCurveJoint(s1, s2, alpha1, d1, p2)
        print(f'Added linkage {sp}')
        self.named_joints[sp.name] = len(self.joints)
        self.sols[s2].named_points[sp.name] = p2
        self.joints.append(sp)
        sp.system = self
        return sp
    
    def add_double_prismatic(self, s1, s2, angle=0., base=(0., np.pi/2)):
        if isinstance(s1, str):
            s1 = self.named_sols[s1]
        if isinstance(s2, str):
            s2 = self.named_sols[s2]
        
        t = DoublePrismaticJoint(s1, s2, angle, base)
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
            for j in joint:
                if isinstance(j, str):
                    j = self.named_joints[j]
                self.piloted.append(j)
                pil = []
                for _ in self.joints[j].input_mode():
                    pil.append(self.tot)
                    self.tot += 1
                self.indices[j] = tuple(pil)
            return self.show_input()
        elif isinstance(joint, str):
            joint = self.named_joints[joint]
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

        