from linkage import *
from solid import *


class System:
    def __init__(self, sols=(), links=(), piloted=(), sgns=None):
        self.sols, self.links, self.piloted, self.sgns = list(sols) if sols else [Solid(name='Ground')], list(links), list(piloted), dict() if sgns is None else sgns
        self.named_sols = {s.name: i for i, s in enumerate(self.sols)}
        self.named_links = {s.name: i for i, s in enumerate(links)}
        self.tot = 0
        self.indices = {}
        for l in piloted:
            pil = []
            for _ in self.links[l].inputMode():
                pil.append(self.tot)
                self.tot += 1
            self.indices[l] = tuple(pil)
        self.eqs = None
        
    def addSol(self, points=(), named_points=None, name=''):
        s = Solid(points, named_points, name)
        self.named_sols[s.name] = len(self.sols)
        self.sols.append(s)
        return s
    
    def addPivot(self, sol1, sol2, p1=(0., 0.), p2=(0., 0.), name=''):
        if isinstance(sol1, str):
            sol1 = self.named_sols[sol1]
        if isinstance(p1, (tuple, list)):
            self.sols[sol1].points.append(tuple(p1))
            p1 = len(self.sols[sol1].points) - 1
    
        if isinstance(sol2, str):
            sol2 = self.named_sols[sol2]
        if isinstance(p2, (tuple, list)):
            self.sols[sol2].points.append(tuple(p2))
            p2 = len(self.sols[sol2].points) - 1
        
        p = Pivot(sol1, sol2, p1, p2, name)
        print(f'Added linkage {p}')
        self.named_links[p.name] = len(self.links)
        self.links.append(p)
        return p

    def addGlissiere(self, sol1, sol2, alpha1=0., d1=0., alpha2=0., d2=0., name=''):
        if isinstance(sol1, str):
            sol1 = self.named_sols[sol1]
        if isinstance(sol2, str):
            sol2 = self.named_sols[sol2]
            
        g = Glissiere(sol1, sol2, alpha1, d1, alpha2, d2, name)
        print(f'Added linkage {g}')
        self.named_links[g.name] = len(self.links)
        self.links.append(g)
        return g
    
    def addSpherePlan(self, sol1, sol2, p=(0., 0.), alpha=0., d=0., name=''):
        if isinstance(sol1, str):
            sol1 = self.named_sols[sol1]
        if isinstance(sol2, str):
            sol2 = self.named_sols[sol2]
        
        if isinstance(p, (tuple, list)):
            self.sols[sol1].points.append(tuple(p))
            p = len(self.sols[sol1].points) - 1
            
        sp = SpherePlan(sol1, sol2, p, alpha, d, name)
        print(f'Added linkage {sp}')
        self.named_links[sp.name] = len(self.links)
        self.links.append(sp)
        return sp
    
    def addTranslation(self, sol1, sol2, angle=0., base=(0., np.pi/2), name=''):
        if isinstance(sol1, str):
            sol1 = self.named_sols[sol1]
        if isinstance(sol2, str):
            sol2 = self.named_sols[sol2]
        
        t = Translation(sol1, sol2, angle, base, name)
        print(f'Added linkage {t}')
        self.named_links[t.name] = len(self.links)
        self.links.append(t)
        return t
    
    def pilot(self, link):
        if isinstance(link, (tuple, list)):
            for l in link:
                if isinstance(l, str):
                    l = self.named_links[l]
                self.piloted.append(l)
                pil = []
                for _ in self.links[l].inputMode():
                    pil.append(self.tot)
                    self.tot += 1
                self.indices[l] = tuple(pil)
            return self.showInput()
        elif isinstance(link, str):
            link = self.named_links[link]
        pil = []
        for _ in self.links[link].inputMode():
            pil.append(self.tot)
            self.tot += 1
        self.indices[link] = tuple(pil)
        self.piloted.append(link)
        self.showInput()
    
    def showInput(self):
        l = []
        for link in self.piloted:
            for m in self.links[link].inputMode():
                l.append(m)
        print(f"({'; '.join(l)})")
    
    def reset(self, n):
        self.eqs = [(i,) for i in range(len(self.sols))]
        for sol in self.sols:
            sol.reset(n)
        for link in self.links:
            link.reset(n)
    
    def getPoint(self, sol, index):  # -> (2, 1, n)
        return self.sols[sol]._points[:, index+1:index+2, :]
    
    def getOrigin(self, sol):
        return self.sols[sol]._points[:, :1, :]
    
    def getRef(self, sol):
        return self.sols[sol].angle
    
        