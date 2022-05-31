import numpy as np
import matplotlib.pyplot as plt
import tools as t

__version__ = "2.1.3"


# rotation matrices
R = lambda theta: np.array([[c := np.cos(theta), -(s := np.sin(theta))], [s, c]])
# theta: np.ndarray, shape (n,)
# return: np.ndarray, shape (2, 2, n)

# unit vector
U = lambda alpha:  np.array(([np.cos(alpha)], [np.sin(alpha)]))
# alpha: np.ndarray, shape (n,)
# return: np.ndarray, shape (2, 1, n)

# np.einsum magic for (2, 2, n) x (2, m, n) -> (2, m, n) shapes
TAG = 'ijl, jkl->ikl'



def is_bati(union: tuple[int]):
    return 0 in union

class Solid:
    """
    Create a Solid object.
    
    Attributes
    ----------
    name : str
        The name of the solid.
    index : int
        The index of the solid.
    origin : ndarray
        Successive values of the origin of the reference frame of the solid after a simulation.
    angle : ndarray
        Successive values of the angle between the solid and the global coordinate system after a simulation.
        
    Methods
    -------
    get_point(name: str):
        Coordinates of the successive points *name* expressed in the global coordinate system after a simulation.
    __setitem__(name:str, point):
        Set up the coordinate of the point *name* in the frame of the solid.
   __getitem__(name: str):
        Get the coordinate of the point *name* in the frame of the solid.        
    """

    def __init__(self, points=None, name=''):
        # (0, 0) est toujours le premier point, sinon je te tape
        self.name = name
        self.named_points = dict()
        self.points, self.alpha = 0, 0
        self.index = 0
        if points is not None:
            self._points = np.reshape(points, points.shape + (1,))
        else:
            self._points = None
        self.forces, self.index_forces, self._forces = [], (), []
        self.couples, self._couples = [], []
        self.m, self.I = 0., 0.
        self.G = (0, 0)
        self.G_index = 0
        
    def change_ref(self, alpha, matrix, vec):
        self.alpha += alpha
        np.einsum(TAG, matrix, self.points, out=self.points)
        self.points += vec
        
    def __getitem__(self, name: str):
        return self.named_points[name]
    
    def __setitem__(self, name: str, point: tuple):
        self.named_points[name] = point

    def reset(self, n: int):
        self.points = np.concatenate((self._points,) * n, axis=2)
        self.alpha = np.zeros((n,), float)

    def get_point(self, i):
        """
        documentation
        """
        
        if isinstance(i, str):
            i = list(self.named_points.keys()).index(i) + 1

        return self.points[:, i, :]
    
    @property
    def angle(self):
        return self.alpha
    
    @property
    def origin(self):
        return self.get_point(0)
    
    def physical_parameters(self, m, I, G):
        """
        Add physical parameters.
        
        Parameters
        ----------
        m : float
            Mass of solid. Default is 0.
        I : float
            Inertia of solid at the point *G*. Default is 0.
        G : tuple
            Center of gravity and inertia of the solid, Default is (0, 0).
        """
        
        self.m, self.I, self.G = m, I, G
        
        
    def applie_external_force(self, force, point): # F(ext -> sol1) exprimer dans le rep 0
        self.forces.append((force, point))
        
    def applie_external_couple(self, couple): # C(ext -> sol1) exprimer dans le rep 0
        self.couples.append(couple)
        
    def __repr__(self):
        return self.name if self.name else f'Solid {self.index}'




def distance(current, graph, dists, nodes):
    queue = [current]
    while queue:
        c = queue.pop(0)
        for s, _ in graph[c]:
            eq = nodes[s]
            if dists[eq] > dists[c] + 1:
                queue.append(eq)
                dists[eq] = dists[c] + 1

def selectPiloted(piloted, dists, nodes):
    ib, d = 0, min(dists[nodes[piloted[0][0]]], dists[nodes[piloted[0][1]]])
    for i, (s1, s2, _) in enumerate(piloted[1:]):
        d_ = min(dists[nodes[s1]], dists[nodes[s2]])
        if d_ < d:
            ib, d = i, d_
    return ib, d

def fusion(i, j, eqs, nodes, graph, dists):
    pi, pj = nodes[i], nodes[j]
    eqi, eqj = eqs[pi], eqs[pj]
    dists[pi] = min(dists[pi], dists[pj])
    for s in eqj:
     nodes[s] = pi
    eqs[pi] = eqi + eqj
    graph[pi] = [(s, l) for s, l in (graph[pi] + graph[pj]) if nodes[s] != pi]
    if pj + 1 == len(eqs):
        eqs.pop()
        graph.pop()
        dists.pop()
    else:
        eqs[pj] = eqs.pop()
        graph[pj] = graph.pop()
        dists[pj] = dists.pop()
        for s in eqs[pj]:
         nodes[s] = pj
    distance(min(pi, pj), graph, dists, nodes)


valid_cycles = {
    'G_G',
    'P_P_P',
    'G_P_P',
    'P_G_P',
    'P_P_G',
    'G_G_P',
    'G_P_G',
    'P_G_G'
}

signed_cycles = {
    'P_P_P',
    'G_P_P',
    'P_G_P',
    'P_P_G'
}

def chercheCycle(graph, nodes, dists, dp, ls):
    valid = [True] * len(graph)
    queue = [0]
    while queue and dists[(x := queue.pop(0))] < dp:
        nbs, check = [], [None] * len(graph)
        for s, i in graph[x]:
            n = nodes[s]
            if not valid[n]:
                continue
            if check[n] is not None and '_'.join((ls[i].id_, ls[check[n]].id_)) in valid_cycles: # 2-cycle
                return min(i, check[n]), max(i, check[n])
            check[n] = i
            nbs.append(n)
        for nb in nbs:
            for s, i in graph[nb]:
                n = nodes[s]
                if check[n] is not None and '_'.join((ls[i].id_, ls[check[n]].id_, ls[check[nb]].id_)) in valid_cycles: # 3-cycle
                    return tuple(sorted((i, check[n], check[nb])))
        valid[x] = False
        queue += nbs


# arrange les identifieurs de Liaison de sorte que les solides dans les même Eq soient "côte-à-côte"
def identify(cycle, nodes, eqs, m_sols): # mise en place de arguments pour fonctions et choix du ref, ref est l'Eq entre l1 et l2
    if len(cycle) == 2:
        l1, l2 = cycle
        if nodes[l1[2][0]] != nodes[l2[0][0]]:
            l2 = l2[::-1]
        if is_bati(eqs[nodes[l1[0][0]]]) or sum(m_sols[i] for i in eqs[nodes[l1[2][0]]]) < sum(m_sols[i] for i in eqs[nodes[l1[0][0]]]):
            return l2, l1
        return l1, l2
    l1, l2, l3 = cycle
    if nodes[l1[2][0]] != nodes[l2[0][0]]:
        if nodes[l1[2][0]] != nodes[l2[0][0]]:
            l2, l3 = l3, l2
            if nodes[l1[2][0]] != nodes[l2[0][0]]:
                l2 = l2[::-1]
        else:
            l2 = l2[::-1]
    if nodes[l2[2][0]] == nodes[l3[2][0]]:
        l3 = l3[::-1]
    if is_bati(eqs[nodes[l2[2][0]]]):
        return l2, l3, l1
    if is_bati(eqs[nodes[l3[2][0]]]):
        return l3, l1, l2
    if sum(m_sols[i] for i in eqs[nodes[l1[2][0]]]) < sum(m_sols[i] for i in eqs[nodes[l1[0][0]]]):
        if sum(m_sols[i] for i in eqs[nodes[l1[0][0]]]) < sum(m_sols[i] for i in eqs[nodes[l2[2][0]]]):
            return l2, l3, l1
        return l3, l1, l2
    if sum(m_sols[i] for i in eqs[nodes[l1[2][0]]]) < sum(m_sols[i] for i in eqs[nodes[l2[2][0]]]):
        return l2, l3, l1
    return l1, l2, l3


changed_arangements = {
    'P_G_P', # vers GPP  ! faut garder le même ref !
    'G_P_G' # vers  PGG  ! faut garder le même ref !
}


def change(arg):
    l1, l2, l3 = arg
    return l2[::-1], l1[::-1], l3[::-1]


class System:
    """
    Create a System object.
    
    Attributes
    ----------
    name : str
        Name of the system.
    sols : tuple
        List of solids present in the system. The position of the solids in the list corresponds to their index.
    liaisons : tuple
        List of linkages present in the system. The order of the linkages corresponds to the order in which they were added.
    sgns : dict
        Lists the different signs of kinematic cycles.
        
    Methods
    -------
    add(obj):
        Add to system *obj*.
    pilot(liaisons_piloted):
        Specifies the links piloted during the simulation.
    compile_kinematic(sgns: dict):
        Detects configuration errors.
        Informs the user of the cycles present in the system and the sign chosen for them so that he/she can make a correction if necessary.
        Establishes the resolution strategy.
    solve(inputs):
        Performs the simulation.
    """
    
    def __init__(self, sols=(Solid(name='bati'), ), liaisons=tuple(), piloted=tuple(), sgns=None, name=''):
        # sols, le premier est considéré comme le bâti
        # piloted, indices des liaisons pilotées
        # sgns, dictionnaire des signes pour les triplets d'indices de liaisons dans l'ordre (pour l'unicité de description)
        self.sols, self.eqs, self.liaisons, self.piloted = sols, [], liaisons, piloted
        self.sgns = {} if sgns is None else sgns
        self.blocked = []
        self.compiled_kinematic = False
        self.instructions_kinematic = []
        self.compiled_static = False
        self.instructions_static = []
        self.name = name
        self.gravity = (False, -np.pi/2, 10)

    def change_ref(self, eq, alpha, matrix, vec):
        for s in eq:
            self.sols[s].change_ref(alpha, matrix, vec)

    def get_point(self, sol, i):
        return self.sols[sol].points[:, i:i+1, :]

    def get_ref(self, sol):
        return self.sols[sol].alpha
            
    def compile_kinematic(self, sgns=None):
        """
        Detects configuration errors.
        Informs the user of the cycles present in the system and the sign chosen for them so that he/she can make a correction if necessary.
        Establishes the resolution strategy.
        
        Parameters
        ----------
        sgns : dict, optionnal
            Dictionary of the form {(1, 2, 0):1, (3, 4, 1):-1, ...} allowing to fix the signs of the different kinematic cycles of the mechanism
        """
        
        self.reset_points()
        self.sgns = {} if sgns is None else sgns

        instructions = []
        
        for i, p in enumerate(self.piloted): # identification des L_p
            self.liaisons[p].piloted = True
        
        eqs, nodes = [(i,) for i in range(len(self.sols))], list(range(len(self.sols)))
        graph = [[] for _ in range(len(self.sols))]
        piloted = []
        
        for i, L in enumerate(self.liaisons): # création du graphe sans L_p
            if L.piloted:
                piloted.append((L.sol1, L.sol2, i))
            else:
                graph[L.sol1].append((L.sol2, i))
                graph[L.sol2].append((L.sol1, i))
        
        dists = [float('inf')] * len(self.sols) # distances de chaque eq à 0
        dists[0] = 0
        distance(0, graph, dists, nodes) # parcours en largeur
        
        m_sols = [sol._points.shape[1] for sol in self.sols]
        while len(eqs) > 1:
            if piloted:
                ip, dp = selectPiloted(piloted, dists, nodes)
                c = chercheCycle(graph, nodes, dists, dp, self.liaisons)
                if not dp or c is None:
                    s1, s2, l = piloted.pop(ip)
                    instructions.append(('Pilot', (l,)))
                    fusion(s1, s2, eqs, nodes, graph, dists)
                    continue
            else:
                c = chercheCycle(graph, nodes, dists, len(graph), self.liaisons)
            arg = identify(tuple(self.liaisons[i].identifier(i) for i in c), nodes, eqs, m_sols)
            cycle = '_'.join(self.liaisons[i].id_ for _, i, _ in arg)
            if cycle in changed_arangements:
                arg = change(arg)
                cycle = '_'.join(self.liaisons[i].id_ for _, i, _ in arg)
            if cycle in signed_cycles:
                print(f'Signed cycle detected for resolution: {c} of type {cycle}')
                if c not in self.sgns:
                    self.sgns[c] = 1
                    print(f'Chosen 1 as sign for {c}')
                else:
                    print(f'You have Chosen {self.sgns[c]} as sign for {c}')
            instructions.append((cycle, (c,) + arg))
            fusion(arg[0][0][0], arg[0][2][0], eqs, nodes, graph, dists)
            if len(c) == 3:
                fusion(arg[0][0][0], arg[1][2][0], eqs, nodes, graph, dists)
                
        self.instructions_kinematic = instructions
        self.compiled_kinematic = True

    def G_G(self, c, l1, l2):
        G1, G2 = self.liaisons[l1[1]], self.liaisons[l2[1]]
        eq1, eq2 = self.eqs[l1[2][0]], self.eqs[l2[2][0]]
        (X1, Y1), (X2, Y2), D1, D2 = U(l1[2][1]), U(l2[0][1]), G1.dist if l1[2][0] == G1.sol1 else -G1.dist, G2.dist if l2[0][0] == G2.sol1 else -G2.dist
        X, Y = (X1 * Y2 - Y1 * X2) ** -1 * np.einsum(TAG, ((Y2, -X2), (-Y1, X1)), (Y1 * D1 - Y2 * D2, X2 * D2 - X1 * D1))
        theta = self.get_ref(l1[2][0]) - self.get_ref(l1[0][0]) + l1[2][1] - l1[0][1]
        M = R(theta)
        self.change_ref(eq2, theta, M, self.get_point(l1[2][0], 0) - np.einsum(TAG, M, self.get_point(l1[0][0])) + np.einsum(TAG, R(self.get_ref(l1[2][0])), (D1 * Y1 + X * X1, X * Y1 - D1 * X1)))
        G1.value = (X if G1.sol1 == l1[0][0] else -X)[0]
        G2.value = (Y if G2.sol1 == l2[2][0] else -Y)[0]
        return eq1 + eq2

    def P_P_P(self, c, l1, l2, l3):
        eq1, eq2, eq3 = self.eqs[l1[2][0]], self.eqs[l2[2][0]], self.eqs[l3[2][0]]
        print('eq1', eq1)
        print('eq2', eq2)
        print('eq3', eq3)
        P1P2, P1P3, P3P2 = self.get_point(*l2[0]) - self.get_point(*l1[2]), self.get_point(*l3[2]) - self.get_point(*l1[0]), self.get_point(*l2[2]) - self.get_point(*l3[0])
        mag12, mag13, mag32 = np.sum(P1P2 ** 2, axis=0), np.sum(P1P3 ** 2, axis=0), np.sum(P3P2 ** 2, axis=0)
        inv_mag12, inv_mag13 = mag12 ** -.5, mag13 ** -.5
        theta = self.sgns[c] * np.arccos((mag12 + mag13 - mag32) * .5 * inv_mag12 * inv_mag13)[0] + np.arccos(P1P3[0] * inv_mag13)[0] * (-2 * (P1P3[1, 0, :] > 0) + 1) - self.get_ref(l1[0][0]) + self.get_ref(l1[2][0]) + np.arccos(P1P2[0] * inv_mag12)[0] * (2 * (P1P2[1, 0, :] > 0) - 1)
        M = R(theta)
        self.change_ref(eq3, theta, M, self.get_point(l1[2][0], l1[2][1]) - np.einsum(TAG, M, self.get_point(l1[0][0], l1[0][1])))
        n_P3P2 = self.get_point(l2[0][0], l2[0][1]) - self.get_point(l3[2][0], l3[2][1])
        theta = np.arccos(n_P3P2[0] * np.sum(n_P3P2 ** 2, axis=0) ** -.5)[0] * (2 * (n_P3P2[1, 0, :] > 0) - 1) + self.get_ref(l1[2][0]) - self.get_ref(l3[0][0]) + np.arccos(P3P2[0] * mag32 ** -.5)[0] * (-2 * (P3P2[1, 0, :] > 0) + 1)
        M = R(theta)
        self.change_ref(eq2, theta, M, self.get_point(l3[2][0], l3[2][1]) - np.einsum(TAG, M, self.get_point(l3[0][0], l3[0][1])))
        self.liaisons[l1[1]].calculValue(self)
        self.liaisons[l2[1]].calculValue(self)
        self.liaisons[l3[1]].calculValue(self)
        return eq1 + eq2 + eq3

    def P_P_G(self, c, l1, l2, l3):
        P1, P2, G = self.liaisons[l1[1]], self.liaisons[l2[1]], self.liaisons[l3[1]]
        eq1, eq2, eq3 = self.eqs[l1[2][0]], self.eqs[l2[2][0]], self.eqs[l3[2][0]]
        P1P2 = self.get_point(l2[0][0], l2[0][1]) - self.get_point(l1[2][0], l1[2][1])
        mag12 = np.sum(P1P2 ** 2, axis=0)[0]
        inv_mag12 = mag12 ** -.5
        omega = np.arccos(P1P2[0][0] * inv_mag12) * (2 * (P1P2[1][0] > 0) - 1)
        X, Y = np.einsum(TAG, R(-self.get_ref(l3[2][0]) - l3[2][1]), np.einsum(TAG, R(self.get_ref(l3[2][0]) - self.get_ref(l3[0][0]) + l3[2][1] - l3[0][1]), self.get_point(l2[2][0], l2[2][1]) - self.get_point(l3[0][0], 0)) + self.get_point(l3[2][0], 0) - self.get_point(l1[0][0], l1[0][1]))[:, 0, :]
        Y += G.dist if G.sol1 == l3[2][0] else -G.dist
        dX = self.sgns[c] * np.sqrt(mag12 - Y ** 2) * (2 * (G.sol1 == l3[2][0]) - 1)
        gamma = np.arccos(dX * inv_mag12) * (2 * (Y > 0) - 1)
        theta2, theta3 = omega - self.get_ref(l3[0][0]) - l3[0][1] - gamma, omega - self.get_ref(l3[2][0]) - l3[2][1] - gamma
        M2, M3 = R(theta2), R(theta3)
        self.change_ref(eq2, theta2, M2, self.get_point(l2[0][0], l2[0][1]) - np.einsum(TAG, M2, self.get_point(l2[2][0], l2[2][1])))
        self.change_ref(eq3, theta3, M3, self.get_point(l1[2][0], l1[2][1]) - np.einsum(TAG, M3, self.get_point(l1[0][0], l1[0][1])))
        P1.calculValue(self)
        P2.calculValue(self)
        G.value = (dX - X) * (2 * (G.sol1 == l3[2][0]) - 1)
        return eq1 + eq2 + eq3

    def G_P_P(self, c, l1, l2, l3):
        G, P2, P3 = self.liaisons[l1[1]], self.liaisons[l2[1]], self.liaisons[l3[1]]
        eq1, eq2, eq3 = self.eqs[l1[2][0]], self.eqs[l2[2][0]], self.eqs[l3[2][0]]
        P2P3 = self.get_point(l3[0][0], l3[0][1]) - self.get_point(l2[2][0], l2[2][1])
        mag23 = np.sum(P2P3 ** 2, axis=0)[0]
        inv_mag23, theta3 = mag23 ** -.5, self.get_ref(l1[2][0]) + l1[2][1] - self.get_ref(l1[0][0]) - l1[0][1]
        omega, M3 = np.arccos(P2P3[0][0] * inv_mag23) * (2 * (P2P3[1][0] > 0) - 1),  R(theta3)
        X, Y = np.einsum(TAG, R(-self.get_ref(l1[2][0]) - l1[2][1]), np.einsum(TAG, M3, self.get_point(l3[2][0], l3[2][1]) - self.get_point(l1[0][0], 0)) + self.get_point(l1[2][0], 0) - self.get_point(l2[0][0], l2[0][1]))[:, 0, :]
        Y += G.dist if G.sol1 == l1[2][0] else -G.dist
        dX = self.sgns[c] * np.sqrt(mag23 - Y ** 2) * (2 * (G.sol1 == l1[2][0]) - 1)
        M2 = R(theta2 := (omega - np.arccos(dX * inv_mag23) * (2 * (Y > 0) - 1)))
        self.change_ref(eq2, theta2, M2, self.get_point(l2[0][0], l2[0][1]) - np.einsum(TAG, M2, self.get_point(l2[2][0], l2[2][1])))
        self.change_ref(eq3, theta3, M3, self.get_point(l3[0][0], l3[0][1]) - np.einsum(TAG, M3, self.get_point(l3[2][0], l3[2][1])))
        P2.calculValue(self)
        P3.calculValue(self)
        G.value = (dX - X) * (2 * (G.sol1 == l1[2][0]) - 1)
        return eq1 + eq2 + eq3

    def G_G_P(self, c, l1, l2, l3):
        G1, G2, P = self.liaisons[l1[1]], self.liaisons[l2[1]], self.liaisons[l3[1]]
        eq1, eq2, eq3 = self.eqs[l1[2][0]], self.eqs[l2[2][0]], self.eqs[l3[2][0]]
        (X1, Y1), (X2, Y2), D1, D2 = U(self.get_ref(l1[2][0]) + l1[2][1]), U(self.get_ref(l2[0][0]) + l2[0][1]), G1.dist if l1[2][0] == G1.sol1 else -G1.dist, G2.dist if l2[0][0] == G2.sol1 else -G2.dist
        M2, M3 = R(theta2 := self.get_ref(l1[2][0]) + l1[2][1] - l1[0][1] - self.get_ref(l1[0][0])), R(theta3 := self.get_ref(l2[0][0]) + l2[0][1] - l2[2][1] - self.get_ref(l2[2][0]))
        X, Y = (X1 * Y2 - Y1 * X2) ** -1 * np.einsum(TAG, ((Y2[0], -X2[0]), (-Y1[0], X1[0])), np.einsum(TAG, M3, self.get_point(*l3[2]) - self.get_point(l1[0][0], 0)) - np.einsum(TAG, M2, self.get_point(*l3[0]) - self.get_point(l2[2][0], 0)) + (D2 * Y2 - D1 * Y1, D1 * X1 - D2 * Y2))
        self.change_ref(eq3, theta3, M3, self.get_point(l1[2][0], 0) - np.einsum(TAG, M3, self.get_point(l1[0][0], 0)) - X * (X1, Y1))
        self.change_ref(eq2, theta2, M2, self.get_point(l2[0][0], 0) - np.einsum(TAG, M3, self.get_point(l2[2][0], 0)) + Y * (X2, Y2))
        P.calculValue(self)
        G1.value = (X if G1.sol1 == l1[0][0] else -X)[0]
        G2.value = (Y if G2.sol1 == l2[0][0] else -Y)[0]
        return eq1 + eq2 + eq3

    def P_G_G(self, c, l1, l2, l3):
        P, G2, G3 = self.liaisons[l1[1]], self.liaisons[l2[1]], self.liaisons[l3[1]]
        eq1, eq2, eq3 = self.eqs[l1[2][0]], self.eqs[l2[2][0]], self.eqs[l3[2][0]]
        (X1, Y1), (X2, Y2), D1, D2 = U(l2[0][1] + self.get_ref(l2[0][0])), U(l2[0][1] - l2[2][1] + l3[0][1] + self.get_ref(l2[0][0])), G2.dist if l2[0][0] == G2.sol1 else -G2.dist, G3.dist if l3[0][0] == G3.sol1 else -G3.dist
        M2, M3 = R(theta2 := self.get_ref(l2[0][0]) + l2[0][1] - l2[2][1] - self.get_ref(l2[2][0])), R(theta3 := self.get_ref(l2[0][0]) + l2[0][1] - l2[2][1] + l3[0][1] - l3[2][1] - self.get_ref(l3[2][0]))
        X, Y = (X1 * Y2 - Y1 * X2) ** -1 * np.einsum(TAG, ((Y2[0], -X2[0]), (-Y1[0], X1[0])), self.get_point(*l1[2]) - self.get_point(l2[0][0], 0) - np.einsum(TAG, M3, self.get_point(*l1[0]) - self.get_point(l3[2][0], 0)) + (D1 * Y1 + D2 * Y2, -D1 * X1 - D2 *  X2))
        self.change_ref(eq3, theta3, M3, self.get_point(*l1[2]) - np.einsum(TAG, M3, self.get_point(*l1[0])))
        self.change_ref(eq2, theta2, M2, self.get_point(l2[0][0], 0) - np.einsum(TAG, M2, self.get_point(l2[2][0], 0)) + (X * X1 - D1 * Y1, Y * Y1 + D1 * X1))
        P.calculValue(self)
        G2.value = (X if G2.sol1 == l2[0][0] else -X)[0]
        G3.value = (Y if G3.sol1 == l3[0][0] else -Y)[0]
        return eq1 + eq2 + eq3

    def Pilot(self, l):
        print(self.liaisons[l])
        print('eq1', self.eqs[self.liaisons[l].sol1])
        print('eq2', self.eqs[self.liaisons[l].sol2])
        return self.liaisons[l].input(self, self.inputs[self.piloted.index(l)])
    
    def pilot(self, liaisons_piloted):
        """
        Specifies the links piloted during the simulation.
        
        Parameters
        ----------
        liaisons_piloted : list or Liaison
            List of links controlled during the simulation
        """
        
        if isinstance(liaisons_piloted, Liaison):
            liaisons_piloted = [liaisons_piloted]
        self.piloted = tuple(list(self.liaisons).index(L) for L in liaisons_piloted)
        self.compiled_kinematic = False
        
    def block(self, liaisons_blocked):
        """
        Specifies the links blocked during the simulation.
        
        Parameters
        ----------
        liaisons_blocked : list or Liaison
            List of links blocked during the simulation
        """

        if isinstance(liaisons_blocked, Liaison):
            liaisons_blocked = [liaisons_blocked]
        self.blocked = tuple(list(self.liaisons).index(L) for L in liaisons_blocked)
        self.compiled_static = False
    
    def reset(self, n):
        self.eqs = [(i,) for i in range(len(self.sols))]
        for sol in self.sols:
            sol.reset(n)
            
    def solve_kinematic(self, inputs=None):
        """
        Performs the kinematic simulation.
        
        Parameters
        ----------
        inputs : list or ndarray, optionnal
            List of input, in the form of an array, for each piloted link. The order corresponds to the order in which the control specification was made. If only one link is controlled, it is possible to put the input array directly. If inputs is None, it means that the mechanism is blocked and that no links are controlled.
        """
        
        if self.compiled_kinematic:
            self.reset_points()
        else:
            self.compile_kinematic()
            
        if inputs is None:
            self.reset(1)
        else:
            if isinstance(inputs, list):
                inputs = np.array(inputs)
            if len(inputs.shape) == 1:
                inputs = inputs.reshape((1,) + inputs.shape)
            self.inputs = inputs
            self.reset(inputs[0].size)

        for method, args in self.instructions_kinematic:
            print()
            print(method)
            n_eq = self.__getattribute__(method)(*args)
            for sol in n_eq:
                self.eqs[sol] = n_eq
            #print(self.eqs)
        
        for L in self.liaisons:
            if isinstance(L, Pivot):
                L.point = self.sols[L.sol1].get_point(L.p1)
    
    def compile_static(self, sgns=None):          
        if self.compiled_kinematic:
            self.reset_points()
        else:
            self.compile_kinematic(sgns)
        
        if set(self.piloted) == set(self.blocked):
            self.instructions_static = self.instructions_kinematic[::-1]
            for i in range(len(self.instructions_static)):
                method = self.instructions_static[i][0]
                if method == 'Pilot':
                    self.instructions_static[i] = ('Block', self.instructions_static[i][1])
                else:
                    self.instructions_static[i] = (method + '_s', self.instructions_static[i][1])
        else:
            instructions = []
            
            for i, p in enumerate(self.blocked): # identification des L_b
                self.liaisons[p].blocked = True
            
            eqs, nodes = [(i,) for i in range(len(self.sols))], list(range(len(self.sols)))
            graph = [[] for _ in range(len(self.sols))]
            blocked = []
            
            for i, L in enumerate(self.liaisons): # création du graphe sans L_b
                if L.blocked:
                    blocked.append((L.sol1, L.sol2, i))
                else:
                    graph[L.sol1].append((L.sol2, i))
                    graph[L.sol2].append((L.sol1, i))
            
            dists = [float('inf')] * len(self.sols) # distances de chaque eq à 0
            dists[0] = 0
            distance(0, graph, dists, nodes) # parcours en largeur
            
            m_sols = [sol._points.shape[1] for sol in self.sols]
            while len(eqs) > 1:
                if blocked:
                    ip, dp = selectPiloted(blocked, dists, nodes)
                    c = chercheCycle(graph, nodes, dists, dp, self.liaisons)
                    if not dp or c is None:
                        s1, s2, l = blocked.pop(ip)
                        instructions.append(('Block', (l,)))
                        fusion(s1, s2, eqs, nodes, graph, dists)
                        continue
                else:
                    c = chercheCycle(graph, nodes, dists, len(graph), self.liaisons)
                arg = identify(tuple(self.liaisons[i].identifier(i) for i in c), nodes, eqs, m_sols)
                cycle = '_'.join(self.liaisons[i].id_ for _, i, _ in arg)
                if cycle in changed_arangements:
                    arg = change(arg)
                    cycle = '_'.join(self.liaisons[i].id_ for _, i, _ in arg)
                instructions.append((cycle + '_s', (c,) + arg))
                fusion(arg[0][0][0], arg[0][2][0], eqs, nodes, graph, dists)
                if len(c) == 3:
                    fusion(arg[0][0][0], arg[1][2][0], eqs, nodes, graph, dists)
        
            self.instructions_static = instructions[::-1]
            
        self.compiled_static = True
    
    def applie_gravity(self, angle=-np.pi/2, g=10):
        self.gravity = (True, angle, g)
        
    def generate_efforts(self, n):
        gravity, angle, g = self.gravity
        if gravity:
            for sol in self.sols:
                sol._forces = np.array([F() for F, _ in sol.forces]).reshape((len(sol.forces), 2, n)) # external efforts
                sol._couples = np.array([C() for C in sol.couples]).reshape((len(sol.forces), n))
                grav = np.repeat(U(angle)*sol.m*g, n, axis=1)
                sol._forces = np.concatenate((sol._forces, grav[np.newaxis])) # gravity force
                sol.index_forces += (sol.G_index,)
        else:
            for sol in self.sols:
                sol._forces = np.array([F() for F, _ in sol.forces]).reshape((len(sol.forces), 2, n)) # external efforts
                sol._couples = np.array([C() for C in sol.couples]).reshape((len(sol.forces), n))
                
        for L in self.liaisons:
            s1, s2 = self.sols[L.sol1], self.sols[L.sol2]
            if isinstance(L, Glissiere):
                for force in L.forces:
                    F = np.transpose(U(s1.angle + L.alpha1)*force(), (1, 0, 2)) # a verifier    
                    s1._forces = np.concatenate((s1._forces, F))
                    s1.index_forces += (L.p1,)
                    
                    s2._forces = np.concatenate((s2._forces, -F)) 
                    s2.index_forces += (L.p2,)
                    
            if isinstance(L, Pivot):
                for couples in L.couples:
                    C = couples()
                    s1._couples = np.concatenate((s1._couples, C[np.newaxis]))
                    
                    s2._couples = np.concatenate((s2._couples, -C[np.newaxis]))
                    
    def solve_static(self, inputs=None):
        if not self.compile_static:
            self.compile_static()
            
        self.solve_kinematic(inputs) # effectue reset_points

        self.generate_efforts(self.inputs[0].size) # generation mtn car les AM peuvent dependre de la position du meca
        
        eqs = [(i,) for i in range(len(self.sols))] # non il faut partir de la fin la ou les solides font tous partie de la mm eq
        self.EQS = [eqs.copy()]
        for method, args in self.instructions_static[:0:-1]:
            if len(args) == 1:
                L = self.liaisons[args[0]]
                n_eq = eqs[L.sol1] + eqs[L.sol2]
            else:
                c, l1, l2, l3 = args
                n_eq = eqs[l1[2][0]] + eqs[l2[2][0]] + eqs[l3[2][0]]
            for sol in n_eq:
                eqs[sol] = n_eq
            self.EQS.append(eqs.copy())
        self.EQS.reverse()
        print()
        print(len(self.instructions_static), len(self.EQS))
        for (method, args), eqs in zip(self.instructions_static, self.EQS):
            print()
            print(method)
            self.eqs = eqs
            self.__getattribute__(method)(*args)
     
    def Block(self, l):
        print(self.liaisons[l])
        print('eq1', self.eqs[self.liaisons[l].sol1])
        print('eq2', self.eqs[self.liaisons[l].sol2])
        return # à compléter!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!            
     
    def P_P_P_s(self, c, l1, l2, l3):
        eq1, eq2, eq3 = self.eqs[l1[2][0]], self.eqs[l2[2][0]], self.eqs[l3[2][0]]
        P1P3, P3P2 = self.get_point(*l3[2]) - self.get_point(*l1[0]), self.get_point(*l2[2]) - self.get_point(*l3[0])
        b, c = np.sqrt(np.sum(P1P3 ** 2, axis=0)), np.sqrt(np.sum(P3P2 ** 2, axis=0))
        print('eq1', eq1)
        print('eq2', eq2)
        print('eq3', eq3)
        # isoler eq2 :
        # TMS en P2 -> F1(P3)
        M = 0
        for s in eq2:
            sol = self.sols[s]
            P2Q = sol.get_point(np.array(sol.index_forces, int)) - self.get_point(*l2[2])
            print(self.get_point(*l2[2]))
            print(P2Q.shape)
            
            print(sol._couples.shape)
            M += np.sum(sol._couples, axis=0) + np.sum(sol._forces[0]*P2Q[1] - sol._forces[1]*P2Q[0], axis=0)
        F3 = -M/c # F(eq3->eq2)
        
        # isoler eq3 :
        # TMS en P1 -> F2(P3)
        M = 0
        for s in eq3:
            sol = self.sols[s]
            P1Q = sol.get_point(np.array(sol.index_forces, int)) - self.get_point(*l1[0])
            M += np.sum(sol._couples, axis=0) + np.sum(sol._forces[0]*P1Q[1] - sol._forces[1]*P1Q[0], axis=0)
        F3_ = M/b # F(eq3->eq2)
        
        # par composition (F1(P3), F2(P3)) -> F(P3)
        
        # isoler eq2 :
        # TRS -> F(P2)
        # appliquer F(P2) sur eq1
        
        # isoler eq3 :
        # TRS -> F(P1)
        # appliquer F(P1) sur eq1
        return
        
    compile_dynamic = lambda self, sgns=None : self.compile_static(sgns)
    
    def solve_dynamic(self, inputs, time=10.):
        """
        Performs the dynamic simulation.
        
        Parameters
        ----------
        inputs : list or ndarray, optionnal
            List of input, in the form of an array, for each piloted link. The order corresponds to the order in which the control specification was made. If only one link is controlled, it is possible to put the input array directly. If inputs is None, it means that the mechanism is blocked and that no links are controlled.
        time : float or int
            Duration of the simulation in seconds.
        """
        
        if not self.compile_static:
            self.compile_static()
            
        if isinstance(inputs, list):
            inputs = np.array(inputs)
        if len(inputs.shape) == 1:
            inputs = inputs.reshape((1,) + inputs.shape)
        
        d = inputs[:, 2] + 3*(inputs[:, 0] - inputs[:, 1])
        f = inputs[:, -3] + 3*(inputs[:, -1] - inputs[:, -2])
        kinematic_inputs = np.concatenate((d.reshape(d.shape + (1,)), inputs, f.reshape(f.shape + (1,))), axis=1)

        self.solve_kinematic(kinematic_inputs) # effectue reset_points
        
        self.generate_efforts(self.inputs[0].size)
        
        dtdt = (time/(len(inputs[0]) - 1))**2
        for sol in self.sols[1:]:
            L = sol.get_point(sol.G_index)
            _acc = (2*L[:, 1:-1] - L[:, 2:] - L[:, :-2])/dtdt # -acc translation
            sol._forces = np.concatenate((sol._forces, sol.m*_acc[np.newaxis])) 
            sol.index_forces += (sol.G_index,)
            # pb de shape du au calcul de l'acc
        
            L = t.make_continuous(sol.angle)
            _acc = (2*L[1:-1] - L[2:] - L[:-2])/dtdt # -acc rotation
            sol._couples = np.concatenate((sol._couples, sol.m*_acc[np.newaxis]))
            # pb de shape du au calcul de l'acc


        # prendre en compte la shape changer des params cinematiques
        # soit tt rogner mtn soit mettre en place un attribut qui permet d'éviter ces valeurs
        # ou encore simuler les 2 valeurs particulieres avant de resoudre le probleme
        
        # tt pareil que la resolution statique
        for method, args in self.instructions_static:
            pass # blablabla la résolution
        self.inputs = None   
        
    def add(self, obj):
        """
        Add to system *obj*.
        
        Parameters
        ----------
        obj : Solid or Linkage
        """
        
        if isinstance(obj, Solid):
            obj.index = len(self.sols)
            self.sols += (obj,)
        elif isinstance(obj, Liaison):
            if obj.sol1 == obj.sol2:
                print("You can't link a solid to itself !")
                return
            for L in self.liaisons:
                if L.sol1 == obj.sol1 and L.sol2 == obj.sol2:
                    print("The linkage graph must be minimal !")
                    return
            self.liaisons += (obj,)
        self.compiled = False
        return obj
    
    def reset_points(self):
        if self.sols[0]._points is None:
            sols_points = [[(0, 0)] + [point for _, point in sol.named_points.items()] for sol in self.sols]
            for L in self.liaisons:
                if isinstance(L, Pivot):
                    s1, s2 = L.sol1, L.sol2
                    try: # mise en place de l'index de p1
                        L.p1 = sols_points[s1].index(L.point_in_sol1)
                    except:
                        L.p1 = len(sols_points[s1])
                        sols_points[s1].append(L.point_in_sol1)
                        
                    try: # mise en place de l'index de p2
                        L.p2 = sols_points[s2].index(L.point_in_sol2)
                    except:
                        L.p2 = len(sols_points[s2])
                        sols_points[s2].append(L.point_in_sol2)
                
                elif isinstance(L, Glissiere): # PA des forces
                    s1, s2 = L.sol1, L.sol2
                    if L.forces != []:
                        point = U(L.alpha1 + np.pi/2)*L.dist1
                        point = (point[0, 0], point[1, 0])
                        try: # mise en place de l'index de p1
                            L.p1 = sols_points[s1].index(point)
                        except:
                            L.p1 = len(sols_points[s1])
                            sols_points[s1].append(point)
                            
                        point = U(L.alpha2 + np.pi/2)*L.dist2
                        point = (point[0, 0], point[1, 0])
                        try: # mise en place de l'index de p2
                            L.p2 = sols_points[s2].index(point)
                        except:
                            L.p2 = len(sols_points[s2])
                            sols_points[s2].append(point)
                        
            for sol, points in zip(self.sols, sols_points):
                for i in range(len(sol.forces)):
                    force, point = sol.forces[i]
                    try:
                        sol.index_forces += (points.index(point),)
                    except:
                        sol.index_forces += (len(points),)
                        points.append(point)
                    
                try:
                    sol.G_index = points.index(sol.G)
                except:
                    sol.G_index = len(points)
                    points.append(sol.G)
                
                # points = [origin, named_points,..., P and SP points,...,(pts d'app des F/couple,...), G]
                points = np.array(points, float).T
                sol._points = points.reshape(points.shape + (1,))
                
    def __repr__(self):
        return self.name if self.name else 'Unnamed system'




class Liaison:
    id_: str
    piloted, index = False, None
    blocked = False
    sol1: int
    sol2: int
    value: np.ndarray

    def get_value(self):
        return self.value

    def input(self, system: System, value: np.ndarray):
        pass

class Pivot(Liaison):
    """
    Create a Pivot object.
    
    Attributes
    ----------
    sol1 : int
        The index of the first solid which is the reference solid for the expression of the angle.
    sol2 :int
        The index of the second solid.
    p1 : tuple
        Coordinate of the pivot point in the frame of *sol1*.
    p2 : tuple
        Coordinate of the pivot point in the frame of *sol2*.
    point : ndarray
        Coordinates of the successive pivot points expressed in the global coordinate system after a simulation.
    angle : ndarray
        Successive values of the angle between *sol2* and *sol1* after a simulation.
    """
    
    id_ = 'P'
    def __init__(self, sol1, p1=(0, 0), sol2=None, p2=(0, 0)):        
        if isinstance(p1, int):
            self.p1, self.p2 = p1, p2
        else:
            self.point_in_sol1, self.point_in_sol2 = p1, p2
        self.sol1, self.sol2 = sol1, sol2
        self.value = 0
        self.point = 0
        self.couples = []

    def input(self, system: System, value: np.ndarray):
        self.value = value
        eq1, eq2 = system.eqs[self.sol1], system.eqs[self.sol2]
        sol1, sol2, p1, p2 = self.sol1, self.sol2, self.p1, self.p2
        if is_bati(eq2):
            sol1, sol2, p1, p2, value, eq1, eq2 = sol2, sol1, p2, p1, -value, eq2, eq1
        theta = system.sols[sol1].alpha - system.sols[sol2].alpha
        M = R(theta + value)
        system.change_ref(eq2, theta + value, M, system.get_point(sol1, p1) - np.einsum(TAG, R(theta), system.get_point(sol2, p2)))
        return eq1 + eq2

    def identifier(self, index):
        return (self.sol1, self.p1), index, (self.sol2, self.p2)

    def calculValue(self, system):
        self.value = system.get_ref(self.sol2) - system.get_ref(self.sol1)
    
    def applie_couple(self, couple):
        self.couples.append(couple)
    
    @property
    def angle(self):
        return self.value

    def __repr__(self):
        return f'Pivot {self.sol2}/{self.sol1}'



class Sphere_plan(Liaison):
    id_ = 'SP'
    def __init__(self, sol1, sol2, p1=(0, 0), alpha2=0, dist=0): # point : point de pivot dans sol1 et angle : angle de l'axe de la G dans sol2
        if isinstance(p1, int):
            self.p1 = p1
        else:
            self.point_in_sol1 = p1
        self.sol1, self.sol2 = sol1, sol2
        self.alpha2 = alpha2
        self.value = 0
        self.forces, self.couples = [], []
        
    def input(self, system: System, value: np.ndarray):
        pass # blablablalablblalbllalbllablblallblbalabllbalblalabllballballbllablbllabllblballablblblalbal
    
    def identifier(self, index):
        return (self.sol1, self.p1), index, (self.sol2, (self.alpha2, self.dist))
        
    def applie_couple(self, couple):
        self.couples.append(couple)
    
    def applie_force(self, force): # F(sol2 -> sol1)
        self.forces.append(force)
        
    @property
    def angle(self):
        return self.value2 # value correspondra a la glissiere

    def __repr__(self):
        return f'Sphere-plan {self.sol2}/{self.sol1}'


class Glissiere(Liaison):
    """
    Create a Slide object.
    
    Attributes
    ----------
    sol1 : int
        The index of the first solid which is the reference solid for the expression of the *value*.
    sol2 :int
        The index of the second solid.
    angle1 : tuple
        Angle of the vector directing the slide expressed in the basis of *sol1*.
    angle2 : tuple
        Angle of the vector directing the slide expressed in the basis of *sol2*.
    dist : float
        Distance between the origins of the reference marks of *sol1* and *sol2* along the normal of the axis of the slide. Can be negative, *sol1* is the reference.
    value : ndarray
        Successive values of the distance between the origins of the *sol1* and *sol2* reference marks along the slide axis. Can be negative, *sol1* is the reference.
    """
    
    id_ ='G'
    def __init__(self, sol1, alpha1=0, sol2=None, alpha2=0, dist1=0, dist2=0):
        self.sol1, self.sol2, self.alpha1, self.alpha2, self.dist1, self.dist2 = sol1, sol2, alpha1, alpha2, dist1, dist2
        self.value = 0
        self.forces = []
        self.p1, self.p2 = None, None

    @property
    def dist(self):
        return self.dist1 - self.dist2

    def input(self, system: System, value: np.ndarray):
        self.value = value
        eq1, eq2 = system.eqs[self.sol1], system.eqs[self.sol2]
        sol1, sol2, a1, a2, d = self.sol1, self.sol2, self.alpha1, self.alpha2, self.dist
        if is_bati(eq2):
            sol1, sol2, a1, a2, d, value, eq1, eq2 = sol2, sol1, a2, a1, -d, -value, eq2, eq1
        theta = system.sols[sol1].alpha - system.sols[sol2].alpha + a1
        M = R(theta)
        system.change_ref(eq2, theta, M, system.get_point(sol1, 0) - np.einsum(TAG, M, system.get_point(sol2, 0)) + d * U((a1 + np.pi/2,)) + value * U((a1,)))
        return eq1 + eq2
    
    def applie_force(self, force): # F(sol2 -> sol1)
        self.forces.append(force)
        
    def identifier(self, index):
        return (self.sol1, self.alpha1), index, (self.sol2, self.alpha2)

    def __repr__(self):
        return f'Glissiere {self.sol2}/{self.sol1}'




if __name__ == '__main__':
    # loic
    SOLIDS = (
        Solid(np.array([[0., 5.], [0., 0.]])),
        Solid(np.array([[0., 1., 5.], [0., 0., 0.]])),
        Solid(np.array([[0., 8.], [0., 0.]])),
        Solid(np.array([[0., 10.], [0., 0.]]))
    )

    LIAISONS = (
        Pivot(0, 0, 1, 0),
        Pivot(1, 1, 2, 0),
        Glissiere(2, np.pi/2, 3, np.pi/2, 0.),
        Glissiere(3, 0., 0, 0., -7.)
    )

    PILOTED = (0,)

    S = System(SOLIDS, LIAISONS, PILOTED)
    S.compile_kinematic()
    S.solve_kinematic(np.linspace(0, 2 * np.pi, 81).reshape((1, 81)))
    plt.plot(list(range(81)), LIAISONS[3].get_value())
    A = SOLIDS[1].get_point(2)
    plt.plot(A[0], A[1])
    plt.show()
    
    
    
    # val
    S = System()
    
    S.add(s1 := Solid())
    S.add(s2 := Solid())
    S.add(s3 := Solid())
    s1['B'] = (5, 0)
    
    S.add(P1 := Pivot(0, (0, 0), 1, (0, 0)))
    S.add(P2 := Pivot(1, (1, 0), 2, (0, 0)))
    S.add(G1 := Glissiere(2, np.pi/2, 3, np.pi/2, 0.))
    S.add(G2 := Glissiere(3, 0., 0, 0., -7.))
    
    S.pilot(P1)
    
    S.compile_kinematic()
    S.solve_kinematic(np.linspace(0, 2 * np.pi, 81))
    plt.plot(list(range(81)), G2.value)
    B = s1.get_point('B')
    plt.plot(B[0], B[1])
    plt.show()
