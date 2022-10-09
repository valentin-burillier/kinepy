from kinepy.graphs import GRAPHS, EDGES, SOLVED, SIGNS
from kinepy.metajoints import LinearRelationBase
from kinepy.linkage import Ghost


SOLVE_PILOT, SOLVE_GRAPH, SOLVE_RELATION, CSA, SET_ORIGIN = range(5)


class CompilationError(Exception):
    pass


def distances(graph):
    dists = [0] * len(graph)
    visited = [False] * len(graph)
    queue = [0]
    visited[0] = True
    while queue:
        c = queue.pop(0)
        for i, (v, _) in enumerate(graph[c]):
            if v and not visited[i]:
                visited[i] = True
                dists[i] = dists[c] + 1
                queue.append(i)
    return dists


def degrees(graph):
    res = tuple([0, 0] for _ in range(len(graph)))
    for i, l in enumerate(graph):
        for j in l:
            if j:
                res[i][j-1] += 1
    return res


def degrees2(graph):
    res = tuple([0, 0] for _ in range(len(graph)))
    for i, l in enumerate(graph):
        for j in l:
            if j[0]:
                res[i][j[0]-1] += 1
    return res


def inf(deg1, deg2):
    return deg1[0] <= deg2[0] and deg1[1] <= deg2[1]


DEGREES = tuple(degrees(g) for g in GRAPHS)


def can_match(g1, g2, f):  # just testing the last added-point
    n = len(f) - 1
    for i, j in enumerate(f[:-1]):
        if g1[i][n] != g2[j][f[n]][0]:
            return False
    return True


def isomorph(g1, g2, d1, d2, f):
    if len(f) == len(g1):
        return f
    for v in range(len(g2)):
        if v not in f and inf(d1[len(f)], d2[v]) and can_match(g1, g2, f + (v,)):
            f_ = isomorph(g1, g2, d1, d2, f + (v,))
            if f_:
                return f_
    return ()


def search_graph(g):
    d = degrees2(g)
    for i, g_ in enumerate(GRAPHS):
        f_ = list(isomorph(g_, g, DEGREES[i], d, ()))
        if f_:
            return i, f_
    raise CompilationError(f"Nothing found in graph {g}")


def _fusion(graph, a, b, eqs, sol_to_eqs):
    #  a < b
    for i in range(len(graph)):
        if graph[b][i][0] and i != a:
            graph[a][i] = graph[i][a] = graph[b][i]
    if b == len(graph) - 1:
        for line in graph:
            line.pop()
    else:
        for i, line in enumerate(graph[:-1]):
            c = line.pop()
            line[b] = c
            graph[b][i] = c
        eqs[b], eqs[-1] = eqs[-1], eqs[b]
        for s in eqs[b]:
            sol_to_eqs[s] = b
    graph.pop()
    e = eqs.pop()
    eqs[a] += e
    for s in e:
        sol_to_eqs[s] = a


def fusion(graph, eq, eqs, sol_to_eqs):
    mcr = min(eq)
    for i in sorted(eq, reverse=True)[:-1]:
        _fusion(graph, mcr, i, eqs, sol_to_eqs)


def make_sets(system, check):
    lst = list(system.sols)
    dct = dict()
    cnt = 0
    for j in system.joints:
        if j.dof == 1:
            dct[j.rep] = j
        if j.dof == 2 and j.rep not in check:
            j.ghost_j1.rep = j.rep
            dct[j.rep] = j.ghost_j1
            j.ghost_j2.rep = len(system.joints) + cnt
            cnt += 1
            dct[j.ghost_j2.rep] = j.ghost_j2
            j.ghost_sol.rep = len(lst)
            lst.append(j.ghost_sol)
    return lst, dct


Z = 0., 0.


def make_graph(joints, n):
    graph = [[Z] * n for _ in range(n)]
    for i, j in joints.items():
        graph[j.s1.rep][j.s2.rep] = graph[j.s2.rep][j.s1.rep] = (j.id_, i)
    return graph


def make_joint_graph(relations, n):
    graph: list[list[tuple[int, LinearRelationBase, bool]]] = [[] for _ in range(n)]
    for rel in relations:
        j1, j2 = rel.j1.rep, rel.j2.rep
        graph[j1].append((j2, rel, True))
        graph[j2].append((j1, rel, False))
    return graph


DYNAMICS, KINEMATICS, BOTH = 0, 1, 2


def compile_relations(joint_graph, solved, queue):
    instruction = []
    while queue:
        j1 = queue.pop(0)
        for j2, rel, b2 in joint_graph[j1]:
            if solved[j2]:
                raise CompilationError("Encountered hyperstatic relation scheme")
            solved[j2] = True
            instruction.append((rel, b2))
            queue.append(j2)
            for i, (_, rel2, _) in enumerate(joint_graph[j2]):
                if rel2 is rel:
                    break
            else:
                continue
            joint_graph[j2].pop(i)
        joint_graph[j1] = []
    return instruction


def compiler(system, mode=KINEMATICS):
    kin_instr, dyn_instr = [], []
    joints, sols = (
        (system.dyn_solving_joints, system.dyn_solving_sols),
        (system.kin_solving_joints, system.kin_solving_sols)
    )[not mode]

    graph = make_graph(joints, len(sols))
    joint_graph = make_joint_graph(system.relations, len(system.joints))
    solved = [False] * len(system.joints)
    queue = (system.blocked, system.piloted)[not not mode]

    eqs = [(i,) for i in range(len(sols))]
    sol_to_eqs = list(range(len(sols)))

    dist = distances(graph)

    # solving for piloted/block
    for j in queue:
        solved[j] = True
        j = system.joints[j]

        e1, e2 = sol_to_eqs[j.s1.rep], sol_to_eqs[j.s2.rep]
        eq1, eq2 = tuple(sols[s] for s in eqs[e1]), tuple(sols[s] for s in eqs[e2])
        kin_instr.append((SOLVE_PILOT, system, j, eq1, eq2))
        dyn_instr.insert(0, (SOLVE_PILOT, j, eq1, eq2, dist[e2] < dist[e1]))

        fusion(graph, (sol_to_eqs[j.s1.rep], sol_to_eqs[j.s2.rep]), eqs, sol_to_eqs)
        dist = distances(graph)

    while len(graph) > 1:
        rel_instr = compile_relations(joint_graph, solved, queue)

        #  relation resolution
        for rel, b in rel_instr:
            j = (rel.j1, rel.j2)[b]
            e1, e2 = sol_to_eqs[j.s1.rep], sol_to_eqs[j.s2.rep]
            ref = dist[e1] > dist[e2]

            kin_instr.append((SOLVE_RELATION, rel, b))
            dyn_instr.insert(0, (SOLVE_RELATION, rel, b, eqs[e1], eqs[e2], ref))

            fusion(graph, (sol_to_eqs[j.s1.rep], sol_to_eqs[j.s1.rep]), eqs, sol_to_eqs)
            dist = distances(graph)

        index, eqs_ = search_graph(graph)
        eqs__ = tuple(tuple(sols[s] for s in eqs[i]) for i in eqs_)
        ref = min(enumerate(eqs_), key=(lambda x: dist[x[1]]))[0]

        if not SOLVED[index]:
            raise CompilationError(f"Sorry, unable to solve the graph, sub-graph index {index}.\n"
                                   f"See on [explanation page] for further information")

        #  joints and direction (True: According to edge, False: Opposed to edge
        js = tuple(((j_ := joints[graph[eqs_[i]][eqs_[j]][1]]), j_.s1.rep in eqs[eqs_[i]]) for i, j in EDGES[index])
        for j, _ in js:
            if not isinstance(j, Ghost):
                solved[j.rep] = True
                queue.append(j.rep)

        sgn = SIGNS[index] if SIGNS[index] is not None else None
        if sgn is not None and mode:
            key = tuple(j.rep for j, _ in js)
            print(f"Identified new signed group: graph n°{index} with joints {key}.\nChosen {sgn} as sign.")
            system.signs[key] = sgn

        kin_instr.append((SOLVE_GRAPH, system, index, eqs__, js))
        dyn_instr.insert(0, (SOLVE_GRAPH, index, eqs__, js, ref))

        fusion(graph, eqs_, eqs, sol_to_eqs)
        dist = distances(graph)

    kin_instr.append((CSA, system))
    kin_instr.append((SET_ORIGIN, system))

    return (dyn_instr, kin_instr, (kin_instr, dyn_instr))[mode]
