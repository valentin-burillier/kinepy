from kinepy.linkage import RevoluteJoint, PrismaticJoint, PinSlotJoint, RectangularJoint

KINEMATICS, DYNAMICS, BOTH = 0, 1, 2
MAX_CYCLE = 3


class CompilationError(Exception):
    pass


def make_graph(system):
    graph: list[list[tuple[int, int]]] = [list() for _ in system.sols]
    for i, l_ in enumerate(system.joints):
        graph[l_.s1].append((l_.s2, i))
        graph[l_.s2].append((l_.s1, i))
    return graph


def set_distances(graph, eqs):
    d = [-1] * len(graph)
    d[0] = 0
    queue = [0]
    while queue:
        c = queue.pop(0)
        for s, _ in graph[c]:
            e = eqs[s]
            if d[e] == -1:
                d[e] = d[c] + 1
                queue.append(e)
    return d


def closest_piloted_joint(system, eqs, d, mode):
    d_min, index, eq_ = float('inf'), None, ()
    for l_i in (system.piloted, system.blocked, system.piloted)[mode]:
        l_ = system.joints[l_i]
        if (eq1 := eqs[l_.s1]) != (eq2 := eqs[l_.s2]) and min(d[eq1], d[eq2]) < d_min:
            d_min, index, eq_ = min(d[eq1], d[eq2]), l_i, ((eq1, l_.s1), (eq2, l_.s2))[::(2 * (d[eq1] < d[eq2]) - 1)]
            if d_min == 0:
                return d_min, index, eq_
    return d_min, index, eq_


def hash_cycle(cycle):  # maximum is 3-cycle
    return sum((MAX_CYCLE+1) ** l_.id_ for l_ in cycle)


valid_cycles = {
    hash_cycle((RevoluteJoint, RevoluteJoint, RevoluteJoint)),
    hash_cycle((RevoluteJoint, RevoluteJoint, PrismaticJoint)),
    hash_cycle((RevoluteJoint, PrismaticJoint, PrismaticJoint)),
    hash_cycle((PinSlotJoint, RevoluteJoint)),
    hash_cycle((PinSlotJoint, PrismaticJoint)),
    hash_cycle((RectangularJoint, RevoluteJoint))
}

signed_cycles = {
    hash_cycle((RevoluteJoint, RevoluteJoint, RevoluteJoint)),
    hash_cycle((RevoluteJoint, RevoluteJoint, PrismaticJoint)),
    hash_cycle((RevoluteJoint, PinSlotJoint))
}


def find_cycle(system, eqs, graph, d):
    queue = [0]
    closed = [False] * len(graph)
    while queue:
        c = queue.pop(0)
        nbs = {}
        for s, l_ in graph[c]:
            eq = eqs[s]
            if closed[eq]:
                continue
            # 2-cycle
            if eq in nbs and (h := hash_cycle((system.joints[nbs[eq]], system.joints[l_]))) in valid_cycles:
                return d[c], (nbs[eq], l_), (c, eq), h in signed_cycles
            if eq in nbs:
                raise CompilationError(f'Invalid Hyper-static 2-cycle encountered: {(nbs[eq], l_)}')
            nbs[eq] = l_
        # 3-cycle
        for nb, l1 in nbs.items():
            for s, l_ in graph[nb]:
                e = eqs[s]
                if closed[e]:
                    continue
                if e in nbs and (h := hash_cycle(tuple(system.joints[i] for i in (l1, nbs[e], l_)))) in valid_cycles:
                    return d[c], (l1, nbs[e], l_), (c, e, nb), h in signed_cycles
            queue.append(nb)
        closed[c] = True
    return float('inf'), None, (), False


def next_step(system, eqs, graph, d, mode):
    d1, i1, eq1 = closest_piloted_joint(system, eqs, d, mode)
    if not d1:
        # pilot that joint
        return i1, eq1, False
    d2, cycle, eq2, sgn = find_cycle(system, eqs, graph, d)
    if d1 < d2:
        # pilot the joint
        return i1, eq1, False
    # solve the cycle
    if d2 == float('inf'):
        graph_text = "\n".join(str(i) for i in graph)
        raise CompilationError(f'Nothing found in graph:\n{graph_text}\nEqs: {eqs}')
    return cycle, eq2, sgn


def align(cycle, cycle_eqs, cycle_indices, eqs):
    n_cycle = tuple(
        l_[::(2 * (eqs[l_[1][0]] == cycle_eqs[i]) - 1)] for i, l_ in enumerate((l_.identifier for l_ in cycle))
    )
    if len(cycle) == 2:
        if cycle[0].id_ < cycle[1].id_:
            return (n_cycle[1][::-1], n_cycle[0][::-1]), cycle_eqs, cycle_indices[::-1]
        return n_cycle, cycle_eqs, cycle_indices
    if cycle[0].id_ == cycle[1].id_ or cycle[1].id_ == cycle[2].id_:
        return n_cycle, cycle_eqs, cycle_indices
    n_cycle = n_cycle[1][::-1], n_cycle[0][::-1], n_cycle[2][::-1]
    cycle_eqs = cycle_eqs[0], cycle_eqs[2], cycle_eqs[1]
    cycle_indices = cycle_indices[1], cycle_indices[0], cycle_indices[2]
    return n_cycle, cycle_eqs, cycle_indices


def eq_union(eq, graph, eqs, _eqs):
    graph[eq[0]] = sum((tuple((s, l_) for s, l_ in graph[i] if eqs[s] not in eq) for i in eq), ())
    _eqs[eq[0]] = sum((_eqs[i] for i in eq), ())
    for i in sorted(eq[1:], reverse=True):
        for s in _eqs[i]:
            eqs[s] = eq[0]
        if i < len(graph) - 1:
            graph[i] = graph.pop()
            _eqs[i] = _eqs.pop()
            for s in _eqs[i]:
                eqs[s] = i
        else:
            graph.pop()
            _eqs.pop()


def compiler(system, mode=KINEMATICS):
    eqs, _eqs = list(range(len(system.sols))), list((i,) for i in range(len(system.sols)))
    graph = make_graph(system)
    kin_instr, dyn_instr = [], []
    d = set_distances(graph, eqs)
    final = [0] * len(eqs)
    while eqs != final:
        cycle, eq, signed = next_step(system, eqs, graph, d, mode)
        if isinstance(cycle, int):
            eq_, eq = tuple((_eqs[i], s) for i, s in eq), tuple(i for i, _ in eq)
            kin_instr.append(('Pilot', cycle))
            dyn_instr.insert(0, ('Block', cycle) + eq_)
        else:
            cycle, eq, cycle_indices = align(tuple(system.joints[i] for i in cycle), eq, cycle, eqs)
            eq_ = tuple(_eqs[i] for i in eq)
            tag = '_'.join(system.joints[l_].tag for l_ in cycle_indices)
            kin_instr.append((tag, cycle_indices,) + cycle)
            dyn_instr.insert(0, (tag, cycle_indices,) + cycle + eq_)

            if signed and cycle_indices not in system.signs:
                system.signs[cycle_indices] = 1
                print(f'Identified new signed cycle: {cycle_indices} ({tag}).\nChosen 1 as sign.')
        eq_union(eq, graph, eqs, _eqs)
        d = set_distances(graph, eqs)
    return (kin_instr, dyn_instr, (kin_instr, dyn_instr))[mode]
