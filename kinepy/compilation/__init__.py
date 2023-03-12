from kinepy.compilation.graph_operations import isomorph, degrees, distances, vertices_fusion
from kinepy.compilation.graphs import GRAPHS, DEGREES, SOLVED, NAMES, EDGES, SIGNS
from kinepy.compilation.preparation import *
from kinepy.objects.joints import GhostRevolute, GhostPrismatic


DYNAMICS, KINEMATICS, BOTH = 0, 1, 2

SOLVE_PILOT, SOLVE_GRAPH, SOLVE_RELATION, CSA, SET_ORIGIN, REC_GHOSTS = range(6)
SOLVE_BLOCK, _, _, COMPUTE_MA, COMPUTE_INERTIA, _ = range(6)


class CompilationError(Exception):
    pass


def match_graph(joint_graph):
    dest_deg = degrees(joint_graph)
    for index, (graph, src_deg) in enumerate(zip(GRAPHS, DEGREES)):
        vertex_map = isomorph(graph, joint_graph, src_deg, dest_deg, ())
        if vertex_map:
            return index, vertex_map
    s = '\n'.join(map(str, joint_graph))
    raise CompilationError(f"None of the known graphs matches the current system graph \n{s}\n"
                           f"Either it is hyperstatic or it is not supported")


def get_eq(vertex, sols, eqs):
    return tuple(sols[i] for i in eqs[vertex])


def manage_relations(relation_graph, solved_joints, joint_queue):
    while joint_queue:
        joint1 = joint_queue.pop(0)
        for joint2, relation, direction in relation_graph[joint1.rep]:
            if solved_joints[joint2.rep]:
                raise CompilationError("Encountered hyperstatic relation scheme")
            solved_joints[joint2.rep] = True
            joint_queue.append(joint2)

            for i, (_, rel2, _) in enumerate(relation_graph[joint2.rep]):
                if rel2 is relation:
                    relation_graph[joint2.rep].pop(i)
                    break

            yield relation, direction
        relation_graph[joint1.rep] = []


def solve_piloted_joint(joint, sol_to_vertex, sols, eqs, dyn_instr, dist, joint_graph, eq_order):
    vertex1, vertex2 = sol_to_vertex[joint.s1.rep], sol_to_vertex[joint.s2.rep]
    eq1, eq2 = get_eq(vertex1, sols, eqs), get_eq(vertex2, sols, eqs)

    eq_order.append((eq1, eq2))
    dyn_instr.insert(0, (SOLVE_BLOCK, joint, eq1, eq2, dist[vertex2] < dist[vertex1]))

    vertices_fusion(joint_graph, (min(vertex1, vertex2), max(vertex1, vertex2)), eqs, sol_to_vertex)


def solve_relation(relation, direction, sol_to_vertex, sols, eqs, kin_instr, dyn_instr, dist, system, joint_graph):
    joint = (relation.j1, relation.j2)[direction]
    vertex1, vertex2 = sol_to_vertex[joint.s1.rep], sol_to_vertex[joint.s2.rep]
    eq1, eq2 = get_eq(vertex1, sols, eqs), get_eq(vertex2, sols, eqs)

    kin_instr.append((SOLVE_RELATION, relation, direction, eq1, eq2))
    dyn_instr.insert(0, (SOLVE_RELATION, relation, direction, eq1, eq2, dist[vertex2] < dist[vertex1]))

    vertices_fusion(joint_graph, (min(vertex1, vertex2), max(vertex1, vertex2)), eqs, sol_to_vertex)


def compiler(system, mode):
    kin_instr, dyn_instr = [], []
    joints, sols = ((system.dyn_joints, system.dyn_sols), (system.kin_joints, system.kin_sols))[not mode]

    joint_graph = make_joint_graph(joints, len(sols))
    relation_graph = make_relation_graph(system.relations, len(system.joints))

    solved_joints = [False] * len(system.joints)

    eqs = [(i,) for i in range(len(sols))]
    sol_to_vertex = list(range(len(sols)))

    dist = distances(joint_graph)

    joint_queue = list((system.blocked, system.piloted)[not not mode])

    # piloted/blocked joints are solved
    eq_order = []
    for joint in joint_queue:
        solved_joints[joint.rep] = True
        joint = system.joints[joint.rep]
        solve_piloted_joint(joint, sol_to_vertex, sols, eqs, dyn_instr, dist, joint_graph, eq_order)
        dist = distances(joint_graph)
    kin_instr.append((SOLVE_PILOT, system, tuple(eq_order)))

    if mode:
        print("Compiling\n")
        print("Step 1: Solving piloted joints\n")

    # until there is only one class
    while len(joint_graph) > 1:

        # runs through the relation graph for all joints that are solved
        for relation, direction in manage_relations(relation_graph, solved_joints, joint_queue):
            solve_relation(relation, direction, sol_to_vertex, sols, eqs, kin_instr, dyn_instr, dist, system, joint_graph)
            dist = distances(joint_graph)

        # looks for an Assur group
        index, vertex_map = match_graph(joint_graph)
        concerned_eqs = tuple(get_eq(vertex, sols, eqs) for vertex in vertex_map)

        # 0 is the only problem as a ref, even if min is not necessary, it is uniform
        ref = min(enumerate(vertex_map), key=lambda x: dist[x[1]])[0]

        if not SOLVED[index]:
            raise CompilationError(f"Sorry, unable to solve the graph {NAMES[index]} (n° {index}).\n"
                                   f"See on [explanation page] for further information")
        concerned_joints, key = [], []
        for src, dest in map(lambda x: (vertex_map[x[0]], vertex_map[x[1]]), EDGES[index]):
            joint = joints[joint_graph[src][dest][1]]

            # a joint and a boolean telling if the joint is directed according to the corresponding edge in the source
            # graph
            concerned_joints.append((joint, joint.s1.rep in eqs[src]))
            key.append(joint.rep)
            if not isinstance(joint, (GhostRevolute, GhostPrismatic)):
                solved_joints[joint.rep] = True
                joint_queue.append(joint)

        sgn = SIGNS[index]
        key, concerned_joints = tuple(key), tuple(concerned_joints)

        js = set(j.master if isinstance(j, (GhostRevolute, GhostPrismatic)) else j for j in (joints[i] for i in key))
        if sgn and mode:
            sgn_key = f'{len(kin_instr)} {NAMES[index]}'
            print(f"Step {len(kin_instr)}: Identified new signed group from graph {NAMES[index]} "
                  f"(n°{index}) with joints {', '.join(repr(j) for j in js)}.\nChosen {sgn} as sign.\n"
                  f"Sign key: {sgn_key}.\n")
            system.signs[sgn_key] = sgn
            system.tags[key] = sgn_key
        elif mode:
            print(f"Step {len(kin_instr)}: Solving new group from graph {NAMES[index]} "
                  f"(n°{index}) with joints {', '.join(repr(j) for j in js)}.\nChosen {sgn} as sign.\n")
            system.tags[key] = ()

        kin_instr.append((SOLVE_GRAPH, system, index, concerned_eqs, concerned_joints))
        dyn_instr.insert(0, (SOLVE_GRAPH, index, concerned_eqs, concerned_joints, ref))

        vertices_fusion(joint_graph, sorted(vertex_map), eqs, sol_to_vertex)
        dist = distances(joint_graph)

    #  make solid angles continous, restore 0 as the origin, remove ghosts
    kin_instr += [(CSA, system), (SET_ORIGIN, system), (REC_GHOSTS, system)]
    dyn_instr = [(COMPUTE_INERTIA, system), (COMPUTE_MA, system)] + dyn_instr + [(REC_GHOSTS, system)] # noqa

    return (dyn_instr, kin_instr, (kin_instr, dyn_instr))[mode]
