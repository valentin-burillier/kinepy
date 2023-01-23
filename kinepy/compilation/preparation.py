def make_sets(system, joint_set):
    solids = list(system.sols)
    joint_dict = dict()
    ghosted = []
    ghost_cnt = 0

    for joint in system.joints:
        if joint in joint_set or joint.dof > 2:
            continue
        if joint.dof == 1:
            joint_dict[joint.rep] = joint
            continue

        joint.ghost_j1.rep = joint.rep
        joint_dict[joint.rep] = joint.ghost_j1

        joint.ghost_j2.rep = len(system.joints) + ghost_cnt
        joint_dict[joint.ghost_j2.rep] = joint.ghost_j2
        ghost_cnt += 1

        joint.ghost_sol.rep = len(solids)
        solids.append(joint.ghost_sol)
        ghosted.append(joint)

    return solids, joint_dict, ghosted


def make_joint_graph(joints, n):
    joint_graph = [[(0, 0)] * n for _ in range(n)]
    for index, joint in joints.items():
        joint_graph[joint.s1.rep][joint.s2.rep] = joint_graph[joint.s2.rep][joint.s1.rep] = (joint.id_, index)
    return joint_graph


def make_relation_graph(relations, n):
    relation_graph = [[] for _ in range(n)]
    for rel in relations:
        j1, j2 = rel.j1.rep, rel.j2.rep
        relation_graph[j1].append((j2, rel, True))
        relation_graph[j2].append((j1, rel, False))
    return relation_graph
