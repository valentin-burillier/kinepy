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



