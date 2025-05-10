import collections.abc
import numpy as np

import kinepy.objects.config as cfg
import kinepy.strategy.types as types
import kinepy.strategy.graph_data as gd
import kinepy.exceptions as ex


def make_joint_graph(config: cfg.Config) -> types.StrategyJointState:
    """
    Create adjacency matrix for joint graph, initial eqs, initial eq mapping (solid_index -> eq_index), compute initial value of degrees
    """
    n_solid = config.solids.count
    result_graph: types.JointGraph = [[types.JointGraphNode(cfg.Joints.Type.EMPTY) for _ in range(n_solid)] for _ in range(n_solid)]

    for index, (_type, (_s1, _s2)) in enumerate(zip(config.joints.type_, config.joints.solids)):
        result_graph[_s1][_s2].set(index, cfg.Joints.Type(_type).primitive())
        result_graph[_s2][_s1].set(index, cfg.Joints.Type(_type).primitive())

    return result_graph, tuple((i,) for i in range(n_solid)), tuple(range(n_solid))


def make_relation_graph(config: cfg.Config) -> types.RelationGraph:
    """
    Create adjacency lists for the relation graph
    """
    n_joint = config.joints.count
    result_graph: types.RelationGraph = [[] for _ in range(n_joint)]

    for index, (_j1, _j2) in enumerate(config.relations.joints):
        result_graph[_j1].append(r1 := types.RelationGraphNode(True, index))
        result_graph[_j2].append(r2 := types.RelationGraphNode(False, index))

        r1.pair = r2
        r2.pair = r1

    return result_graph


def compute_degrees(joint_graph: types.JointGraph) -> types.Degrees:
    degrees: list[tuple[int, int]] = []
    for adj in joint_graph:
        results = [0, 0, 0]
        for node in adj:
            results[node.node_type.value] += 1
        degrees.append((results[cfg.Joints.Type.REVOLUTE.value], results[cfg.Joints.Type.PRISMATIC.value]))

    return tuple(degrees)


def merge(joint_graph: types.JointGraph, eqs: types.Eq, group_to_merge: tuple[int, ...]) -> types.StrategyJointState:
    merge_state: list[bool] = [False] * len(eqs)
    for index in group_to_merge:
        merge_state[index] = True
    target_eq: int = min(group_to_merge)

    new_eqs: list[tuple[int, ...]] = []
    for index, eq in enumerate(eqs):
        if index <= target_eq or not merge_state[index]:
            new_eqs.append(eq)
        else:
            new_eqs[target_eq] += eq
    new_eqs: types.Eq = tuple(new_eqs)

    # region joint_graph merge

    new_joint_graph: types.JointGraph = [[types.JointGraphNode(cfg.Joints.Type.EMPTY) for _ in new_eqs] for _ in new_eqs]

    # number of merging vertices met after the merging target; index displacement of non-merging vertices when met
    merge_count = 0

    for x in range(len(eqs)):
        merge_count += merge_state[x] - (x == target_eq)
        new_x = target_eq if merge_state[x] else x - merge_count
        _old_merge_count = merge_count

        for y in range(x+1, len(eqs)):
            merge_count += merge_state[y] - (y == target_eq)
            new_y = target_eq if merge_state[y] else y - merge_count

            # edges in the merging group are not ported (empty diagonals)
            if new_x == new_y or joint_graph[x][y].node_type == cfg.Joints.Type.EMPTY:
                continue
            new_joint_graph[new_x][new_y] = joint_graph[x][y]
            new_joint_graph[new_y][new_x] = joint_graph[x][y]

        merge_count = _old_merge_count

    # endregion

    new_solid_to_eq = [-1] * sum(len(eq) for eq in eqs)
    for index, _ in enumerate(new_eqs):
        for solid in new_eqs[index]:
            new_solid_to_eq[solid] = index

    return new_joint_graph, new_eqs, tuple(new_solid_to_eq)


def can_match(current_isomorphism: list[int], new_test_graph_vertex: int, new_target_vertex: int, target_graph: types.JointGraph, test_graph: gd.Adjacency) -> bool:
    """
    Check if the new vertex can be added to the current_isomorphism by comparing it with all previous vertices
    """
    test_graph_vertex = new_test_graph_vertex
    while test_graph_vertex:
        test_graph_vertex -= 1
        if test_graph[test_graph_vertex][new_test_graph_vertex] != target_graph[new_target_vertex][current_isomorphism[test_graph_vertex]].node_type:
            return False
    return True


def isomorphism_heuristic(target_degree: tuple[int, int], test_degree: tuple[int, int]) -> bool:
    return target_degree[0] >= test_degree[0] and target_degree[1] >= test_degree[1]


def find_isomorphism_test_graph(target_graph: types.JointGraph, target_degrees: types.Degrees, test_graph: gd.Adjacency, test_degree: types.Degrees) -> tuple[bool, types.Isomorphism]:
    """
    Try to find an isomorphism from test_graph to a subgraph of target_graph.
    The potential isomorphism is built vertex by vertex with backtracking (naive approach with degree heuristic)
    """

    if len(test_graph) > len(target_graph):
        return False, ()

    test_graph_vertex = 0

    # current_isomorphism[test_graph_vertex:] is the set of target graph vertices that are not matched yet
    current_isomorphism: list[int] = list(range(len(target_graph)))

    # current_isomorphism[exploration_stack[test_graph_vertex]] is the is next target_graph vertex to try to match with test_graph_vertex
    # exploration_stack[test_graph_vertex] >= test_graph_vertex for all test_graph_vertex
    exploration_stack: list[int] = list(range(len(test_graph)))

    while test_graph_vertex < len(test_graph) and (test_graph_vertex != 0 or exploration_stack[0] < len(target_graph)):
        # vertices remain to match target graph vertex
        if exploration_stack[test_graph_vertex] < len(target_graph):
            shuffle_index = exploration_stack[test_graph_vertex]
            vertex = current_isomorphism[shuffle_index]

            # prepare next vertex
            exploration_stack[test_graph_vertex] += 1

            # vertex is not eligible
            if not isomorphism_heuristic(target_degrees[vertex], test_degree[test_graph_vertex]) or not can_match(current_isomorphism, test_graph_vertex, vertex, target_graph, test_graph):
                continue

            # push vertex
            current_isomorphism[shuffle_index], current_isomorphism[test_graph_vertex] = current_isomorphism[test_graph_vertex], vertex
            test_graph_vertex += 1
        # all vertices were tested on target graph vertex
        else:
            # pop the stack
            exploration_stack[test_graph_vertex] = test_graph_vertex
            test_graph_vertex -= 1

            exchange = exploration_stack[test_graph_vertex] - 1
            # restore shuffle
            current_isomorphism[exchange], current_isomorphism[test_graph_vertex] = current_isomorphism[test_graph_vertex], current_isomorphism[exchange]

    return test_graph_vertex == len(test_graph), tuple(current_isomorphism[:len(test_graph)])


def find_isomorphism(target_graph: types.JointGraph) -> None | tuple[gd.Graphs, types.Isomorphism]:
    """
    Try to find a test_graph that is isomorphic to a subgraph of target_graph with its isomorphism
    """

    target_degrees = compute_degrees(target_graph)
    for g in gd.Graphs:
        is_iso, iso = find_isomorphism_test_graph(target_graph, target_degrees, g.adjacency, g.degrees)
        if not is_iso:
            continue

        return g, iso
    return None


def register_graph_step(config: cfg.Config, graph: gd.Graphs, isomorphism: types.Isomorphism, joint_graph: types.JointGraph, eqs: types.Eq, solid_to_eq: types.EqMapping, strategy_output: list[types.ResolutionStep], joint_states: list[int], joint_queue: list[int], *, before_inputs: bool) -> types.StrategyJointState:

    edge_orientations = []
    for src, dest in graph.edges:
        src_eq, dest_eq = isomorphism[src], isomorphism[dest]

        edge_joint_index = joint_graph[src_eq][dest_eq].joint_index

        s1, s2 = config.joints.solids[edge_joint_index]
        joint_src_eq, joint_dest_eq = solid_to_eq[s1], solid_to_eq[s2]

        edge_orientations.append((edge_joint_index, joint_src_eq == src_eq))

    step = types.GraphStep(graph, tuple(edge_orientations), tuple(eqs[i] for i in isomorphism))
    strategy_output.append(step)

    try:
        register_solved_joints(config, step.get_joints(), joint_states, joint_queue, value_is_computed=False, certain_continuity=before_inputs)
    except ex.SystemConfigurationError as e:
        message, *_ = e.args
        e.args = f"{message} while solving graph {graph} formed by joints:\n- {'\n- '.join(map(repr, step.get_joints()))}",
        raise

    return merge(joint_graph, eqs, isomorphism)


def register_solved_joints(config: cfg.Config, joints: collections.abc.Iterable[int], joint_states: list[int], joint_queue: list[int], *, value_is_computed: bool, certain_continuity: bool) -> None:
    for joint in joints:
        if joint_states[joint] & types.JointFlags.SOLVED_BIT:
            raise ex.SystemConfigurationError(f"Trying to solve <joint:{joint}> that is already solved")
        joint_states[joint] |= types.JointFlags.solved_joint(config.joints.type_[joint], value_is_computed, certain_continuity)
        joint_queue.append(joint)


def test_gear_conformity(config: cfg.Config, relation_node: types.RelationGraphNode, solid_to_eq: types.EqMapping) -> bool:
    rel = relation_node.relation
    _type = config.relations.type_[rel]
    j1, j2 = config.relations.joints[rel]

    if cfg.Relations.Type(_type).is_gear:
        src, target = (j1, j2) if relation_node.is_1_to_2 else (j2, j1)
        # src is solved: src_eq is unique
        src_eq: int = solid_to_eq[config.joints.s1[src]]

        ts1, ts2 = config.joints.solids[target]
        eq1, eq2 = solid_to_eq[ts1], solid_to_eq[ts2]

        relation_node.common_eq = (eq2 == src_eq) if src_eq in (eq1, eq2) else -1
        inference: int = ts1 if eq1 == src_eq else ts2 if eq2 == src_eq else -1
        array = config.relations.g2 if relation_node.is_1_to_2 else config.relations.g1

        if inference != -1:
            infer_pinion_rack(config, rel, inference, array)

        return relation_node.common_eq != -1
    return True


def find_solved_relations_push_gears(config: cfg.Config, relation_graph: types.RelationGraph, solid_to_eq: types.EqMapping, joint_queue: list[int], gear_queue: list[types.RelationGraphNode]) -> collections.abc.Iterable[types.RelationGraphNode]:
    while joint_queue:
        joint = joint_queue.pop(0)

        for relation_node in relation_graph[joint]:
            if relation_node.solved:
                continue
            if test_gear_conformity(config, relation_node, solid_to_eq):
                relation_node.solved = True
                relation_node.pair.solved = True
                yield relation_node
                continue
            gear_queue.append(relation_node)


def find_solved_relations(relation_graph: types.RelationGraph, joint_queue: list[int]) -> collections.abc.Iterable[types.RelationGraphNode]:
    while joint_queue:
        joint: int = joint_queue.pop(0)

        for relation_node in relation_graph[joint]:
            if relation_node.solved:
                continue
            relation_node.solved = True
            relation_node.pair.solved = True
            yield relation_node


def find_solved_relations_with_delayed_gears(config: cfg.Config, relation_graph: types.RelationGraph, solid_to_eq: types.EqMapping, joint_queue: list[int], gear_queue: list[types.RelationGraphNode]):
    yield from find_solved_relations_push_gears(config, relation_graph, solid_to_eq, joint_queue, gear_queue)
    queue_tail = 0
    while queue_tail < len(gear_queue):
        if not test_gear_conformity(config, gear_queue[queue_tail], solid_to_eq):
            queue_tail += 1
            continue

        gear = gear_queue.pop(queue_tail)
        gear.solved = True
        gear.pair.solved = True

        yield gear
        yield from find_solved_relations_push_gears(config, relation_graph, solid_to_eq, joint_queue, gear_queue)
        queue_tail = 0


def register_input_joints(config: cfg.Config, input_joints: np.ndarray, joint_graph: types.JointGraph, eqs: types.Eq, solid_to_eq: types.EqMapping, strategy_output: list[types.ResolutionStep], joint_states: list[int], joint_queue: list[int]) -> types.StrategyJointState:
    for joint in input_joints:
        joint: int
        _type = config.joints.type_[joint]
        s1, s2 = config.joints.solids[joint]
        eq1_index: int = solid_to_eq[s1]
        eq2_index: int = solid_to_eq[s2]
        step: types.JointStep = types.JointStep(cfg.Joints.Type(_type), s1, s2, joint, eqs[eq1_index], eqs[eq2_index])
        strategy_output.append(step)
        joint_graph, eqs, solid_to_eq = merge(joint_graph, eqs, (eq1_index, eq2_index))

    try:
        register_solved_joints(config, (j for j in input_joints), joint_states, joint_queue, value_is_computed=True, certain_continuity=True)
    except ex.SystemConfigurationError as e:
        message, *_ = e.args
        e.args = f"{message} while solving inputs",
        raise

    return joint_graph, eqs, solid_to_eq


def infer_pinion_rack(config: cfg.Config, relation: int, solid: int, array) -> None:
    if array[relation] == -1:
        array[relation] = solid
    elif array[relation] != solid:
        raise ex.SystemConfigurationError("inferred pinion/rack does not match configuration")


def check_gear_formation(config: cfg.Config, solid_to_eq: types.EqMapping) -> list[int]:
    # Pre-requisite: no gear is in the gear queue, i.e. (eq11 == eq12 xor eq21 == eq22) is false for every gear
    result: list[int] = []

    for rel, (_type, joints) in enumerate(zip(config.relations.type_, config.relations.joints)):
        if not cfg.Relations.Type(_type).is_gear:
            continue
        (s11, s12), (s21, s22) = config.joints.solids[joints]
        eq11, eq12, eq21, eq22 = solid_to_eq[s11], solid_to_eq[s12], solid_to_eq[s21], solid_to_eq[s22]

        if eq11 == eq12 == eq21 == eq22:
            # Already solved
            continue

        if eq11 == eq21:
            infer_pinion_rack(config, rel, s12, config.relations.g1)
            infer_pinion_rack(config, rel, s22, config.relations.g2)
        elif eq12 == eq21:
            infer_pinion_rack(config, rel, s11, config.relations.g1)
            infer_pinion_rack(config, rel, s22, config.relations.g2)
        elif eq11 == eq22:
            infer_pinion_rack(config, rel, s12, config.relations.g1)
            infer_pinion_rack(config, rel, s21, config.relations.g2)
        elif eq12 == eq22:
            infer_pinion_rack(config, rel, s11, config.relations.g1)
            infer_pinion_rack(config, rel, s21, config.relations.g2)
        else:
            result.append(rel)
    return result


def simple_gen(value: int) -> collections.abc.Iterable[int]:
    yield value


def register_relation_step(config: cfg.Config, relation_node: types.RelationGraphNode, eqs: types.Eq, solid_to_eq: types.EqMapping, joint_states: list[int], strategy_output: list[types.ResolutionStep], joint_queue: list[int], joint_graph: types.JointGraph) -> types.StrategyJointState:
    _type = config.relations.type_[relation_node.relation]
    j1, j2 = config.relations.joints[relation_node.relation]

    source, target = (j1, j2) if relation_node.is_1_to_2 else (j2, j1)
    relation = relation_node.relation

    stype, ttype = config.joints.type_[[source, target]]
    (s1, s2), (t1, t2) = config.joints.solids[[source, target]]

    if not types.JointFlags.relation_ready(joint_states[source]):
        strategy_output.append(types.JointValueComputationStep(source, stype, joint_states[source] & types.JointFlags.RELATION_READY, s1, s2))
        joint_states[source] |= types.JointFlags.RELATION_READY

    eq1, eq2 = solid_to_eq[t1], solid_to_eq[t2]

    if cfg.Relations.Type(_type).is_gear:
        src_g, dst_g = (config.relations.g1, config.relations.g2) if relation_node.is_1_to_2 else (config.relations.g2, config.relations.g1)
        src_eq = solid_to_eq[s1]
        if src_eq == eq1:
            infer_pinion_rack(config, relation, t2, dst_g)
        elif src_eq == eq2:
            infer_pinion_rack(config, relation, t1, dst_g)
        else:
            raise ex.SystemConfigurationError(f"(internal error) no common eq on solved gear {relation}")
        if src_g[relation] == -1:
            raise ex.SystemConfigurationError(f"Could not infer pinion/rack for {relation}")
    strategy_output.append(types.RelationStep(relation_node.relation, _type, relation_node.is_1_to_2, source, target, ttype, eqs[eq1], eqs[eq2]))
    try:
        register_solved_joints(config, simple_gen(target), joint_states, joint_queue, value_is_computed=True, certain_continuity=True)
    except ex.SystemConfigurationError as e:
        message, *_ = e.args
        e.args = f"{message} while solving relation {relation}",
        raise

    return merge(joint_graph, eqs, (solid_to_eq[t1], solid_to_eq[t2]))


def determine_computation_order(config: cfg.Config, input_joints: np.ndarray[int], strategy_output: list[types.ResolutionStep]) -> None:
    strategy_output.clear()

    joint_states = config.final_joint_states
    joint_states[:] = [0] * config.joints.count
    joint_queue: list[int] = []
    gear_queue: list[types.RelationGraphNode] = []

    joint_graph, eqs, solid_to_eq = make_joint_graph(config)
    relation_graph = make_relation_graph(config)

    # find all graphs before joint resolution
    while len(eqs) > 1:
        at_least_one = False
        while (iso := find_isomorphism(joint_graph)) is not None:
            graph, isomorphism = iso
            joint_graph, eqs, solid_to_eq = register_graph_step(config, graph, isomorphism, joint_graph, eqs, solid_to_eq, strategy_output, joint_states, joint_queue, before_inputs=True)
            at_least_one = True
        if not at_least_one:
            break

        # infer joint relation
        for relation_node in find_solved_relations_with_delayed_gears(config, relation_graph, solid_to_eq, joint_queue, gear_queue):
            joint_graph, eqs, solid_to_eq = register_relation_step(config, relation_node, eqs, solid_to_eq, joint_states, strategy_output, joint_queue, joint_graph)

    if gear_waiting_list := [rel.relation for rel in gear_queue] or check_gear_formation(config, solid_to_eq):
        raise ex.SystemConfigurationError(f"All these gear relations are not formed before input joints:\n-{'\n-'.join(map(repr, gear_waiting_list))}")

    # solve input_joints
    joint_graph, eqs, solid_to_eq = register_input_joints(config, input_joints, joint_graph, eqs, solid_to_eq, strategy_output, joint_states, joint_queue)

    # find the rest
    while len(eqs) > 1:
        # infer joint relation
        for relation_node in find_solved_relations(relation_graph, joint_queue):
            joint_graph, eqs, solid_to_eq = register_relation_step(config, relation_node, eqs, solid_to_eq, joint_states, strategy_output, joint_queue, joint_graph)

        at_least_one = False
        while (iso := find_isomorphism(joint_graph)) is not None:
            graph, isomorphism = iso
            joint_graph, eqs, solid_to_eq = register_graph_step(config, graph, isomorphism, joint_graph, eqs, solid_to_eq, strategy_output, joint_states, joint_queue, before_inputs=False)
            at_least_one = True
        if not at_least_one:
            break

    if len(eqs) > 1:
        raise ex.SystemConfigurationError("Could not solve entire system")
