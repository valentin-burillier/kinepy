from kinepy.objects.joints import Joint
from kinepy.objects.relations import GearRelation
from kinepy.strategy.types import *
from kinepy.objects.solid import Solid
import kinepy.strategy.graph_data as graph_data
import kinepy.exceptions as ex


def make_joint_graph(solid_count: int, joints: list[PrimitiveJoint]) -> tuple[JointGraph, Eq, EqMapping]:
    """
    Create adjacency matrix for joint graph, initial eqs, initial eq mapping (solid_index -> eq_index), compute initial value of degrees
    """
    result_graph: JointGraph = [[JointGraphNode(NodeType.EMPTY) for _ in range(solid_count)] for _ in range(solid_count)]

    for joint in joints:
        result_graph[joint.s1._index][joint.s2._index].set_node_type(joint)
        result_graph[joint.s2._index][joint.s1._index].set_node_type(joint)

    return result_graph, tuple((i,) for i in range(solid_count)), tuple(range(solid_count))


def make_relation_graph(joint_count: int, relations: list[Relation]) -> RelationGraph:
    """
    Create adjacency lists for the relation graph
    """
    result_graph: RelationGraph = [[] for _ in range(joint_count)]

    for relation in relations:
        result_graph[relation.j1._index].append(r1 := RelationGraphNode(True, relation))
        result_graph[relation.j2._index].append(r2 := RelationGraphNode(False, relation))

        r1.pair = r2
        r2.pair = r1

    return result_graph


def compute_degrees(joint_graph: JointGraph) -> Degrees:
    degrees: list[tuple[int, int]] = []
    for adj in joint_graph:
        results = [0, 0, 0]
        for node in adj:
            results[node.node_type.value] += 1
        degrees.append((results[1], results[2]))

    return tuple(degrees)


def merge(joint_graph: JointGraph, eqs: Eq, group_to_merge: tuple[int, ...]) -> tuple[JointGraph, Eq, EqMapping]:
    merge_state = [False] * len(eqs)
    for index in group_to_merge:
        merge_state[index] = True
    target_eq = min(group_to_merge)

    new_eqs: list[tuple[int, ...], ...] = []
    for index, eq in enumerate(eqs):
        if index <= target_eq or not merge_state[index]:
            new_eqs.append(eq)
        else:
            new_eqs[target_eq] += eq
    new_eqs: Eq = tuple(new_eqs)

    # region joint_graph merge

    new_joint_graph = [[JointGraphNode(NodeType.EMPTY) for _ in new_eqs] for _ in new_eqs]

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
            if new_x == new_y or joint_graph[x][y].node_type == graph_data.NodeType.EMPTY:
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


def can_match(current_isomorphism, new_test_graph_vertex, new_target_vertex, target_graph: JointGraph, test_graph: graph_data.Adjacency) -> bool:
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


def find_isomorphism_test_graph(target_graph: JointGraph, target_degrees: Degrees, test_graph: graph_data.Adjacency, test_degree: Degrees) -> tuple[bool, Isomorphism]:
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


def find_isomorphism(target_graph: JointGraph) -> None | tuple[int, Isomorphism]:
    """
    Try to find a test_graph that is isomorphic to a subgraph of target_graph with its isomorphism
    """

    target_degrees = compute_degrees(target_graph)
    for index, (adj, deg) in enumerate(zip(graph_data.ADJACENCY, graph_data.DEGREES)):
        is_iso, iso = find_isomorphism_test_graph(target_graph, target_degrees, adj, deg)
        if not is_iso:
            continue

        return index, iso
    return None


def register_graph_step(graph_index: int, isomorphism: Isomorphism, joint_graph: JointGraph, eqs: Eq, solid_to_eq: EqMapping, joints: list[PrimitiveJoint], strategy_output: list[ResolutionStep], joint_states: list[int], joint_queue: list[PrimitiveJoint], *, before_inputs: bool) -> tuple[JointGraph, Eq, EqMapping]:

    edge_orientations = []
    for src, dest in graph_data.EDGES[graph_index]:
        src_eq, dest_eq = isomorphism[src], isomorphism[dest]

        edge_joint_index = joint_graph[src_eq][dest_eq].joint_index
        edge_joint: PrimitiveJoint = joints[edge_joint_index]

        joint_src_eq, joint_dest_eq = solid_to_eq[edge_joint.s1._index], solid_to_eq[edge_joint.s2._index]

        edge_orientations.append((edge_joint, joint_src_eq == src_eq))

    step = GraphStep(graph_index, tuple(edge_orientations), tuple(eqs[i] for i in isomorphism))
    strategy_output.append(step)

    try:
        register_solved_joints(step.get_joints(), joint_states, joint_queue, value_is_computed=False, certain_continuity=before_inputs)
    except ex.SystemConfigurationError as e:
        message, *_ = e.args
        e.args = f"{message} while solving graph {graph_data.NAMES[graph_index]} formed by joints:\n- {'\n- '.join(map(repr, step.get_joints()))}",
        raise

    return merge(joint_graph, eqs, isomorphism)


def register_solved_joints(joints: Generator[PrimitiveJoint, None, None], joint_states: list[int], joint_queue: list[PrimitiveJoint], *, value_is_computed: bool, certain_continuity: bool) -> None:
    for joint in joints:
        joint_index = joint._index
        if joint_states[joint_index] & JointFlags.SOLVED_BIT:
            raise ex.SystemConfigurationError(f"Trying to solve {joint} that is already solved")
        joint_states[joint_index] |= JointFlags.SOLVED_BIT | (certain_continuity or isinstance(joint, Prismatic)) * JointFlags.CONTINUOUS_BIT | value_is_computed * JointFlags.COMPUTED_BIT
        joint_queue.append(joint)


def test_gear_conformity(relation_node: RelationGraphNode, solid_to_eq: EqMapping) -> bool:
    rel = relation_node.relation
    if isinstance(rel, GearRelation):
        src: PrimitiveJoint = relation_node.get_source_joint()
        src_eq: int = solid_to_eq[src.s1._index]

        target: PrimitiveJoint = relation_node.get_target_joint()
        eq1, eq2 = solid_to_eq[target.s1._index], solid_to_eq[target.s2._index]

        attr: str = ('_g1', '_g2')[relation_node.is_1_to_2]

        if eq1 == src_eq:
            relation_node.common_eq = 0
            if getattr(rel, attr) is None:
                setattr(rel, attr, target.s1)
            elif getattr(rel, attr) is not target.s1:
                raise ex.SystemConfigurationError("inferred pinion/rack does not match configuration")
        elif eq2 == src_eq:
            relation_node.common_eq = 1
            if getattr(rel, attr) is None:
                setattr(rel, attr, target.s2)
            elif getattr(rel, attr) is not target.s2:
                raise ex.SystemConfigurationError("inferred pinion/rack does not match configuration")

        return relation_node.common_eq != -1
    return True


def find_solved_relations_push_gears(relation_graph: RelationGraph, solid_to_eq: EqMapping, joint_queue: list[PrimitiveJoint], gear_queue: list[RelationGraphNode]) -> Generator[RelationGraphNode, None, None]:
    while joint_queue:
        joint = joint_queue.pop(0)

        for relation_node in relation_graph[joint._index]:
            if relation_node.solved:
                continue
            if test_gear_conformity(relation_node, solid_to_eq):
                relation_node.solved = True
                relation_node.pair.solved = True
                yield relation_node
                continue
            gear_queue.append(relation_node)


def find_solved_relations(relation_graph: RelationGraph, joint_queue: list[PrimitiveJoint]) -> Generator[RelationGraphNode, None, None]:
    while joint_queue:
        joint = joint_queue.pop(0)

        for relation_node in relation_graph[joint._index]:
            if relation_node.solved:
                continue
            yield relation_node


def find_solved_relations_with_delayed_gears(relation_graph: RelationGraph, solid_to_eq: EqMapping, joint_queue: list[PrimitiveJoint], gear_queue: list[RelationGraphNode]):
    yield from find_solved_relations_push_gears(relation_graph, solid_to_eq, joint_queue, gear_queue)
    queue_tail = 0
    while queue_tail < len(gear_queue):
        if not test_gear_conformity(gear_queue[queue_tail], solid_to_eq):
            queue_tail += 1
            continue

        gear = gear_queue.pop(queue_tail)
        gear.solved = True
        gear.pair.solved = True

        yield gear
        yield from find_solved_relations_push_gears(relation_graph, solid_to_eq, joint_queue, gear_queue)
        queue_tail = 0


def register_input_joints(input_joints: list[Joint], joint_graph: JointGraph, eqs: Eq, solid_to_eq: EqMapping, strategy_output: list[ResolutionStep], joint_states: list[int], joint_queue: list[PrimitiveJoint]) -> tuple[JointGraph, Eq, EqMapping]:
    for input_index, joint in enumerate(j for joint in input_joints for j in joint.get_all_joints()):
        eq1_index: int = solid_to_eq[joint.s1._index]
        eq2_index: int = solid_to_eq[joint.s2._index]
        step: JointStep = JointStep(input_index, joint, eqs[eq1_index], eqs[eq2_index])
        strategy_output.append(step)
        joint_graph, eqs, solid_to_eq = merge(joint_graph, eqs, (eq1_index, eq2_index))

    try:
        register_solved_joints((j for joint in input_joints for j in joint.get_all_joints()), joint_states, joint_queue, value_is_computed=True, certain_continuity=True)
    except ex.SystemConfigurationError as e:
        message, *_ = e.args
        e.args = f"{message} while solving inputs",
        raise

    return joint_graph, eqs, solid_to_eq


def infer_pinion_rack(relation: GearRelation, solid: Solid, attr: str) -> None:
    if getattr(relation, attr) is None:
        setattr(relation, attr, solid)
    elif getattr(relation, attr) is not solid:
        raise ex.SystemConfigurationError("inferred pinion/rack does not match configuration")


def check_gear_formation(relations: list[Relation], solid_to_eq: EqMapping) -> list[GearRelation]:
    # Pre-requisite: no gear is in the gear queue, i.e. (eq11 == eq12 xor eq21 == eq22) is false for every relation
    result: list[GearRelation] = []

    for relation in relations:
        if not isinstance(relation, GearRelation):
            continue
        relation: GearRelation
        eq11, eq12, eq21, eq22 = solid_to_eq[relation.j1.s1._index], solid_to_eq[relation.j1.s2._index], solid_to_eq[relation.j2.s1._index], solid_to_eq[relation.j2.s2._index]

        if eq11 == eq12 == eq21 == eq22:
            # Already solved
            continue

        if eq11 == eq21:
            infer_pinion_rack(relation, relation.j1.s2, '_g1')
            infer_pinion_rack(relation, relation.j2.s2, '_g2')
        elif eq12 == eq21:
            infer_pinion_rack(relation, relation.j1.s1, '_g1')
            infer_pinion_rack(relation, relation.j2.s2, '_g2')
        elif eq11 == eq22:
            infer_pinion_rack(relation, relation.j1.s2, '_g1')
            infer_pinion_rack(relation, relation.j2.s1, '_g2')
        elif eq12 == eq22:
            infer_pinion_rack(relation, relation.j1.s1, '_g1')
            infer_pinion_rack(relation, relation.j2.s1, '_g2')
        else:
            result.append(relation)
    return result


def register_relation_step(relation_node: RelationGraphNode, eqs: Eq, solid_to_eq: EqMapping, joint_states: list[int], strategy_output: list[ResolutionStep], joint_queue: list[PrimitiveJoint], joint_graph: JointGraph) -> tuple[JointGraph, Eq, EqMapping]:
    source = relation_node.get_source_joint()
    target = relation_node.get_target_joint()

    if not joint_states[source._index] & JointFlags.COMPUTED_BIT or not joint_states[source._index] & JointFlags.CONTINUOUS_BIT:
        strategy_output.append(JointValueComputationStep(source, joint_states[source._index] & (JointFlags.CONTINUOUS_BIT | JointFlags.COMPUTED_BIT)))
        joint_states[source._index] |= JointFlags.COMPUTED_BIT | JointFlags.CONTINUOUS_BIT

    relation = relation_node.relation
    eq1, eq2 = eqs[solid_to_eq[target.s1._index]], eqs[solid_to_eq[target.s2._index]]
    src_g, dst_g = ('_g1', '_g2')[::2 * relation_node.is_1_to_2 - 1]
    if isinstance(relation, GearRelation):
        src_eq = solid_to_eq[source.s1._index]
        if src_eq == eq1:
            infer_pinion_rack(relation, target.s2, dst_g)
        elif src_eq == eq2:
            infer_pinion_rack(relation, target.s1, dst_g)
        else:
            raise ex.SystemConfigurationError(f"(internal error) no common eq on solved gear {relation}")
        if getattr(relation, src_g) is None:
            raise ex.SystemConfigurationError(f"Could not infer pinion/rack for {relation}")
    strategy_output.append(RelationStep(relation_node.relation, relation_node.is_1_to_2, eq1, eq2))

    try:
        register_solved_joints(relation_node.yield_target_joint(), joint_states, joint_queue, value_is_computed=True, certain_continuity=True)
    except ex.SystemConfigurationError as e:
        message, *_ = e.args
        e.args = f"{message} while solving relation {relation}",
        raise

    return merge(joint_graph, eqs, (solid_to_eq[relation_node.get_target_joint().s1._index], solid_to_eq[relation_node.get_target_joint().s2._index]))


def determine_computation_order(solid_count: int, joints: list[PrimitiveJoint], relations: list[Relation], joint_states: list[int], input_joints: list[Joint], strategy_output: list[ResolutionStep]) -> None:
    strategy_output.clear()

    joint_states[:] = [0] * len(joints)
    joint_queue: list[PrimitiveJoint] = []
    gear_queue: list[RelationGraphNode] = []

    joint_graph, eqs, solid_to_eq = make_joint_graph(solid_count, joints)
    relation_graph = make_relation_graph(len(joints), relations)

    # find all graphs before joint resolution
    while len(eqs) > 1:
        at_least_one = False
        while (iso := find_isomorphism(joint_graph)) is not None:
            graph_index, isomorphism = iso
            joint_graph, eqs, solid_to_eq = register_graph_step(graph_index, isomorphism, joint_graph, eqs, solid_to_eq, joints, strategy_output, joint_states, joint_queue, before_inputs=True)
            at_least_one = True
        if not at_least_one:
            break

        # infer joint relation
        for relation_node in find_solved_relations_with_delayed_gears(relation_graph, solid_to_eq, joint_queue, gear_queue):
            joint_graph, eqs, solid_to_eq = register_relation_step(relation_node, eqs, solid_to_eq, joint_states, strategy_output, joint_queue, joint_graph)

    if gear_waiting_list := [rel.relation for rel in gear_queue] or check_gear_formation(relations, solid_to_eq):
        raise ex.SystemConfigurationError(f"All these gear relations are not formed before input joints:\n-{'\n-'.join(map(repr, gear_waiting_list))}")

    # solve input_joints
    joint_graph, eqs, solid_to_eq = register_input_joints(input_joints, joint_graph, eqs, solid_to_eq, strategy_output, joint_states, joint_queue)

    # find the rest
    while len(eqs) > 1:
        # infer joint relation
        for relation_node in find_solved_relations(relation_graph, joint_queue):
            joint_graph, eqs, solid_to_eq = register_relation_step(relation_node, eqs, solid_to_eq, joint_states, strategy_output, joint_queue, joint_graph)

        at_least_one = False
        while (iso := find_isomorphism(joint_graph)) is not None:
            graph_index, isomorphism = iso
            joint_graph, eqs, solid_to_eq = register_graph_step(graph_index, isomorphism, joint_graph, eqs, solid_to_eq, joints, strategy_output, joint_states, joint_queue, before_inputs=False)
            at_least_one = True
        if not at_least_one:
            break

    if len(eqs) > 1:
        raise ex.SystemConfigurationError("Could not solve entire system")
