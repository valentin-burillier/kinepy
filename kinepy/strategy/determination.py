from kinepy.objects.new_joints import Joint, Revolute, Prismatic
import enum
from kinepy.strategy.graphs import NodeType, ADJACENCY, Adjacency, DEGREES


class SystemConfigurationError(Exception):
    pass


class StrategyItem(enum.Enum):
    GRAPH, JOINT_INPUTS, RELATION = range(3)


class GraphNode:

    node_type: NodeType
    joint_index: int = -1

    def __init__(self, joint: Revolute | Prismatic | None):
        self.set_node_type(joint)

    def set_node_type(self, joint: Revolute | Prismatic | None):
        if joint is not None:
            self.joint_index = joint.index
        else:
            self.joint_index = -1
        self.node_type = NodeType.EMPTY if joint is None else NodeType.REVOLUTE if isinstance(joint, Revolute) else NodeType.PRISMATIC


JointGraph = list[list[GraphNode]]
Degrees = tuple[tuple[int, int], ...]
Eq = tuple[tuple[int], ...]
PrimitiveJointList = list[Revolute | Prismatic]
JointList = list[Joint]


def make_joint_graph(solid_count: int, joints: PrimitiveJointList) -> tuple[JointGraph, Eq, Degrees]:
    result_graph: JointGraph = [[GraphNode(None) for _ in range(solid_count)] for _ in range(solid_count)]

    for joint in joints:
        result_graph[joint.s1][joint.s2].set_node_type(joint)
        result_graph[joint.s2][joint.s1].set_node_type(joint)

    return result_graph, tuple((i,) for i in range(solid_count)), compute_degrees(result_graph)


def compute_degrees(joint_graph: JointGraph) -> Degrees:
    degrees: list[tuple[int, int]] = []
    for adj in joint_graph:
        results = [0, 0, 0]
        for node in adj:
            results[node.node_type.value] += 1
        degrees.append((results[1], results[2]))

    return tuple(degrees)


def merge(joint_graph: JointGraph, eqs: Eq, group_to_merge: tuple[int]) -> tuple[JointGraph, Eq, Degrees]:
    merge_state = [False] * len(eqs)
    for index in group_to_merge:
        merge_state[index] = True
    target_eq = min(group_to_merge)

    new_eqs = []
    for index, eq in enumerate(eqs):
        if index <= target_eq or not merge_state[index]:
            new_eqs.append(eq)
        else:
            new_eqs[target_eq] += eq
    new_eqs = tuple(new_eqs)

    new_joint_graph = [[GraphNode(None) for _ in new_eqs] for _ in new_eqs]

    merge_count = 0
    for x in range(len(eqs)):
        merge_count += merge_state[x] - (x == target_eq)
        new_x = target_eq if merge_state[x] else x - merge_count
        _old_merge_count = merge_count
        for y in range(x+1, len(eqs)):
            merge_count += merge_state[y] - (y == target_eq)
            new_y = target_eq if merge_state[y] else y - merge_count
            if new_x == new_y or joint_graph[x][y].node_type == NodeType.EMPTY:
                continue
            new_joint_graph[new_x][new_y] = joint_graph[x][y]
            new_joint_graph[new_y][new_x] = joint_graph[x][y]

        merge_count = _old_merge_count

    return new_joint_graph, new_eqs, compute_degrees(new_joint_graph)


def can_match(current_isomorphism, new_test_graph_vertex, new_target_vertex, target_graph: JointGraph, test_graph: Adjacency) -> bool:
    """
    Check if the new vertex can be added to the current_isomorphism by comparing it with all previous vertices
    """
    test_graph_vertex = new_test_graph_vertex
    while test_graph_vertex:
        test_graph_vertex -= 1
        if test_graph[test_graph_vertex][new_test_graph_vertex] != target_graph[new_target_vertex][current_isomorphism[test_graph_vertex]].node_type.value:
            return False
    return True


def isomorphism_heuristic(target_degree: tuple[int, int], test_degree: tuple[int, int]) -> bool:
    return target_degree[0] >= test_degree[0] and target_degree[1] >= test_degree[1]


def find_isomorphism_test_graph(target_graph: JointGraph, target_degrees, test_graph: Adjacency, test_degree) -> tuple[bool, tuple[int, ...]]:
    """
    Try to find an isomorphism from test_graph to a subgraph of target_graph
    """

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


def find_isomorphism(target_graph: JointGraph, target_degrees: Degrees) -> None | tuple[int, tuple[int, ...]]:
    """
    Try to find a test_graph that is isomorphic to a subgraph of target_graph with its isomorphism
    """
    for index, (adj, deg) in enumerate(zip(ADJACENCY, DEGREES)):
        is_iso, iso = find_isomorphism_test_graph(target_graph, target_degrees, adj, deg)
        if not is_iso:
            continue

        return index, iso
    return None


def determine_computation_order(solid_count: int, joints: PrimitiveJointList, input_joints: JointList, strategy_output: list) -> None:
    strategy_output.clear()
    joint_graph, eqs, joint_degree = make_joint_graph(solid_count, joints)

    # find all graphs before joint resolution
    while len(eqs) > 1 and (iso := find_isomorphism(joint_graph, joint_degree)) is not None:
        graph_index, isomorphism = iso

        # TODO: register graph step

        joint_graph, eqs, joint_degree = merge(joint_graph, eqs, isomorphism)

        # TODO: infer joint relation resolution

    # solve input_joints
    # TODO: Check gear formation
    # TODO: register joint steps
    # TODO: infer joint relation resolution

    # find the rest
    while len(eqs) > 1 and (iso := find_isomorphism(joint_graph, joint_degree)) is not None:
        graph_index, isomorphism = iso

        # TODO: register graph step

        joint_graph, eqs, joint_degree = merge(joint_graph, eqs, isomorphism)

        # TODO: infer joint relation resolution
