from kinepy.objects.new_joints import Joint, Revolute, Prismatic
from kinepy.strategy.graphs import *


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


def make_joint_graph(solid_count: int, joints: PrimitiveJointList) -> tuple[JointGraph, Degrees]:
    result_graph: JointGraph = [[GraphNode(None) for _ in range(solid_count)] for _ in range(solid_count)]

    for joint in joints:
        result_graph[joint.s1][joint.s2].set_node_type(joint)
        result_graph[joint.s2][joint.s1].set_node_type(joint)

    return result_graph, compute_degrees(result_graph)


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


def can_match(shuffle, stage, new_vertex, joint_graph: JointGraph, test_graph: Adjacency) -> bool:
    index = stage
    while index:
        index -= 1
        if test_graph[index][stage] != joint_graph[new_vertex][shuffle[index]].node_type.value:
            return False
    return True


def isomorphism_heuristic(src, target) -> bool:
    return src[0] >= target[0] and src[1] >= target[1]


def find_isomorphism_test_graph(joint_graph: JointGraph, joint_degree, test_graph: Adjacency, test_degree) -> tuple[bool, tuple[int, ...]]:
    stage = 0
    stack: list[int] = list(range(len(test_graph)))
    shuffle: list[int] = list(range(len(joint_graph)))
    while stage < len(test_graph) and (stage != 0 or stack[0] < len(joint_graph)):
        # vertices remain to match target graph vertex
        if stack[stage] < len(joint_graph):
            shuffle_index = stack[stage]
            vertex = shuffle[shuffle_index]

            # prepare next vertex
            stack[stage] += 1

            # vertex is not eligible
            if not isomorphism_heuristic(joint_degree[vertex], test_degree[stage]) or not can_match(shuffle, stage, vertex, joint_graph, test_graph):
                continue

            # push vertex
            shuffle[shuffle_index], shuffle[stage] = shuffle[stage], vertex
            stage += 1
        # all vertices were tested on target graph vertex
        else:
            # pop the stack
            stack[stage] = stage
            stage -= 1

            exchange = stack[stage] - 1
            # restore shuffle
            shuffle[exchange], shuffle[stage] = shuffle[stage], shuffle[exchange]

    return stage == len(test_graph), tuple(shuffle[:len(test_graph)])


def find_isomorphism(joint_graph: JointGraph, degrees: Degrees) -> None | tuple[int, ...]:
    for adj, deg in zip(ADJACENCY, DEGREES):
        is_iso, iso = find_isomorphism_test_graph(joint_graph, degrees, adj, deg)
        if not is_iso:
            continue

        return iso
    return None


def determine_computation_order(solid_count: int, joints: PrimitiveJointList, input_joints: JointList, strategy_output: list):
    joint_graph, joint_degree = make_joint_graph(solid_count, joints)

    # find all graphs before joint resolution

    # solve input_joints

    # find the rest
