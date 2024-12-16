import enum
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
PrimitiveJointList = list[Revolute | Prismatic]
JointList = list[Joint]


def make_joint_graph(solid_count: int, joints: PrimitiveJointList) -> JointGraph:
    result_graph: JointGraph = [[GraphNode(None) for _ in range(solid_count)] for _ in range(solid_count)]

    for joint in joints:
        result_graph[joint.s1][joint.s2].set_node_type(joint)
        result_graph[joint.s2][joint.s1].set_node_type(joint)

    return result_graph


def determine_computation_order(solid_count: int, joints: PrimitiveJointList, input_joints: JointList, strategy_output: list):
    joint_graph: JointGraph = make_joint_graph(solid_count, joints)
