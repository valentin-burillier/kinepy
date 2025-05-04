import enum


class RelationType(enum.Enum):
    GEAR, GEAR_RACK, BELT, DISTANT, EFFORTLESS = range(5)


GEAR_TYPES = RelationType.GEAR_RACK, RelationType.GEAR, RelationType.BELT


class JointType(enum.Enum):
    EMPTY, REVOLUTE, PRISMATIC = range(3)
    X, Y, J_AXLE = 6, 10, 14
    GHOST_ANGLE = 5

    def simple(self):
        """
        Removes extra information giving only the primitive type
        """
        return JointType(self.value & 3)


E, R, P = JointType.EMPTY, JointType.REVOLUTE, JointType.PRISMATIC

Adjacency = tuple[tuple[JointType, ...], ...]


graph_rrr: Adjacency = (
    (E, R, R),
    (R, E, R),
    (R, R, E)
)
r"""
        0
       / \
      R   R
     /     \
    1 - R - 2  
"""

graph_rrp: Adjacency = (
    (E, R, R),
    (R, E, P),
    (R, P, E)
)
r"""
        0
       / \
      R   R
     /     \
    1 - P - 2  
"""

graph_ppr: Adjacency = (
    (E, P, P),
    (P, E, R),
    (P, R, E)
)
r"""
        0
       / \
      P   P 
     /     \
    1 - R - 2  
"""


graph_3rr: Adjacency = (
    (E, R, R, R, E),
    (R, E, E, E, R),
    (R, E, E, E, R),
    (R, E, E, E, R),
    (E, R, R, R, E)
)
r"""
         0
        /|\
       R R R
      /  |  \
     1   2   3
      \  |  /
       R R R
        \|/
         4
"""


graph_2rr_pp: Adjacency = (
    (E, R, R, P, E),
    (R, E, E, E, R),
    (R, E, E, E, R),
    (P, E, E, E, P),
    (E, R, R, P, E)
)
r"""
         0
        /|\
       R R P
      /  |  \
     1   2   3
      \  |  /
       R R P
        \|/
         4
"""


graph_3pr: Adjacency = (
    (E, P, P, P, E),
    (P, E, E, E, R),
    (P, E, E, E, R),
    (P, E, E, E, R),
    (E, R, R, R, E)
)
r"""
         0
        /|\
       P P P
      /  |  \
     1   2   3
      \  |  /
       R R R
        \|/
         4
"""


graph_2pr_rr: Adjacency = (
    (E, P, P, R, E),
    (P, E, E, E, R),
    (P, E, E, E, R),
    (R, E, E, E, R),
    (E, R, R, R, E)
)
r"""
         0
        /|\
       P P R
      /  |  \
     1   2   3
      \  |  /
       R R R
        \|/
         4
"""


graph_pp_rr_pr: Adjacency = (
    (E, P, R, P, E),
    (P, E, E, E, P),
    (R, E, E, E, R),
    (P, E, E, E, R),
    (E, P, R, R, E)
)
r"""
         0
        /|\
       P R P
      /  |  \
     1   2   3
      \  |  /
       P R R
        \|/
         4
"""


graph_2rr_pr: Adjacency = (
    (E, R, R, P, E),
    (R, E, E, E, R),
    (R, E, E, E, R),
    (P, E, E, E, R),
    (E, R, R, R, E)
)
r"""
         0
        /|\
       R R R
      /  |  \
     1   2   3
      \  |  /
       R R R
        \|/
         4
"""


graph_pp_pr_rp: Adjacency = (
    (E, P, P, R, E),
    (P, E, E, E, P),
    (P, E, E, E, R),
    (R, E, E, E, P),
    (E, P, R, P, E)
)
r"""
         0
        /|\
       P P R
      /  |  \
     1   2   3
      \  |  /
       P R P
        \|/
         4
"""


graph_2rp_pp: Adjacency = (
    (E, R, R, P, E),
    (R, E, E, E, P),
    (R, E, E, E, P),
    (P, E, E, E, P),
    (E, P, P, P, E)
)
r"""
         0
        /|\
       R R P
      /  |  \
     1   2   3
      \  |  /
       P P P
        \|/
         4
"""


graph_rr_pr_rp: Adjacency = (
    (E, R, P, R, E),
    (R, E, E, E, R),
    (P, E, E, E, R),
    (R, E, E, E, P),
    (E, R, R, P, E)
)
r"""
         0
        /|\
       R P R
      /  |  \
     1   2   3
      \  |  /
       R R P
        \|/
         4
"""


graph_2rp_pr: Adjacency = (
    (E, R, R, P, E),
    (R, E, E, E, P),
    (R, E, E, E, P),
    (P, E, E, E, R),
    (E, P, P, R, E)
)
r"""
         0
        /|\
       R R P
      /  |  \
     1   2   3
      \  |  /
       P P R
        \|/
         4
"""


dyad_edges = (0, 1), (0, 2), (1, 2)
triad_edges = (0, 1), (0, 2), (0, 3), (1, 4), (2, 4), (3, 4)


ADJACENCY: tuple[Adjacency, ...] = (
    graph_rrr, graph_rrp, graph_ppr, graph_3rr, graph_2rr_pp, graph_3pr, graph_2pr_rr, graph_pp_rr_pr, graph_2rr_pr, graph_pp_pr_rp, graph_2rp_pp, graph_rr_pr_rp, graph_2rp_pr
)

SOLUTION_COUNT = (
    2, 2, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
)
EDGES = (dyad_edges,) * 3 + (triad_edges,) * 10


def _degrees(graph: Adjacency):
    result: list[tuple[int, int]] = [(0, 0)] * len(graph)
    for index, adj_list in enumerate(graph):
        sums = [0, 0, 0]
        for joint_id in adj_list:
            sums[joint_id.value] += 1
        result[index] = sums[R.value], sums[P.value]
    return tuple(result)


DEGREES = tuple(_degrees(adj) for adj in ADJACENCY)


class Graphs(enum.Enum):
    """
    Name are prefixed with 'g' since some names start with a number
    """
    gRRR, gRRP, gPPR, g3RR, g2RR_PP, g3PR, g2PR_RR, gPP_RR_PR, g2RR_PR, gPP_PR_RP, g2RP_PP, gRR_PR_RP, g2RP_PR = range(13)

    def __str__(self):
        return f'<{self.__class__.__name__}.{self.name[1:]}: {self.value}>'

    __repr__ = __str__

    @property
    def edges(self):
        return EDGES[self.value]

    @property
    def adjacency(self) -> Adjacency:
        return ADJACENCY[self.value]

    @property
    def solutions(self) -> int:
        return SOLUTION_COUNT[self.value]

    @property
    def degrees(self):
        return DEGREES[self.value]
