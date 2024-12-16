import enum


class NodeType(enum.Enum):
    EMPTY, REVOLUTE, PRISMATIC = range(3)


E, R, P = NodeType.EMPTY.value, NodeType.REVOLUTE.value, NodeType.PRISMATIC.value

Adjacency = tuple[tuple[int, ...], ...]


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

NAMES = (
    'RRR', 'RRP', 'PPR', '3RR', '2RR-PP', '3PR', '2PR-RR', 'PP-RR-PR', '2RR-PR', 'PP-PR-RP', '2RP-PP', 'RR-PR-RP', '2RP-PR'
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
            sums[joint_id] += 1
        result[index] = sums[R], sums[P]
    return tuple(result)


DEGREES = tuple(_degrees(adj) for adj in ADJACENCY)
print(graph_rrr.__doc__)