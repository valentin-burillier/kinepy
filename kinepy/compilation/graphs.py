edges0 = (0, 1), (0, 2), (1, 2)
edges1 = (0, 1), (0, 2), (0, 3), (1, 4), (2, 4), (3, 4)


graph0 = [
    [0, 1, 1],  # -      O
    [1, 0, 1],  # -    R/ \R
    [1, 1, 0]   # -    1 _ 2
]  # -                   R

graph1 = [
    [0, 1, 1],  # -      0
    [1, 0, 2],  # -    R/ \R
    [1, 2, 0]   # -    1 _ 2
]  # -                   P

graph2 = [
    [0, 2, 2],  # -      0
    [2, 0, 1],  # -    P/ \P
    [2, 1, 0]   # -    1 _ 2
]  # -                   R

graph3 = [
    [0, 1, 1, 1, 0],  # -         O
    [1, 0, 0, 0, 1],  # -     R/ R|  \R
    [1, 0, 0, 0, 1],  # -     1   2   3
    [1, 0, 0, 0, 1],  # -     R\ R|  /R
    [0, 1, 1, 1, 0]   # -         4
]

graph4 = [
    [0, 1, 1, 2, 0],  # -         O
    [1, 0, 0, 0, 1],  # -     R/ R|  \P
    [1, 0, 0, 0, 1],  # -     1   2   3
    [2, 0, 0, 0, 2],  # -     R\ R|  /P
    [0, 1, 1, 2, 0]   # -         4
]

graph5 = [
    [0, 1, 1, 1, 0],  # -         O
    [1, 0, 0, 0, 2],  # -     R/ R|  \R
    [1, 0, 0, 0, 2],  # -     1   2   3
    [1, 0, 0, 0, 2],  # -     P\ P|  /P
    [0, 2, 2, 2, 0]   # -         4
]

graph6 = [
    [0, 1, 1, 1, 0],  # -         O
    [1, 0, 0, 0, 2],  # -     R/ R|  \R
    [1, 0, 0, 0, 2],  # -     1   2   3
    [1, 0, 0, 0, 1],  # -     P\ P|  /R
    [0, 2, 2, 1, 0]   # -         4
]

graph7 = [
    [0, 2, 1, 2, 0],  # -         O
    [2, 0, 0, 0, 2],  # -     P/ R|  \P
    [1, 0, 0, 0, 1],  # -     1   2   3
    [2, 0, 0, 0, 1],  # -     P\ R|  /R
    [0, 2, 1, 1, 0]   # -         4
]

graph8 = [
    [0, 1, 1, 1, 0],  # -         O
    [1, 0, 0, 0, 1],  # -     R/ R|  \R
    [1, 0, 0, 0, 1],  # -     1   2   3
    [1, 0, 0, 0, 2],  # -     R\ R|  /P
    [0, 1, 1, 2, 0]   # -         4
]

graph9 = [
    [0, 2, 2, 1, 0],  # -         O
    [2, 0, 0, 0, 2],  # -     P/ P|  \R
    [2, 0, 0, 0, 1],  # -     1   2   3
    [1, 0, 0, 0, 2],  # -     P\ R|  /P
    [0, 2, 1, 2, 0]   # -         4
]

graph10 = [
    [0, 2, 1, 1, 0],  # -         O
    [2, 0, 0, 0, 2],  # -     P/ R|  \R
    [1, 0, 0, 0, 2],  # -     1   2   3
    [1, 0, 0, 0, 2],  # -     P\ P|  /P
    [0, 2, 2, 2, 0]   # -         4
]

graph11 = [
    [0, 1, 1, 2, 0],  # -         O
    [1, 0, 0, 0, 1],  # -     R/ R|  \P
    [1, 0, 0, 0, 2],  # -     1   2   3
    [2, 0, 0, 0, 1],  # -     R\ P|  /R
    [0, 1, 2, 1, 0]   # -         4
]

graph12 = [
    [0, 1, 2, 1, 0],  # -         O
    [1, 0, 0, 0, 2],  # -     R/ P|  \R
    [2, 0, 0, 0, 1],  # -     1   2   3
    [1, 0, 0, 0, 2],  # -     P\ R|  /P
    [0, 2, 1, 2, 0]   # -         4
]


GRAPHS = [
    graph0, graph1, graph2, graph3, graph4, graph5, graph6, graph7, graph8, graph9, graph10, graph11, graph12
]

EDGES = [
    edges0, edges0, edges0, edges1, edges1, edges1, edges1, edges1, edges1, edges1, edges1, edges1, edges1
]

SOLVED = [
    True, True, True, False, True, True, False, True, False, True, True, False, False
]

SIGNS = [
    1, 1, None, None, 1, 1, None, 1, None, None, None, None, None
]

NAMES = (
    'RRR', 'RRP', 'PPR', '3-RR', '2-RR-PP', '3-PR', '2-PR-RR', 'PP-RR-PR', '2-RR-PR', 'PP-PR-RP', 'RR-PR-RP', '2-PR-RP'
)


def _degrees(graph):
    res = tuple([0, 0] for _ in range(len(graph)))
    for index, adj_list in enumerate(graph):
        for joint_id in adj_list:
            if joint_id:
                res[index][joint_id-1] += 1
    return res


DEGREES = tuple(_degrees(g) for g in GRAPHS)
