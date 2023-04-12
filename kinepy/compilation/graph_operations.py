from kinepy.compilation.graphs import GRAPHS, DEGREES


def make_joint_graph(joints, solid_number):
    """Adjacency matrix of the joint graph, not oriented"""
    joint_graph = [[(0, 0)] * solid_number for _ in range(solid_number)]
    for index, joint in joints.items():
        # Adjacency is (Type of the joint, index of the joint)
        joint_graph[joint.s1.rep][joint.s2.rep] = joint_graph[joint.s2.rep][joint.s1.rep] = (joint.id_, index)
    return joint_graph


def make_relation_graph(relations, joint_number):
    """Adjacency list of the relation graph, oriented"""
    relation_graph: list[list[tuple]] = [[] for _ in range(joint_number)]
    for rel in relations:
        j1, j2 = rel.j1.rep, rel.j2.rep
        # Adjacency is (the other related joint, the relation object, is it from joint1 to joint2)
        relation_graph[j1].append((rel.j2, rel, True))
        relation_graph[j2].append((rel.j1, rel, False))
    return relation_graph


def distances(graph):
    """Ditance of vertices to the vertex 0, each edge has weight 1"""

    dists, visited, queue = [0] * len(graph), [False] * len(graph), [0]
    visited[0] = True

    # Breadth-first search
    while queue:
        current = queue.pop(0)
        for index, (joint_id, _) in enumerate(graph[current]):
            if joint_id and not visited[index]:
                visited[index] = True
                dists[index] = dists[current] + 1
                queue.append(index)
    return dists


def degrees(graph):
    """
    degrees of vertices in terms of 'R' edges and 'P' edges
    """

    res = tuple([0, 0] for _ in range(len(graph)))
    for index, adj_list in enumerate(graph):
        for joint_id, _ in adj_list:
            if joint_id:
                res[index][joint_id - 1] += 1
    return res


def inf(deg1, deg2):
    """
    R-P-degrees is the heuristic for ismorphism research
    """
    return deg1[0] <= deg2[0] and deg1[1] <= deg2[1]


def can_match(source_graph, dest_graph, vertex_map):
    """
    isomorphism sub-function: checks if the vertex_map correctly maps the currrent n vertices of the source graph
    on n vertices of the dest_graph, assuming all points except the last one are correct
    """
    n = len(vertex_map) - 1
    for source, dest in enumerate(vertex_map[:-1]):
        if source_graph[source][n] != dest_graph[dest][vertex_map[n]][0]:
            return False
    return True


def isomorph(src_graph, dest_graph, src_deg, dest_deg, vertex_map):
    """
    Searches a sub-graph of dest that is isomorphic to src
    """
    if len(vertex_map) == len(src_graph):
        return vertex_map
    for vertex in range(len(dest_graph)):
        if vertex not in vertex_map and inf(src_deg[len(vertex_map)], dest_deg[vertex]) and can_match(src_graph, dest_graph, vertex_map + (vertex,)):
            new_map = isomorph(src_graph, dest_graph, src_deg, dest_deg, vertex_map + (vertex,))
            if new_map:
                return new_map
    return ()


def match_graph(joint_graph) -> tuple[int, tuple]:
    dest_deg = degrees(joint_graph)
    for index, (graph, src_deg) in enumerate(zip(GRAPHS, DEGREES)):
        vertex_map = isomorph(graph, joint_graph, src_deg, dest_deg, ())
        if vertex_map:
            return index, vertex_map
    # s = '\n'.join(map(str, joint_graph))
    # raise CompilationError(f"None of the known graphs matches the current system graph \n{s}\n"
    #                      f"Either it is hyperstatic or it is not supported")


def vertices_fusion(joint_graph, vertices, eqs, sol_to_vertex):
    """merges all eqs descriped in vertices, vertices is sorted"""
    dest_vertex = vertices[0]

    # going in the reverse order, dest_vertiex is excluded
    for vertex in vertices[:0:-1]:
        # merging eqs
        eqs[dest_vertex] += eqs[vertex]
        # replacing the vertex with the last one
        eqs[vertex] = eqs[-1]
        for sol in eqs[vertex]:
            # updating solids' vertices
            sol_to_vertex[sol] = vertex
        eqs.pop()

        # merging on the graph
        for index, dest_joint in enumerate(joint_graph[vertex]):
            # gathering of edges on the lowest vertex
            if dest_joint[0]:
                joint_graph[dest_vertex][index] = joint_graph[index][dest_vertex] = dest_joint
        # replacing the vertex with the last one
        for index, copy_joint in enumerate(joint_graph[-1][:-1]):
            joint_graph[vertex][index] = joint_graph[index][vertex] = copy_joint
        # makes sure the vertex is not conected to itself
        joint_graph[vertex][vertex] = 0, 0

        # gets rid of the last vetex
        joint_graph.pop()
        for row in joint_graph:
            row.pop()

    # updates solids' vetices
    for sol in eqs[dest_vertex]:
        sol_to_vertex[sol] = dest_vertex
    # makes sure the vertex is not conected to itself
    joint_graph[dest_vertex][dest_vertex] = 0, 0
