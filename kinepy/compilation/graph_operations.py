
def distances(graph):
    """
    Ditance of vertices to the vertex 0, each edge has weight 1
    """

    dists = [0] * len(graph)
    visited = [False] * len(graph)
    queue = [0]
    visited[0] = True
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


def can_match(source_graph, dest_graph, vertex_map):  # just testing the last-added point
    """
    isomorphism sub-function: checks if the vertex_map correctly maps the currrent n vertices of the source graph
    on n vertices of the dest_graph
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


def vertices_fusion(joint_graph, vertices, eqs, sol_to_vertex):  # vertices are sorted
    dest_vertex = vertices[0]

    for vertex in vertices[:0:-1]:

        eqs[dest_vertex] += eqs[vertex]
        eqs[vertex] = eqs[-1]
        for sol in eqs[vertex]:
            sol_to_vertex[sol] = vertex
        eqs.pop()

        for index, dest_joint in enumerate(joint_graph[vertex]):
            # gathering of edges on the lowest vertex
            if dest_joint[0]:
                joint_graph[dest_vertex][index] = joint_graph[index][dest_vertex] = dest_joint

        for index, copy_joint in enumerate(joint_graph[-1][:-1]):
            # refactor of the graph
            joint_graph[vertex][index] = joint_graph[index][vertex] = copy_joint
        joint_graph[vertex][vertex] = 0, 0

        joint_graph.pop()
        for row in joint_graph:
            row.pop()
    for sol in eqs[dest_vertex]:
        sol_to_vertex[sol] = dest_vertex
    joint_graph[dest_vertex][dest_vertex] = 0, 0
