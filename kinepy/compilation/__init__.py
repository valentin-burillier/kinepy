from kinepy.compilation.graph_operations import isomorph, degrees, distances, vertices_fusion, make_joint_graph, \
    make_relation_graph, match_graph
from kinepy.compilation.graphs import GRAPHS, DEGREES, SOLVED, NAMES, EDGES, SIGNS
from kinepy.compilation.preparation import *
from kinepy.objects.joints import GhostRevolute, GhostPrismatic
from kinepy.objects.relations import GearRelation


class CompilationError(Exception):
    pass


# ----------------------------------------------- Error messages -------------------------------------------------------

disclaimer = ' (internal movements are not computed)'
HYPERSTATIC_MESSAGE = 'System\'s {} configuration is hyperstatic with degree at least {}'
HYPOSTATIC_MESSAGE = 'System\'s {} configuration is hypostatic or has internal movements with degree at least {}'
HYPERSTATIC_MESSAGE += disclaimer
HYPOSTATIC_MESSAGE += disclaimer
NOT_SOLVED_GRAPH_MESSAGE = "Sorry, unable to solve the graph {} (n° {}).\n" \
                           "See on [explanation page] for further information"
BAD_RELATION_MESSAGE = "{} does not involve exactly 3 eqs that do not depend on inputs"

# ---------------------------------------------- Compiling messages ----------------------------------------------------

SIGNED_GRAPH_MESSAGE = """Step {}:
Identified new signed group {} (n°{}) with joints {}.
Chosen {} as sign.
Sign key: {}.
"""

SIMPLE_GRAPH_MESSAGE = """Step {}:
Identified new group from graph {} (n°{}) with joints {}.
"""

INPUT_MESSAGE = """Step {}:
Solved inputs
"""

RELATION_MESSAGE = """Step {}:
Solved relation {}
"""

# --------------------------------------------------- Constants --------------------------------------------------------

DYNAMICS, KINEMATICS, BOTH = 0, 1, 2

SOLVE_PILOT, SOLVE_GRAPH, SOLVE_RELATION, CSA, SET_ORIGIN, REC_GHOSTS = range(6)
SOLVE_BLOCK, _, _, COMPUTE_MA, COMPUTE_INERTIA, _ = range(6)


class Compiler:
    solved = eqs = sol_to_vertex = distances = joint_queue = before_pilot_mapping = delayed_gear_relations = None

    def __init__(self, system):
        self.system = system
        self.kin_instr, self.dyn_instr = [], []
        self.joint_graph, self.relation_graph = [], []

    def _hyperstatism(self, joints, configuration):
        static_variables = sum(3 - j.dof for j in self.system.joints) + len(self.system.relations)
        h = static_variables - 3 * len(self.system.sols) + 3 + sum(j.dof for j in joints)
        if h > 0:
            raise CompilationError(HYPERSTATIC_MESSAGE.format(configuration, h))
        if h < 0:
            raise CompilationError(HYPOSTATIC_MESSAGE.format(configuration, h))

    def _pre_process(self):
        """gives its rep to each main object (not ghosts), returns rep of piloted and blocked joints"""
        for index, sol in enumerate(self.system.sols):
            sol.rep = index
        for index, joint in enumerate(self.system.joints):
            joint.rep = index
        return tuple(j.rep for j in self.system.piloted), tuple(j.rep for j in self.system.blocked)

    def _set_ghosts(self, piloted_or_blocked):
        solids_with_ghosts = list(self.system.sols)
        joint_dict, uses_ghosts = {}, []

        for joint in self.system.joints:
            # piloted/blocked joints don't use ghosts and are not used to find graphs
            if joint in piloted_or_blocked or joint.dof > 2:
                continue
            if joint.dof == 1:
                joint_dict[joint.rep] = joint
                continue
            joint.ghost_j1.rep = joint.rep
            joint_dict[joint.rep] = joint.ghost_j1

            joint.ghost_j2.rep = len(self.system.joints) + len(uses_ghosts)
            joint_dict[joint.ghost_j2.rep] = joint.ghost_j2

            joint.ghost_sol.rep = len(solids_with_ghosts)
            solids_with_ghosts.append(joint.ghost_sol)
            uses_ghosts.append(joint)
        return solids_with_ghosts, joint_dict, uses_ghosts

    def __call__(self):
        self.system.signs, self.system.tags = {}, {}
        piloted, blocked = self._pre_process()

        # piloted and blocked joints are the same
        if not blocked or set(blocked) == set(piloted):
            self._hyperstatism(self.system.piloted, 'global')
            res = self._set_ghosts(piloted)
            self.system.kin_sols, self.system.kin_joints, self.system.kin_ghosted = res
            self.system.dyn_sols, self.system.dyn_joints, self.system.dyn_ghosted = res
            self.system.kin_instr, self.system.dyn_instr = \
                self._compile(self.system.kin_joints, self.system.kin_sols, list(self.system.piloted), BOTH)
        else:
            self._hyperstatism(self.system.piloted, 'kinematic')
            self._hyperstatism(self.system.piloted, 'dynamic')
            self.system.kin_sols, self.system.kin_joints, self.system.kin_ghosted = self._set_ghosts(piloted)
            self.system.dyn_sols, self.system.dyn_joints, self.system.dyn_ghosted = self._set_ghosts(blocked)
            self.system.kin_instr = \
                self._compile(self.system.kin_joints, self.system.kin_sols, list(self.system.piloted), KINEMATICS)
            self.system.dyn_instr = \
                self._compile(self.system.dyn_joints, self.system.dyn_sols, list(self.system.blocked), DYNAMICS)

    def _compile(self, joints, solids, joint_queue, mode: int):
        if mode:
            print('\nCompiling...\n')

        self.kin_instr, self.dyn_instr = [], []

        # graphs initialisations
        self.joint_graph = make_joint_graph(joints, len(solids))
        self.relation_graph = make_relation_graph(self.system.relations, len(joints))
        self.distances = distances(self.joint_graph)

        # solving state of each joint
        self.solved = [False] * len(joints)
        # eqs are the equivalence classes of the relation: "<x>'s and <y>'s relative positions are solved"
        self.eqs = [(i,) for i in range(len(solids))]
        # correspondence of solid and eqs, eqs are also joint_graph's vertices
        self.before_pilot_mapping = self.sol_to_vertex = list(range(len(solids)))

        running = True
        self.delayed_gear_relations = []
        self.joint_queue = []
        while running:
            # graph is solved, it did not need inputs
            if len(self.joint_graph) == 1:
                return self._finish(mode)
            match = match_graph(self.joint_graph)
            if match is None:
                running = False
                continue
            self._manage_graph(match[0], match[1], solids, joints, mode)
            self._manage_relations(solids, True, mode)

        self.joint_queue = joint_queue
        self.before_pilot_mapping = list(self.before_pilot_mapping)
        self._manage_inputs(solids, mode)
        self._manage_relations(solids, False, mode)

        running = True
        while running:
            if len(self.joint_graph) == 1:
                running = False
                continue
            match = match_graph(self.joint_graph)
            if match is None:
                raise CompilationError("")
            self._manage_graph(match[0], match[1], solids, joints, mode)
            self._manage_relations(solids, False, mode)
        return self._finish(mode)

    def _finish(self, mode):
        if mode:
            print("Compiling done.\n")
        system = self.system
        self.kin_instr += [(CSA, system), (SET_ORIGIN, system), (REC_GHOSTS, system)]
        self.dyn_instr = [(COMPUTE_INERTIA, system), (COMPUTE_MA, system), *self.dyn_instr, (REC_GHOSTS, system)]
        return (self.dyn_instr, self.kin_instr, (self.kin_instr, self.dyn_instr))[mode]

    def _manage_inputs(self, solids, mode):
        if mode:
            print(INPUT_MESSAGE.format(len(self.kin_instr) + 1))

        eq_order = []
        for joint in self.joint_queue:
            self.solved[joint.rep] = True
            vertex1, vertex2 = self.sol_to_vertex[joint.s1.rep], self.sol_to_vertex[joint.s2.rep]
            eq1, eq2 = tuple(solids[i] for i in self.eqs[vertex1]), tuple(solids[i] for i in self.eqs[vertex2])

            eq_order.append((eq1, eq2))
            self.dyn_instr.insert(0, (SOLVE_BLOCK, joint, eq1, eq2, self.distances[vertex2] < self.distances[vertex1]))

            vertices_fusion(self.joint_graph, sorted((vertex1, vertex2)), self.eqs, self.sol_to_vertex)
            self.distances = distances(self.joint_graph)
        self.kin_instr.append((SOLVE_PILOT, self.system, tuple(eq_order)))

    def _manage_graph(self, graph_index, vertex_map, solids, joints, mode: int):
        if not SOLVED[graph_index]:
            raise CompilationError(NOT_SOLVED_GRAPH_MESSAGE.format(NAMES[graph_index], graph_index))

        edges, key = [], []
        for src, dest in ((vertex_map[src1], vertex_map[dest1]) for src1, dest1 in EDGES[graph_index]):
            joint = joints[self.joint_graph[src][dest][1]]
            edges.append((joint, joint.s1.rep in self.eqs[src]))
            key.append(joint.rep)
            if not isinstance(joint, (GhostRevolute, GhostPrismatic)):
                self.solved[joint.rep] = True
                if self.relation_graph[joint.rep]:
                    self.joint_queue.append(joint)
        edges, key, sgn = tuple(edges), tuple(key), SIGNS[graph_index]
        joint_string = set(j.master if isinstance(j, (GhostRevolute, GhostPrismatic)) else j for j, _ in edges)
        if sgn and mode:
            sgn_key = f'{len(self.kin_instr) + 1} {NAMES[graph_index]}'
            self.system.signs[sgn_key] = sgn
            self.system.tags[key] = sgn_key

            print(SIGNED_GRAPH_MESSAGE.format(
                len(self.kin_instr) + 1, NAMES[graph_index], graph_index, ', '.join(repr(j) for j in joint_string),
                sgn, sgn_key
            ))
        elif mode:
            self.system.tags[key] = ()
            print(SIMPLE_GRAPH_MESSAGE.format(
                len(self.kin_instr) + 1, NAMES[graph_index], graph_index, ', '.join(repr(j) for j in joint_string))
            )

        concerned_eqs = tuple(tuple(solids[i] for i in self.eqs[vertex]) for vertex in vertex_map)
        ref = min(enumerate(vertex_map), key=lambda x: self.distances[x[1]])[0]

        self.kin_instr.append((SOLVE_GRAPH, self.system, graph_index, concerned_eqs, edges, key))
        self.dyn_instr.insert(0, (SOLVE_GRAPH, graph_index, concerned_eqs, edges, ref))

        vertices_fusion(self.joint_graph, sorted(vertex_map), self.eqs, self.sol_to_vertex)
        self.distances = distances(self.joint_graph)

    def _common_eq(self, joint1, joint2):
        found = None
        for i, s_rep1 in enumerate((joint1.s1.rep, joint1.s2.rep)):
            for j,  s_rep2 in enumerate((joint2.s1.rep, joint2.s2.rep)):
                if self.before_pilot_mapping[s_rep1] == self.before_pilot_mapping[s_rep2]:
                    if found is not None:
                        return
                    found = i, j
        return found

    def _get_relations(self, delay_allowed):
        while self.joint_queue:
            joint1 = self.joint_queue.pop()
            for joint2, relation, direction in self.relation_graph[joint1.rep]:
                # GearRelation need exactly 3 eqs that do not depend on inputs
                if isinstance(relation, GearRelation):
                    common = self._common_eq(joint1, joint2)
                    if common is None:
                        # common eqs might not be formed yet
                        if delay_allowed:
                            self.delayed_gear_relations.insert(0, joint1)
                            continue
                        # they should be formed by now
                        else:
                            raise CompilationError(BAD_RELATION_MESSAGE.format(relation))
                    relation.common_eq = common
                if self.solved[joint2.rep]:
                    raise CompilationError("Hyperstatic relation model")
                self.solved[joint2.rep] = True
                self.joint_queue.insert(0, joint2)
                self.relation_graph[joint2.rep] = [
                    adj for adj in self.relation_graph[joint2.rep] if not (adj[1] is relation)
                ]
                yield joint2, relation, direction
            self.relation_graph[joint1.rep] = []

    def _manage_relations(self, solids, delay_allowed, mode):
        if not delay_allowed:
            self.joint_queue = self.delayed_gear_relations + self.joint_queue
            self.delayed_gear_relations = []
            self._sub_manage_relations(solids, delay_allowed, mode)
        else:
            running = True
            while running:
                old = self.delayed_gear_relations
                self.joint_queue = self.delayed_gear_relations + self.joint_queue
                self.delayed_gear_relations = []
                self._sub_manage_relations(solids, delay_allowed, mode)
                running = old != self.delayed_gear_relations

    def _sub_manage_relations(self, solids, delay_allowed, mode):
        for joint, relation, direction in self._get_relations(delay_allowed):
            if mode:
                print(RELATION_MESSAGE.format(len(self.kin_instr) + 1, relation))

            vertex1, vertex2 = self.sol_to_vertex[joint.s1.rep], self.sol_to_vertex[joint.s2.rep]
            eq1, eq2 = tuple(solids[i] for i in self.eqs[vertex1]), tuple(solids[i] for i in self.eqs[vertex2])

            self.kin_instr.append((SOLVE_RELATION, relation, direction, eq1, eq2))
            self.dyn_instr.insert(0, (
                SOLVE_RELATION, relation, direction, eq1, eq2, self.distances[vertex2] < self.distances[vertex1]
            ))
            vertices_fusion(self.joint_graph, sorted((vertex1, vertex2)), self.eqs, self.sol_to_vertex)
