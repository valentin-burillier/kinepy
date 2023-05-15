import numpy as np

from kinepy.compilation import *
from kinepy.math.kinematic import KIN
from kinepy.math.dynamic import DYN


class System:
    dt = inputs = n = None

    def __init__(self, sols, joints, relations, piloted, blocked, interactions):
        self.sols, self.joints, self.relations, self.piloted, self.blocked, self.interactions = \
            sols, joints, relations, piloted, blocked, interactions
        self.signs, self.tags = {}, {}
        self.kin_sols = self.kin_joints = self.dyn_sols = self.dyn_joints = self.kin_ghosted = self.dyn_ghosted = None
        self.dyn_instr, self.kin_instr = [], []
        self.compiler = Compiler(self)

    def reset(self, n):
        self.n = n
        for sol in self.sols:
            sol.reset(n)
        for joint in self.joints:
            joint.reset(n)

    def compile(self):
        """
        Prepares resolution for dynamics and kinematics
        """
        self.compiler()

    def solve_kinematics(self, inputs: np.ndarray):
        self.reset(inputs.shape[1])
        self.inputs = inputs
        for instr in self.kin_instr:
            KIN[instr[0]](*instr[1:])

    def solve_statics(self):
        for sol in self.dyn_sols:
            sol.dyn_reset()
        for instr in self.dyn_instr[1:]:
            DYN[instr[0]](*instr[1:])

    def solve_dynamics(self, t):
        self.dt = t / self.inputs.shape[1]
        for sol in self.dyn_sols:
            sol.dyn_reset()
        for instr in self.dyn_instr:
            DYN[instr[0]](*instr[1:])
