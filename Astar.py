#!/usr/bin/env python

import math
import random
import matplotlib.pyplot as plt
import heapq

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), "py-bindings"))
    from ompl import base as ob
    from ompl import geometric as og


class Node():
    """A node class for A* pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position.getX() == other.position.getX() and self.position.getY() == other.position.getY()
        #return self.position[0] == other.position[0] and self.position[1] == other.position[1]

    def __str__(self):
        return str(self.position.getX()) + ", " + str(self.position.getY()) + ", " + str(self.position.getYaw()*180/math.pi)

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f


class Astar(ob.Planner):
    def __init__(self, si):
        super(Astar, self).__init__(si, "Astar")
        self.states_ = []
        self.sampler_ = si.allocStateSampler()

    def solve(self, ptc):
        pdef = self.getProblemDefinition()
        goal = pdef.getGoal()
        si = self.getSpaceInformation()
        pi = self.getPlannerInputStates()
        st = pi.nextStart()
        while st:
            self.states_.append(st)
            st = pi.nextStart()
        solution = None
        approxsol = 0
        approxdif = 1e6
        start_state = pdef.getStartState(0)
        goal_state = goal.getState()
        start_node = Node(None, start_state)
        start_node.g = start_state.h = start_node.f = 0
        end_node = Node(None, goal_state)
        end_node.g = end_node.h = end_node.f = 0

        open_list = []
        closed_list = []
        heapq.heapify(open_list)
        adjacent_squares = ((1, 0, 0), (1, 1, 45), (0, 1, 90), (-1, 1, 135),
                            (-1, 0, 0), (-1, -1, -135), (0, -1, -90), (1, -1, -45))

        heapq.heappush(open_list, start_node)
        while len(open_list) > 0 and not ptc():
            current_node = heapq.heappop(open_list)
            if current_node == end_node:  # if we hit the goal
                current = current_node
                path = []
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                for i in range(1, len(path)):
                    self.states_.append(path[len(path) - i - 1])
                solution = len(self.states_)
                break
            closed_list.append(current_node)

            children = []
            for new_position in adjacent_squares:
                node_position = si.allocState()
                current_node_x = current_node.position.getX()
                current_node_y = current_node.position.getY()
                node_position.setXY(current_node_x + new_position[0], current_node_y + new_position[1])
                node_position.setYaw(new_position[2] * math.pi / 180)

                if not si.checkMotion(current_node.position, node_position):
                    continue
                if not si.satisfiesBounds(node_position):
                    continue
                new_node = Node(current_node, node_position)
                children.append(new_node)

            for child in children:
                if child in closed_list:
                    continue
                if child.position.getYaw() % (math.pi/2) == 0:
                    child.g = current_node.g + 1
                else:
                    child.g = current_node.g + math.sqrt(2)
                child.h = goal.distanceGoal(child.position)
                child.f = child.g + child.h
                if len([i for i in open_list if child == i and child.g >= i.g]) > 0:
                    continue
                heapq.heappush(open_list, child)

        solved = False
        approximate = False
        if not solution:
            solution = approxsol
            approximate = False
        if solution:
            path = og.PathGeometric(si)
            for s in self.states_[:solution]:
                path.append(s)
            pdef.addSolutionPath(path)
            solved = True
        return ob.PlannerStatus(solved, approximate)

    def clear(self):
        super(Astar, self).clear()
        self.states_ = []


def isStateValid(state):
    x = state.getX()
    y = state.getY()
    return True#(x - 0) * (x - 0) + (y - 0) * (y - 0) > 0 * 0


def dist_between_states(state1, state2):
    return math.sqrt(math.pow(state2[0] - state1[0], 2) + math.pow(state2[1] - state1[1], 2))


def plan():
    N = 100
    # create a state space
    space = ob.ReedsSheppStateSpace(2)
    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(N)
    space.setBounds(bounds)
    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    start = ob.State(space)
    start[0] = 1  # random.randint(0, int(N / 2))
    start[1] = 1  # random.randint(0, int(N / 2))
    goal = ob.State(space)
    goal[0] = N-1  # random.randint(int(N / 2), N)
    goal[1] = N-1  # random.randint(int(N / 2), N)
    ss.setStartAndGoalStates(start, goal)
    planner = Astar(ss.getSpaceInformation())
    ss.setPlanner(planner)

    result = ss.solve(10)
    if result:
        if result.getStatus() == ob.PlannerStatus.APPROXIMATE_SOLUTION:
            print("Solution is approximate")
        matrix = ss.getSolutionPath().printAsMatrix()
        print(matrix)
        verts = []
        for line in matrix.split("\n"):
            x = []
            for item in line.split():
                x.append(float(item))
            if len(x) != 0:
                verts.append(list(x))
        plt.axis([0, N, 0, N])
        x = []
        y = []
        for i in range(0, len(verts)):
            x.append(verts[i][0])
            y.append(verts[i][1])
        plt.plot(x, y, 'ro-')
        plt.show()


if __name__ == "__main__":
    plan()
