import sys
from constants import *
from environment import *
from state import State
from tester import *
from play import *
import heapq
"""
solution.py

This file is a template you should use to implement your solution.

You should implement

COMP3702 2022 Assignment 1 Support Code

Last updated by njc 01/08/22
"""


class Solver:

    def __init__(self, environment, loop_counter):
        self.environment = environment
        self.loop_counter = loop_counter

        #
        # TODO: Define any class instance variables you require here.
        #

    def solve_ucs(self):
        """
        Find a path which solves the environment using Uniform Cost Search (UCS).
        :return: path (list of actions, where each action is an element of ROBOT_ACTIONS)
        """
        #
        #
        # TODO: Implement your UCS code here
        #
        # === Important ================================================================================================
        # To ensure your code works correctly with tester, you should include the following line of code in your main
        # search loop:
        #
        # self.loop_counter.inc()
        #
        # e.g.
        # while loop_condition():
        #   self.loop_counter.inc()
        #   ...
        #
        # ==============================================================================================================
        #
        #
        # while loop_condition():
        #    self.loop_counter.inc()
        state = self.environment.get_init_state()
        env = self.environment
        container = [state]
        heapq.heapify(container)
        # dict: state --> path_cost
        visited = {state: 0}
        n_expanded = 0

        while len(container) > 0:
            self.loop_counter.inc()
            n_expanded += 1
            node = heapq.heappop(container)

            # check if this state is the goal
            if env.is_solved(node):
                # print("solved")
                return node.get_path()

            # add unvisited (or visited at higher path cost) successors to container
            successors = node.get_successors()
            for s in successors:
                if s not in visited.keys() or s.action_cost < visited[s]:
                   # print("Added")
                    visited[s] = s.action_cost
                    heapq.heappush(container, s)

        return None

    def solve_a_star(self):
        """
        Find a path which solves the environment using A* search.
        :return: path (list of actions, where each action is an element of ROBOT_ACTIONS)
        """

        #
        #
        # TODO: Implement your A* search code here
        #
        # === Important ================================================================================================
        # To ensure your code works correctly with tester, you should include the following line of code in your main
        # search loop:
        #
        # self.loop_counter.inc()
        #
        # e.g.
        # while loop_condition():
        #   self.loop_counter.inc()
        #   ...
        #
        # ==============================================================================================================
        #
        #
        state = self.environment.get_init_state()
        env = self.environment
        heuristic = 0 + state.get_heuristic()
        print("Target list: ")
        print(env.target_list)

        print("State Widget Centres: ")
        print(state.widget_centres)
        
        
        container = [(heuristic, state)]
        heapq.heapify(container)
        # dict: state --> path_cost
        visited = {state: 0}
        n_expanded = 0

        while len(container) > 0:
            self.loop_counter.inc()
            n_expanded += 1
            _, node = heapq.heappop(container)

            # check if this state is the goal
            if env.is_solved(node):
                # print("solved")
                return node.get_path()

            # add unvisited (or visited at higher path cost) successors to container
            successors = node.get_successors()
            for s in successors:
                if s not in visited.keys() or s.action_cost < visited[s]:
                   # print("Added")
                    visited[s] = s.action_cost
                    heapq.heappush(container, (s.action_cost + heuristic, s))
        return None

        pass
        
    #
    #
    # TODO: Add any additional methods here
    #
    #
