"""
Objective - to compute the next move for the robot in the mapping phase given the robots current position and map understanding.

design decisions -
isolate heading of robot
implement memoizaiton w/ memo table?
"""
import numpy as np
import math

class MoveComputer2(object):
    def __init__(self):
        self.graph = None
        self.pos = None
        self.confidences = []
        self.target = np.array((7.5, 7.5)) # maze target
        self.coef = 1.0
        self.debug = False

    def compute_next_move(self, graph, pos):
        """
        returns next position to move to in global F B L R
        """

        self.graph = graph.graph # graph dict from graph object
        self.pos = np.array(pos) # neato position
        self.confidences = []
        guiding_vector = self.target-self.pos
        print("Current Position     ", pos)
        if len(self.graph[pos])== 1:
            return self.graph[pos][0]
        else:
            for i, direction in enumerate(self.graph[pos]): # walk through options, compute confidences
                m_vec = np.array(direction) - self.pos
                dp = math.fabs(np.dot(m_vec, guiding_vector)) # get dot product
                if self.debug:
                    print("dotproduct for %s, %s" %(i, dp))
                # multiply confidence by dot product
                num_to_unknown = self.compute_unkown_distance(pos, i) # pass tuple, index
                if self.debug:
                    print("num_to_unknown for %s, %s" %(i, num_to_unknown))

                if num_to_unknown is not None:
                    self.confidences.append(dp*(self.coef/(1+num_to_unknown))) # TODO: fix computation
                else:
                    self.confidences.append(0.0) # there is only a dead end in this direction
            # pick index of top confdince and get corresponding direction from self.graph
            if self.debug:
                print('current pos: %s, directions: %s, confidences: %s' %(pos, self.graph[pos], self.confidences))
            return self.graph[pos][self.confidences.index(max(self.confidences))]


    def compute_unkown_distance(self, pos, index):
        # using position and index of connection, compute distance to unkown node recursively

        new_node = self.graph[pos][index] # returns tuple

        if not new_node in self.graph.keys():
            return 0
        else:
        #    print("connections to pos %s" %(self.graph[new_node]))
            for i, direction in enumerate(self.graph[new_node]):
        #        print("pos: %s, i: %s" %(new_node, i))
                if direction != pos:
                    val = self.compute_unkown_distance(new_node, i)

                    if val is not None:
                        return val + 1
            return None # handles case where the only direction was the original node
