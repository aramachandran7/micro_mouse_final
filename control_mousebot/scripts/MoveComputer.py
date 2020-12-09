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
        self.debug = True
        self.debug_rec = False
        self.coef = 1
        # self.dp_coef = 1.0

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
            print("Only direction: ", self.graph[pos])
            return self.graph[pos][0]
        else:
            if self.debug:
                print("directions: ", self.graph[pos])
            for i, direction in enumerate(self.graph[pos]): # walk through options, compute confidences
                m_vec = np.array(direction) - self.pos
                dp = ((np.dot(m_vec, guiding_vector)) + 7.5)/15 # get dot product
                # if self.debug:
                #     print("dotproduct for %s, %s" %(direction, dp))
                # multiply confidence by dot product
                if self.debug:
                    print("recursing for ", direction)
                    # print(self.graph)
                num_to_unknown = self.compute_unkown_distance(pos, i) # pass tuple, index
                # if self.debug:
                #     print("%s: num_to_unknown: %s, dp: %s, m_vec: %s, gv: %s" %(direction, num_to_unknown, dp, m_vec, guiding_vector))

                if num_to_unknown is not None:
                    self.confidences.append(dp*(self.coef/(1+(num_to_unknown*2)))) # TODO: fix computation
                else:
                    self.confidences.append(0.0) # there is only a dead end in this direction
            # pick index of top confdince and get corresponding direction from self.graph
            if self.debug:
                print('recursion complete. | current pos: %s, directions: %s, confidences: %s' %(pos, self.graph[pos], self.confidences))
            return self.graph[pos][self.confidences.index(max(self.confidences))]

        # self.publish_vector(guiding_vector[0], guiding_vector[1])


    def compute_unkown_distance(self, pos, index):
        # using position and index of connection, compute distance to unkown node recursively

        new_node = self.graph[pos][index] # returns tuple

        if not new_node in self.graph.keys(): # this is our base case
            return 0
        else:
            if self.debug_rec:
                print("connections to pos %s" %(self.graph[new_node]))
            distances = []
            for i, direction in enumerate(self.graph[new_node]):
                if self.debug_rec:
                    print("pos: %s, i: %s" %(new_node, i))
                if direction != pos:
                    val = self.compute_unkown_distance(new_node, i)
                    if val is not None:
                        distances.append(val+1)

                    # if val is not None:
                    #     return val+1
            #print("found distances: ", distances)

            return min(distances) if (len(distances) != 0) else None  # handles case where the only direction was the original node
            # return None
