"""
Objective - to compute the next move for the robot in the mapping phase given the robots current position and map understanding.

design decisions -
isolate heading of robot
implement memoizaiton w/ memo table?

save dictionary information


guardian angel -
think long term, and pathplan to that point. How do you select which point is the best and use different metrics
than what we'd use by default?

adjust confidence compute strategy with circular buffer of nodes visited
"""
import numpy as np
import math

class MoveComputer2(object):
    def __init__(self):
        self.graph = None
        self.pos = None
        self.confidences = []
        self.target = np.array((7.5, 7.5)) # maze target
        self.debug = False
        self.debug_rec = False
        self.coef = 1.0
        self.recursion_limit = 80
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
                # dp = ((np.dot(m_vec, guiding_vector)) + 7.5)/15 # get dot product TODO: fix scaling for global vars
                # if self.debug:
                #     print("dotproduct for %s, %s" %(direction, dp))
                # multiply confidence by dot product
                if self.debug:
                    print("recursing for ", direction)
                    # print(self.graph)
                result = self.compute_unknown_distance2(pos, i, 0) # pass tuple, index


                # if self.debug:
                #     print("%s: num_to_unknown: %s, dp: %s, m_vec: %s, gv: %s" %(direction, num_to_unknown, dp, m_vec, guiding_vector))

                if result is not None:
                    num_to_unknown, location = result
                    distance_to_center = math.sqrt((location[1]-self.target[1])**2 + (location[0]-self.target[0])**2)/math.sqrt(2*(7.5**2)) # size matters; smaller is better
                    num_to_unknown /= self.recursion_limit
                    if self.debug: 
                        print("distance to center: ", distance_to_center, "num to unknown: ", num_to_unknown)
                    self.confidences.append(1/(distance_to_center*(1.0+num_to_unknown*6))) # TODO: fix computation  50 WORKED!!
                else:
                    self.confidences.append(0.0) # there is only a dead end in this direction
            # pick index of top confdince and get corresponding direction from self.graph
            if self.debug:
                print('recursion complete. | current pos: %s, directions: %s, confidences: %s' %(pos, self.graph[pos], self.confidences))
            return self.graph[pos][self.confidences.index(max(self.confidences))]

        # self.publish_vector(guiding_vector[0], guiding_vector[1])


    def compute_unknown_distance(self, pos, index, depth_recursions):
        """ Other things to try -
        could store visited nodes in every recursive call, check if node has been visited return none.
        Variable memory.

        It doesn't store it's previous moves, only its understanding of the map. So when the robot is in a part of the map
        it's already explored, it has a high liklihood to get stuck in a 'back and forth' loop


        """
        # using position and index of connection, compute distance to unknown node recursively
        new_node = self.graph[pos][index] # returns tuple

        if not new_node in self.graph.keys(): # this is our base case
            return 0
        else:
            if depth_recursions>self.recursion_limit:
                print("you're in a loop! Returning None.")
                return None

            if self.debug_rec:
                print("connections to pos %s" %(self.graph[new_node]))

            depth_recursions += 1
            distances = []
            for i, direction in enumerate(self.graph[new_node]):
                if self.debug_rec:
                    print("pos: %s, i: %s" %(new_node, i))
                if direction != pos:
                    val = self.compute_unknown_distance(new_node, i, depth_recursions)
                    if val is not None:
                        distances.append(val+1)

                    # if val is not None:
                    #     return val+1
            #print("found distances: ", distances)
            return min(distances) if (len(distances) != 0) else None  # handles case where the only direction was the original node
            # return None

    def compute_unknown_distance2(self, pos, index, depth_recursions):
        """ Other things to try -
        could store visited nodes in every recursive call, check if node has been visited return none.
        Variable memory.

        It doesn't store it's previous moves, only its understanding of the map. So when the robot is in a part of the map
        it's already explored, it has a high liklihood to get stuck in a 'back and forth' loop

        returns (distance to unknown, tuple of unknown position)
        """
        # using position and index of connection, compute distance to unknown node recursively
        new_node = self.graph[pos][index] # returns tuple

        if not new_node in self.graph.keys(): # this is our base case
            return (0, new_node)
        else:
            if depth_recursions>self.recursion_limit:
                print("you're in a loop! Returning None.")
                return None

            if self.debug_rec:
                print("connections to pos %s" %(self.graph[new_node]))

            depth_recursions += 1
            distances = []
            for i, direction in enumerate(self.graph[new_node]):
                if self.debug_rec:
                    print("pos: %s, i: %s" %(new_node, i))
                if direction != pos:
                    result = self.compute_unknown_distance2(new_node, i, depth_recursions)
                    if result is not None:
                        val,unkown_node = result
                        distances.append((val+1, unkown_node))

            return min(distances, key = lambda i : i[0]) if (len(distances) != 0) else None # handles case where the only direction was the original node
