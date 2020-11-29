"""
Objective - to compute the next move for the robot in the mapping phase given the robots current position and map understanding.

design decisions -
isolate heading of robot
"""
import numpy as np

class MoveComputer2(Object):
    def __init__(self):
        self.graph = None
        self.pos = None
        self.confidences = []
        self.center = np.array((7.5, 7.5)) # maze center

    def compute_next_move(self, graph, pos):
        """
        returns next position to move to in global F B L R
        """

        self.graph = graph.graph # graph dict from graph object
        self.pos = np.array(pos) # neato position
        self.confidences = []
        guiding_vector = self.center-self.pos

        if len(self.graph[pos])== 1:
            return self.graph[pos][0]
        else:
            for i, direction in enumerate(self.graph[pos]): # walk through options, compute confidences
                m_vec = np.array(direction) - self.pos
                dp = np.dot(m_vec, guiding_vector) # get dot product
                # multiply confidence by dot product
                num_to_unknown = self.compute_unkown_distance(pos, i) # pass tuple, index
                self.confidences[i] = dp*num_to_unknown # TODO: fix computation

            # pick index of top confdince and get corresponding direction from self.graph
            return self.graph[pos][self.confidences.index(max(self.confidences))]


    def compute_unkown_distance(self, pos, index):
        # using position and index of connection, compute distance to unkown node recursively

        new_node = self.graph[pos][index] # returns tuple

        if !(self.graph.has_key(new_node)):
            return 0
        else:
            for i, direction in self.graph[new_node]:
                if direction != pos:
                    val = self.compute_unkown_distance(new_node, i)

                    if val is not None:
                        return val + 1
            return None # handles case where the only direction was the original node





class MoveComputer(Object):
    """ OLD: """
    def __init__(self):
        self.confidences = [0.0, 0.0, 0.0, 0.0] # f b l r
        self.graph = None
        self.neato_position = None
        self.heading = 0
        self.surrounding_walls = []

    def compute_next_move(self, graph, current_position):
        """
        :param:
        :param current_position:
        :return: returns a direction to move.
        """
        self.graph = graph
        self.neato_position = current_position
        guiding_vector = self.compute_guiding_vector()

        # set walls to 0
        self.set_confidences()

    def set_confidences(self):
        """ set's zeroes if walls"""
        self.surrounding_walls = self.get_walls(self.neato_position, self.heading)
        for i, status in enumerate(self.surrounding_walls):
            if status == 1:
                # if it is a wall
                self.confidences[i] = 0.0
                # distance_to_unknown = self.unkown(i)
            else:
                #  if an opening, evaluate the unknown
                self.confidences[i] = self.evaluate_directions(i)

    def evaluate_directions(self, direction):
        """
        calculate the weight of each direction based on finding unknown places
        """
        # 0 1 2 3 : f b l r
        for i, status in enumrate(self.confidences):
            if status == 1: # if there's an opening, how far till zero
                if i == 0:
                    new_pos = self.neato_position
                find_walls(new_pos)

    def get_walls(self, map_coord, heading):
        """
        returns list of 0, 1, or None for a given map square
        (f r b l) in map frame
        heading = 0, 1, 2, or 3    f, r, b, l
        """
        walls = []
        surrounding_nodes = self.graph.return_surrounding_nodes(map_coord)
        for n in surrounding_nodes:
            walls.append(self.graph.graph[n[0]][n[1]])

        return walls

        # return walls[heading:] + walls[:heading]


    def compute_guiding_vector(self):
        return (7.5-self.neato_position[0],7.5-self.neato_position[1])