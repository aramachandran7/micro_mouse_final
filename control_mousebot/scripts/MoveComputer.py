"""
Objective - to compute the next move for the robot in the mapping phase given the robots current position and map understanding.

design decisions -
isolate heading of robot
"""

class MoveComputer(Object):
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
