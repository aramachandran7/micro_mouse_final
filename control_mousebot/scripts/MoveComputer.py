"""
Objective - to compute the next move for the robot in the mapping phase given the robots current position and map understanding.
"""

class MoveComputer(Object):
    def __init__(self):
        self.confidences = [0.0, 0.0, 0.0, 0.0] # f b l r
        self.graph = None
        self.neato_position = None

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
        self.check_for_walls()

        self.evaluate_directions()


    def evaluate_directions(self):
        # 0 1 2 3 : f b l r
        for i, status in enumrate(self.confidences):
            if status == 1: # if there's an opening, how far till zero
                if i == 0:
                    new_pos = self.neato_position
                find_walls(new_pos)



    def check_for_walls(self):
        """ set's zeroes if walls"""
        surrounding_nodes = graph.return_surrounding_nodes(self.neato_position)
        for i, direction in enumerate(surrounding_nodes):
            status = graph.graph[direction[0]][direction[1]]
            if status != 1:
                # if its an opening
                self.confidences[i] = 1.0
                # distance_to_unknown = self.unkown(i)
            else:
                self.confidences[i] = 0.0


    def compute_guiding_vector(self):
        return (7.5-self.neato_position[0],7.5-self.neato_position[1])
