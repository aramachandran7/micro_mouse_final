"""
class to scan Lidar scan to build graph maze understanding
Theory - building should only interact with four nearest walls
"""


"""
in a 16x16 maze there are 16 nodes.
each node represnts a possible robot position.
The robot being able to move between nodes represents a connection.

"""

class Graph2(object):
    def __init__(self):

        self.graph = {
            # (0,0): [(0,1)]
            # (4,10): [(4,9), (4,11)]
        } # contains a list of connected node positions

    def update_graph(self,pos,walls):
        """
        called after scan after Neato enters new space, before computing next step.
        :param pos: tuple (x,y)
        :param walls: walls always returns in global F B L R ex. []
        """
        #
        # need an easy way to access nodes by position
        if pos in self.graph.keys():
            # visited. we've been here. No need to update graph
            pass
        else:
            # create dict
            self.graph[pos] = []
            if not (walls[0]):
                self.graph[pos].append((pos[0], pos[1]+1))
            if not (walls[1]):
                self.graph[pos].append((pos[0], pos[1]-1))
            if not (walls[2]):
                self.graph[pos].append((pos[0]-1, pos[1]))
            if not (walls[3]):
                self.graph[pos].append((pos[0]+1, pos[1]))
