"""
class to scan Lidar scan to build graph maze understanding
Theory - building should only interact with four nearest walls
"""


"""
in a 16x16 maze there are 16 nodes.
each node represnts a possible robot position.
The robot being able to move between nodes represents a connection.

"""

class Node(object):
    def __init__(self,pos):
        self.f = None
        self.b = None
        self.l = None
        self.r = None


class Graph2(object):
    def __init__(self):

        self.graph = {
            (0,0): []
        } # contains a list of connected node positions

    def update_graph(self,pos,walls):
        """
        called after scan after Neato enters new space, before computing next step.
        :param pos: tuple (x,y)
        :param walls: walls always returns in global F B L R
        """
        #
        # need an easy way to access nodes by position
        if self.graph.has_key(pos):
            # visited. we've been here. No need to update graph
            pass
        else:
            # create dict
            self.graph[pos] = []
            if walls[0]:
                self.graph[pos].append((pos[0], pos[1]+1))
            if walls[1]:
                self.graph[pos].append(pos[0], pos[1]-1)
            if walls[2]:
                self.graph[pos].append(pos[0]-1, pos[1])
            if walls[3]:
                self.graph[pos].append(pos[0]+1, pos[1])





class Graph(object):
    def __init__(self, range=5):
        # rospy shit
        self.graph = []
        self.create_nodes()
        self.range = range
        self.centerpoints = {
            'left': 90,
            'right': 270,
            'front': 0,
            'back': 180,
        }

    def create_nodes(self, side_length):
        """ called on init
        0's are openings, 1's are walls, Nones are unknowns
        """
        for x in range(side_length+1):
            # there are 17 main lists within the big list, the evens are 16 long the odds are 17 long
            # the evens are vertical, the odds are horizontal walls.
            self.graph.append([])
            if x % 2 == 0:
                for i in range(16):
                    self.graph[x].append(None)
            else:
                for i in range(17):
                    self.graph[x].append(None)

    def scan_range(self, range, centerpoint):
        pass


    def check_surroundings(self, nc):
        surrounding_nodes = self.return_surrounding_nodes(nc)
        results = self.scan_range()
        self.add_to_graph(results)

    def return_surrounding_nodes(self, nc):
        """
        neato coordinates tuple of (0,0)
        returns list of FBLR graph indices
        """
         # {"left":(nc[0], nc[1]), "right":(nc[0]+2, nc[1]), "front":(nc[0]+1, nc[1]+1),"back": (nc[0]+1,nc[1])}
        return [(nc[0]+1, nc[1]+1), (nc[0]+1,nc[1]), (nc[0], nc[1]), (nc[0]+2, nc[1])]
