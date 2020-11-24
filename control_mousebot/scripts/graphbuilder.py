"""
class to scan Lidar scan to build graph maze understanding 
Theory - building should only interact with four nearest walls 
"""

class Graph(object): 
    def __init__(self, side_length=(16)): 
        self.create_nodes(side_length)
        self.graph = []
    
    def create_nodes(self, side_length): 
        for x in range(side_length+1): 
            # there are 17 main lists within the big list, the evens are 16 long the odds are 17 long 
            # the evens are vertical, the odds are horizontal walls. 
            self.graph.append([])
            if x % 2 == 0: 
                for i in range(16): 
                    self.graph[x].append(False)
            else: 
                for i in range(17): 
                    self.graph[x].append(False)
                
                

class GraphBuilder(object):
    def __init__(self, range=5): 
        # rospy shit 
        self.graph = Graph()
        self.range = range
        self.centerpoints = {
            'left': 90,
            'right': 270,
            'front': 0,
            'back': 180,
        }


    def scan_range(self, range, centerpoint): 
        pass

    
    def check_surroundings(self, nc): 
        surrounding_nodes = self.return_surrounding_nodes(nc)
        results = self.scan_range()
        self.add_to_graph(results)

    def return_surrounding_nodes(self, nc): 
        """
        neato coordinates tuple of (0,0)
        """
        surrounding_nodes = {"left":(nc[0], nc[1]), "right":(nc[0]+2, nc[1]), "front":(nc[0]+1, nc[1]+1),"back": (nc[0]+1,nc[1])}
        return surrounding_nodes

