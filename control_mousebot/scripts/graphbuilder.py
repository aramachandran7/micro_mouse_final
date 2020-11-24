"""
class to scan Lidar scan to build graph maze understanding 
Theory - building should only interact with four nearest walls 
"""

class Node(object): 
    def __init__(self, x,y, left=None, right=None, front=None, back=None): 
        self.left = left
        self.right = right
        self.front = front
        self.back = back
        self.x = x
        self.y = y

    def update(self): 
        pass 

class Graph(object): 
    def __init__(self, dimensions=(64,64)): 
        self.create_nodes(dimensions)
    
    def create_nodes(self, dimensions): 
        for x in range(dimensions[0]): 
            for y in range(dimensions[1]): 
                 


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

    def add_to_graph(self, results): 
        pass 

    def scan_range(self, range, centerpoint): 
        pass

    def check_nodes(self): 
        """
        Check all adjacent nodes in graph
        """
    
    def check_surroundings(self): 
        results = self.scan_range()
        self.add_to_graph(results)

