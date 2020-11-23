"""
class to scan Lidar scan to build graph understanding 
"""

class Graph(object): 
    def __init__(self, dimensions=(64,64)): 
        self.create_nodes(dimensions)
    
    def create_nodes(self, dimensions): 
        pass


class GraphBuilder(object):
    def __init__(self, range=5): 
        # rospy shit 
        self.Graph = Graph()
        self.range = range
        self.centerpoints = {
            'left': 90
            'right': 270
            'front': 0
            'back': 180
        }

    def add_to_graph(self, results): 
        pass 

    def scan_range(self, range, centerpoint): 
        pass
    
    def check_surroundings(self): 
        results = self.scan_range()
        self.add_to_graph(results)

