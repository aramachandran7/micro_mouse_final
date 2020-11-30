"""objective - generate optimal path

pseudo code from article : 
https://gist.github.com/ryancollingwood/32446307e976a11a1185a5394d6657bc

"""

class PathPlanner(object):
    def __init__(self):
        self.graph = None
        self.values = None
        self.iterations = 0
        self.max_iterations = 60


    def init_graph(self, graph): 
        self.iterations = 0 # reset
        self.graph = graph.graph.copy() # create a copy of the graph to make edits to. 
        for key in self.graph.keys(): # create F G H values and None Parent for every node. 
            self.graph[key] = {"conns": self.graph[key], "F": 0, "G": 0, "H": 0, "parent": None} 


    def a_star(self, graph, start=(0,0), target=(7,7)):
        """
        returns list of neato positions in optimtal path to be consolidated
        """
        self.init_graph(graph)

        ol = [] # open list 
        cl = [] # closed list 
        ol.append(start)
        while len(ol)>0: 


            # set current node as lowest f in the open list 
            lowest_f = 10000
            for i, node in enumerate(ol): 
                if self.graph[node].f  < lowest_f: 
                    CN = node
                    i_lowest_f = i


            # remove CN from open list, move to closed
            ol.pop(i_lowest_f)
            # handle base cases or failure modes
            if self.iterations > self.max_iterations: 
                print("failed to complete before max iterations")
                return self.return_path(CN)

            if CN == target: 
                return self.return_path(CN)                

            # generate all 8 children nodes surrounding CN , walkt through 
            directions = self.graph[CN]["conns"]

            for child in directions: 
                # ensure child isn't on the closed list 

                if child in cl: 
                    continue 
                # compute f g h

                self.graph[child].g = CN.g + 1
                self.graph[child].h = self.pythag(child,end)
                self.graph[child].f = self.graph[child].h + self.graph[child].g


                for open_node in ol: 
                    if child == open_node and self.graph[child].g > self.graph[open_node].g: 
                        continue

                # finall add the fucking kid to hte OL 
                ol.append(child)
            self.iterations += 1


    def return_path(self, CN): 
        path = []
        current = CN 
        while current.parent is not None: 
            path.append(current)
            current = self.graph[current].parent
        return path[::-1] # reverse path 


    def pythag(self, child, end): 
        # asqured plus bsqured lit
        pass 


    def consolidate_path(self, path): 
        """ consolidate path for speedrunning"""
        pass 