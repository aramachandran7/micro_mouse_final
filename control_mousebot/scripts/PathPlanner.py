"""objective - generate optimal path

pseudo code from article : 
https://gist.github.com/ryancollingwood/32446307e976a11a1185a5394d6657bc

"""

class PathPlanner(object):
    def __init__(self):
        self.graph = None
        self.values = None


    def init_graph(self, graph): 
        self.graph = graph.graph.copy() # create a copy of the graph to make edits to. 
        for key in self.graph.keys(): # create F G H values and None Parent for every node. 
            self.graph[key] = {"cons": self.graph[key], "F": 0, "G": 0, "H": 0, "parent": None} 


    def a_star(self, graph, start=(0,0), target=(7,7)):
        """
        returns list of neato positions in optimtal path to be consolidated
        """
        self.init_graph(graph)

        ol = [] # open list 
        cl = [] # closed list 
        ol.append(start)
        while len(open_list)>0: 


            # set current node as lowest f in the open list 


            # remove CN from open list, move to closed

            # handle if the current node is the end 

            # generate all 8 children nodes surrounding CN , walkt through 
            directions = []

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



    def pythag(self, child, end): 
        # asqured plus bsqured lit
        pass 


    def consolidate_path(self, path): 
        """ consolidate path for speedrunning"""
        pass 