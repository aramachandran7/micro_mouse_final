"""objective - generate optimal path

pseudo code from article :
https://gist.github.com/ryancollingwood/32446307e976a11a1185a5394d6657bc



A star pseudocode

you have an open list and closed list
"""

class PathPlanner(object):
    def __init__(self, maze_sl=16):
        self.graph = None
        self.values = None
        self.iterations = 0
        self.max_iterations = maze_sl**2 / 2
        self.debug = True


    def init_graph(self, graph):
        self.iterations = 0 # reset
        self.graph = {} # create a copy of the graph to make edits to.
        for key in graph.keys(): # create F G H values and None Parent for every node.
            self.graph[key] = {"conns": graph[key], "F": 0, "G": 0, "H": 0, "parent": None} # TODO: Set these values?


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
                if self.graph[node]['F']  < lowest_f:
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

            for child in directions: # CN is parent

                # ensure child isn't on the closed list
                if child in cl:
                    continue

                # compute f g h
                self.graph[child]['G'] = self.graph[CN]['G'] + 1

                if len([open_node for open_node in ol if child == open_node and self.graph[child]['G'] > self.graph[open_node]['G']]) > 0:
                    continue

                self.graph[child]['H'] = self.pythag(child,target)
                self.graph[child]['F'] = self.graph[child]['H'] + self.graph[child]['G']

                if self.debug:
                    print("appended child: ", child, " to OL")
                ol.append(child)

            self.iterations += 1
            if self.debug:
                print("iterations: ", self.iterations, ", OL: ", ol)


    def return_path(self, CN):
        path = []
        current = CN
        while self.graph[current]['parent'] is not None:
            path.append(current)
            current = self.graph[current]['parent']
        return path[::-1] # reverse path


    def pythag(self, child, target):
        # asqured plus bsqured lit
        return (target[0]-child[0])**2 + (target[1]-child[1])**2


    def consolidate_path(self, path):
        """ consolidate path for speedrunning"""
        pass
