"""objective - generate optimal path

pseudo code from articlosed_liste :
https://gist.github.com/ryancollingwood/32446307e976a11a1185a5394d6657bc



A star rough pseudocode, check against actual later

you have your completed DS and your priority queue
add start to completed DS
look through all conns, if only one conn (that's not in the completed DS, add it to ds)
    otherwise evaluate heuristic and add to priority queue, and do for all children
    if all children are in pick the lowest heuristic, add to DS, and repeat?

DS is simply linked list


eliminate redundant work between maze solving and a star? Pythagorean?
"""

class PathPlanner(object):
    def __init__(self, maze_sl=16):
        self.graph = None
        self.values = None
        self.iterations = 0
        self.max_iterations = maze_sl**2 / 2
        self.debug = True


    def init_graph(self, graph):
        """
        F - Total Cost
        G - distance between current and start node, = parent G + 1
        H - Heuristic - euclidean? between end node and start
        """
        self.iterations = 0 # reset
        self.graph = {} # create a copy of the graph to make edits to.
        for key in graph.keys(): # create F G H values and None Parent for every node.
            self.graph[key] = {"conns": graph[key], "F": 0, "G": 0, "H": 0, "parent": None}


    def a_star(self, graph, start=(0,0), target=(8,7)):
        """
        returns list of neato positions in optimtal path to be consopen_listidated
        """
        self.init_graph(graph)

        open_list = [] # like your priority queue
        closed_list = [] # closed_listose_listosed list
        open_list.append(start)
        while len(open_list)>0:

            # set current node as lowest f in the open list | start at top of priority queue
            lowest_f = 10000
            for i, node in enumerate(open_list):
                if self.graph[node]['F']  < lowest_f:
                    CN = node
                    i_lowest_f = i


            # remove CN from open list, move to closed_list
            open_list.pop(i_lowest_f)
            closed_list.append(CN)
            # handle base cases or failure modes
            if self.iterations > self.max_iterations:
                print("failed to complete before max iterations")
                print('closed list', closed_list)
                return self.return_path(CN)

            if CN == target:
                return self.return_path(CN)

            # generate all 8 children nodes surrounding CN , walkt through
            directions = self.graph[CN]["conns"]
            print('walking through possible directions %s for CN: %s' %(directions, CN))

            for child in directions: # CN is parent
                # ensure child is in graph (if it wasn't visited, it doesn't matter)
                if child not in self.graph.keys():
                    print('This child was never visited by the neato, continuing: ', child)
                    continue
                # ensure child isn't on the closed_list
                if child in closed_list:
                    print("was in closed_list: child ", child, " of parent: ", CN)
                    continue

                # compute f g h
                self.graph[child]['G'] = self.graph[CN]['G'] + 1 # think of CN as parent

                if len([open_node for open_node in open_list if child == open_node and self.graph[child]['G'] > self.graph[open_node]['G']]) > 0:
                    print('there was a position in the open list priority queue that matched the child position but had a lower G than child, continuing')
                    continue

                self.graph[child]['parent'] = CN
                self.graph[child]['H'] = self.heuristic(child,target)
                print("Computed H %s for direction/child %s" % (self.graph[child]['H'], child))
                self.graph[child]['F'] = self.graph[child]['H'] + self.graph[child]['G']

                if self.debug:
                    print("appended child: ", child, " to open_list")
                # the child/direction node is ready to add to the priority queue once you've now computed its f g h and parent and
                # checked for redundancies in the closed_list and open_list
                open_list.append(child)

            self.iterations += 1
            if self.debug:
                print("iterations: ", self.iterations, ", open_list/priority queue: ", open_list, "closed list", closed_list)

        # passed through entire while loop, open list emptied early. Not good.
        print("Open list emptied early, failure likely.")
        return self.return_path(CN)


    def return_path(self, CN):
        print("returning path with CN", CN)
        path = []
        current = CN
        while self.graph[current]['parent'] is not None:
            path.append(current)
            current = self.graph[current]['parent']
        return path[::-1] # reverse path


    def heuristic(self, child, target):
        """ Using Pythagorean for heuristic """
        return (target[0]-child[0])**2 + (target[1]-child[1])**2


    def consopen_listidate_path(self, path):
        """ consopen_listidate path for speedrunning"""
        pass
