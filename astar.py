 """
A star Search Algorithm returns the shortest path between two points on a map.

Arguments:
M: Map object with intersections (ID, X, Y) and roads (adjacency matrix).
start: starting point Intersection ID (integer) 
goal: destination point Intersection ID (integer)
returns: shortest path between start and goal as a list of intersection IDs
 
Note:
Helper classes GraphEdge, GraphNode, Graph are taken from Udacity classroom material, 
(Dijkstra's Algorithm in Advanced Algorithms) and modified where necessary. 

 """

import math
import heapq

def shortest_path(M,start,goal):

# Helper Class, Edge represents road between intersections. Stores the distance.
    class GraphEdge(object):
        def __init__(self, destinationNode, distance):
            self.node = destinationNode
            self.distance = distance

# Helper Class, Node represents Intersections. Edges are children of Nodes. 
    class GraphNode(object):
        def __init__(self, val):
            self.value = val   # intersection index
            self.parent = None
            self.coordx = 0
            self.coordy = 0
            self.gvalue = 0
            self.hvalue = 0
            self.fvalue = math.inf
            self.edges = []     
        def __lt__(self, other):          
            	return self.fvalue < other.fvalue   # ensures that nodes are sorted by min fvalue

        # add or remove children, i.e. Graphedge objects
        def add_child(self, node, distance):		
            self.edges.append(GraphEdge(node, distance))

        def remove_child(self, del_node):          
            if del_node in self.edges:
                self.edges.remove(del_node)

        # calculate Euklidian distance to another node
        def getdistance(self, othernode):
            distance = math.sqrt( math.pow((self.coordx - othernode.coordx), 2) + math.pow((self.coordy - othernode.coordy), 2) )
            return distance

    # Helper class, Graph object represents intersections and roads  
    class Graph(object):
        def __init__(self, node_list):
            self.nodes = node_list

    # adds an edge between node1 and node2 
        def add_edge(self, node1, node2, distance):
            if node1 in self.nodes and node2 in self.nodes:
                node1.add_child(node2, distance)
                #node2.add_child(node1, distance)

        def remove_edge(self, node1, node2):
            if node1 in self.nodes and node2 in self.nodes:
                node1.remove_child(node2)
                node2.remove_child(node1)

    # Create list of nodes from the map of intersections; store x, y coordinates in each node
    nodelist = []
    for key in M.intersections:
        x = M.intersections[key][0]
        y = M.intersections[key][1]
        node = GraphNode(key)
        node.coordx = x
        node.coordy = y
        nodelist.append(node)

    # Create Graph object from Nodes and Edges
    graph = Graph(nodelist)
    for index, list2 in enumerate(M.roads):
        for x in range(len(list2)):
            value = list2[x]
            distance_ = graph.nodes[index].getdistance(graph.nodes[value])
            graph.add_edge(graph.nodes[index], graph.nodes[value], distance_)

    # Dictionary of all nodes of the Graph: key= intersection index, value = node
    nodedict = {}
    for node in graph.nodes:
        nodedict[node.value] = node
 
    # Check if node should be added to open list. 
    def ok_to_open(open, gvalue, neighbour):
        for node in open:
            if (neighbour.value == node.value and gvalue >= node.gvalue):
                return False
        return True

    # Return the path by going through the parents of closed nodes
    def getpath(closed_nodes, goal):
        path = []
        for node in closed_nodes:
            if node.value != start:
               path.append(node.parent)
        path.append(goal) 
        path = list(dict.fromkeys(path))
        return path


    # Begin A star search

    # Declare start node, end node, list of open nodes, list of closed nodes
    start_node = nodedict[start]
    end_node = nodedict[goal]   
    open = []
    closed = []
    
    # Define start node's g, h, f values
    start_node.gvalue = 0                                
    start_node.hvalue = start_node.getdistance(end_node)  # h = euclidian distance to the goal
    start_node.fvalue = start_node.gvalue + start_node.hvalue  # f = g + h
   
    # Push start node into the open list
    current_node = start_node
    heapq.heappush(open, current_node)  

    while len(open) > 0:
    
        # Return node with minimum f and push it into list of closed nodes
        heapq.heapify(open)
        current_node = heapq.heappop(open)
        closed.append(current_node)

        # If goal is reached return path
        if current_node.value == goal:
            path = getpath(closed, goal)
            break

        # loop through the neighbours of current node
        for edge in current_node.edges: 

             # Skip the neighbour if it is already in closed nodes
            if edge.node in closed:      
                continue    
            
            # compute the new g of the neighbour, which is the total travelled distance 
            tmp_gvalue = current_node.gvalue + edge.distance   
              
            # Check if it is OK to add neighbour to the open list. It is not added 
            # if it already exists in the open list with a lower g-value.
            # If the neighbour is added, its f, g, h and parent are updated.
            if(ok_to_open(open, tmp_gvalue, edge.node) == True):        
                edge.node.gvalue = current_node.gvalue + edge.distance                   
                edge.node.hvalue = edge.node.getdistance(end_node) 
                edge.node.fvalue = edge.node.gvalue + edge.node.hvalue   
                edge.node.parent = current_node.value
                heapq.heappush(open, edge.node)

    # return intersection indixes from start to goal
    return path

#___________________________________________________________________

#(5, 34, [5, 16, 37, 12, 34])
#(8, 24, [8, 14, 16, 37, 12, 17, 10, 24])

output = shortest_path(map40,8,24)

#print(output[0].coordx, output[0].coordy, output[0].value )

#print(output[0].coordx, output[0].coordy )
# print list of edges for a give node. using list graph.nodes
#for y in range(len(output[30].edges)):
#    print(output[30].coordx, output[30].coordy, output[30].edges[y].node.value, output[30].edges[y].distance )

#print values of dict_f
#for value in output.values():
#    print(value)

#print node values of dict_f
#for key in output.keys():
#    print(key.value)

#heapq.heapify(output)
#current_node = heapq.heappop(output)
#print(current_node.value)

# print nodes in queue
#for node in output:
#    print(node.value, node.fvalue)

#for x in output:
    #print(x.value, x.fvalue, x.parent)

#for x in output:
   # print(x.value, x.fvalue, x.parent)

for x in output:
    print(x)
