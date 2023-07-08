from classes.Node import Node
from classes.NodeList import NodeList
from classes.Algorithm import Algorithm
from config.Constants import *
from plotter import plot

nodes = [Node((0, 0)), Node((2, 0)), Node((2, 2)), Node((2, 6)), Node((0, 6)), Node((5, 6)), Node((5, 2))]
nodes[0].neighbours[0] = nodes[1]

nodes[1].neighbours[90] = nodes[2]
nodes[1].neighbours[180] = nodes[0]

nodes[2].neighbours[0] = nodes[6]
nodes[2].neighbours[90] = nodes[3]
nodes[2].neighbours[270] = nodes[1]

nodes[3].neighbours[0] = nodes[5]
nodes[3].neighbours[90] = FREE
nodes[3].neighbours[180] = nodes[4]
nodes[3].neighbours[270] = nodes[2]

nodes[4].neighbours[0] = nodes[3]

nodes[5].neighbours[90] = FREE
nodes[5].neighbours[180] = nodes[3]
nodes[5].neighbours[270] = nodes[6]

nodes[6].neighbours[90] = nodes[5]
nodes[6].neighbours[180] = nodes[2]

node_list = NodeList(nodes)

plot(node_list)

algo = Algorithm()
path = algo.dijkstra(nodes[3], node_list)

for node in path:
    print(node, end = " ")
    
print()