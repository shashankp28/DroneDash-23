import math
import numpy as np
from config.Constants import *
import matplotlib.pyplot as plt


class NodeList:

    def __init__(self, node_list=[]):
        self.nodes = []
        self.nodes_dict = {}
        for node in node_list:
            self.add_node(node)

    def add_node(self, node):
        self.nodes.append(node)
        self.nodes_dict[node.coordinate] = len(self.nodes) - 1

    def get_index(self, node):
        if node.coordinate in self.nodes_dict:
            return self.nodes_dict[node.coordinate]
        else:
            return -1

    def get_all_nodes(self):
        return self.nodes

    def return_if_same(self, node):
        min_dist = float('inf')
        index = 0
        if len(self.nodes) == 0:
            return None
        for i in range(len(self.nodes)):
            dist = np.linalg.norm(np.array(self.nodes[i].coordinate) -
                                  np.array(node.coordinate))
            if dist < min_dist:
                min_dist = dist
                index = i
        if min_dist < DISTANCE_THRESHOLD:
            return self.nodes[index]
        return None
    
    def plot(self, path):
        plt.figure(figsize=(10, 10))
        x, y = [], []
        length = 0.3
        for node in self.nodes:
            x.append(node.coordinate[0])
            y.append(node.coordinate[1])
        plt.scatter(x, y, s=50)
        for node in self.nodes:
            for key, value in node.neighbours.items():
                if value==WALL: color ='black'
                elif value==FREE: color ='blue'
                else: color ='red'
                coord = node.coordinate
                angle = key
                angle_rad = math.radians(angle)
                x_end = coord[0] + length * math.cos(angle_rad)
                y_end = coord[1] + length * math.sin(angle_rad)
                plt.plot([coord[0], x_end], [coord[1], y_end], c=color)
                
        for node in self.nodes:
            for key, value in node.get_neighbours().items():
                coord1 = node.coordinate
                coord2 = value.coordinate
                plt.arrow(coord1[0], coord1[1], coord2[0] - coord1[0], 
                          coord2[1] - coord1[1], color='red', 
                          length_includes_head=True, head_width=0.08)
        plt.savefig(path)
        plt.close()

    def size(self):
        return len(self.nodes)
