import numpy as np
from config.Constants import *


class Node:

    def __init__(self, coordinate):
        self.coordinate = coordinate
        self.neighbours = {
            0: WALL,
            90: WALL,
            180: WALL,
            270: WALL
        }
        self.distance = float("inf")
        self.previous = None

    def get_neighbours(self):
        valid = {key: value for key, value in
                 self.neighbours.items() if value != WALL and value != FREE}
        return valid

    def distance_to(self, node):
        return np.linalg.norm(np.array(self.coordinate) - np.array(node.coordinate))

    def has_free(self):
        return FREE in self.neighbours.values()

    def __str__(self):
        string = ""
        string += "Coordinate: " + str(self.coordinate) + " "
        temp_neighbours = {}
        for key, value in self.neighbours.items():
            if value == WALL:
                temp_neighbours[key] = "WALL"
            elif value == FREE:
                temp_neighbours[key] = "FREE"
            else:
                temp_neighbours[key] = value.coordinate
        string += str(temp_neighbours) + "\n"
        string += "-"*50
        return string

    def __lt__(self, other):
        return self.distance < other.distance
