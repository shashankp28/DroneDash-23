class PathNode:

    def __init__(self, node, angle):
        self.node = node
        self.angle = angle

    def __str__(self):
        string = str(self.node.coordinate) + f"----{self.angle}--->  "
        return string
