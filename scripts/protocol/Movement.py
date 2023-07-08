import numpy as np
from classes.Node import Node
from config.Constants import *
from classes.Timer import Timer
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

class Movement:
    
    def __init__(self, pose):
        self.pose = pose
        self.state = INIT
        self.current_angle = 0
        self.timer = Timer()
        self.current_angle = 0
        self.path = []
        self.destination = None
    
    def is_near(self, coor1, coor2):
        distance = Node(coor1).distance_to(Node(coor2))
        return distance <= DISTANCE_THRESHOLD
    
    def path_traversal(self):
        print("Traversing Path")
        if len(self.path) == 0:
            print("No Path Found")
            return
        if self.current_angle != self.path[0].angle:
            self.rotate_to(self.path[0].angle)
            self.timer.set_timeout(timeout_s=ROTATION_TIME)
            print("Set Current Angle to: ", self.current_angle)
            return
        if len(self.path) == 1:
            self.destination = None
        else:
            self.destination = self.path[1].node.coordinate
        del self.path[0]
        self.state = FREE_ROAM
        print("Set state to free roam")
        
    def free_roam(self):
        print("Free Roam State")
        if self.destination is not None:
            if self.is_near((self.pose.pose.position.x,
                             self.pose.pose.position.y), self.destination):
                self.state = PATH_TRAVERSE
                return
        self.move_forward(DISTANCE_STEP)
        self.timer.set_timeout(timeout_s=5)
    
    def move(self):
        if not self.timer.check_timeout():
            return
        
        if self.state == PATH_TRAVERSE:
            self.path_traversal()
        
        elif self.state == FREE_ROAM:
            self.free_roam()
    
    def move_forward(self, distance):
        print("Moving Forward by ", distance, "meters")
        self.pose.pose.position.x += distance*np.cos(np.deg2rad(self.current_angle))
        self.pose.pose.position.y += distance*np.sin(np.deg2rad(self.current_angle))
    
    def set_current_angle(self, angle):
        new_angle = angle % 360
        self.current_angle = new_angle
    
    def rotate_by(self, angle):
        self.set_current_angle(self.current_angle + angle)
        quat_np = quaternion_from_euler(0, 0, np.deg2rad(self.current_angle))
        self.pose.pose.orientation = Quaternion(*quat_np)
    
    def rotate_to(self, angle):
        self.set_current_angle(angle)
        quat_np = quaternion_from_euler(0, 0, np.deg2rad(self.current_angle))
        self.pose.pose.orientation = Quaternion(*quat_np)
