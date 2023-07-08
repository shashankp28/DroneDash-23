import os
import rospy
import numpy as np
from classes.Node import Node
from config.Constants import *
from classes.Timer import Timer
from classes.NodeList import NodeList
from classes.FixedList import FixedList
from classes.Algorithm import Algorithm


class Controller:

    def __init__(self, pose, movement):
        self.pose = pose
        self.home = os.path.expanduser('~')
        self.rgb_images = FixedList(MAX_IMAGES)
        self.depth_images = FixedList(MAX_IMAGES)
        self.movement = movement
        self.all_nodes = []
        self.node_list = NodeList()
        self.prev_node = None
        self.current_node = None
        self.timer = Timer()
        self.analysed_angles = []
        self.next_destination = None
        self.temp_walls = 0
        self.counter = 0

    def crop_center(self, imgs):
        _,x,y = imgs.shape
        cropx = int(x*VERT_CROP)
        cropy = int(y*HOR_CROP)
        startx = x//2-(cropx//2)
        starty = y//2-(cropy//2)
        new_imgs = imgs[:, startx:startx+cropx,starty:starty+cropy]
        return new_imgs
    
    def get_distance_to_wall(self):
        cropped_images = self.crop_center(np.array(self.depth_images))
        distances = self.divide_and_average(cropped_images, COL_SPLITS)
        return distances
    
    def obstacle(self, threshold):
        distances = self.get_distance_to_wall()
        mean_distance = np.mean(distances)
        print("Mean Distance to Wall:", mean_distance)
        return mean_distance <= threshold

    def divide_and_average(self, images, col_split):
        _, A, B = images.shape
        column_width = B // col_split
        columns = [images[:, :, i * column_width:(i + 1) * column_width] for i in range(col_split)]
        averages = [np.mean(column) for column in columns]
        result = np.array(averages).reshape(COL_SPLITS, 1).flatten()

        return result
    
    def skewness(self, arr):
        median = sorted(arr)[len(arr)//2]
        higher_values = []
        for i, num in enumerate(arr):
            if num > median:
                higher_values.append(i - len(arr)//2)
        sum_distances = sum(higher_values)
        return 4*sum_distances/np.mean(arr)
    
    def skewness2(self, arr):
        max_value = max(arr)
        differences = [abs(max_value - num) for num in arr]
        significance = [diff / max_value * 100 for diff in differences]
        index = np.where(arr == max_value)[0]
        return ((index - COL_SPLITS//2) * np.mean(significance))[0]/10

    def colour_image_callback(self, msg):
        try:
            temp_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            self.rgb_images.fixedAppend(temp_rgb)
        except Exception as e:
            rospy.logerr(e)

    def depth_image_callback(self, msg):
        try:
            temp_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width, 1)
            self.depth_images.fixedAppend(temp_depth[:, :, 0])
        except Exception as e:
            rospy.logerr(e)

    def initialize(self):
        x, y = self.pose.pose.position.x, self.pose.pose.position.y
        new_node = Node((x, y))
        temp_node = self.node_list.return_if_same(new_node)
        if temp_node:
            new_node = temp_node
        else:
            self.node_list.add_node(new_node)
        self.current_node = new_node
        self.movement.state = ANALYZING
        self.timer.set_timeout(timeout_s=10)

    def analyze_surroundings(self):
        current_angle = self.movement.current_angle
        print("Node Children: ", self.current_node)
        if self.temp_walls < 4:
            if self.obstacle(ANA_OBSTACLE_THRESHOLD):
                self.current_node.neighbours[current_angle] = WALL
            else:
                self.current_node.neighbours[current_angle] = FREE
            self.movement.rotate_by(90)
            self.timer.set_timeout(timeout_s=ROTATION_TIME)
            self.temp_walls += 1

        elif self.temp_walls == 4:
            if self.prev_node:
                self.prev_node.neighbours[current_angle] = self.current_node
                self.current_node.neighbours[
                    (current_angle+180) % 360] = self.prev_node
            self.temp_walls = 0
            self.movement.state = THINKING
            self.prev_node = self.current_node
            self.current_node = None

    def path_finder(self):
        algorithm = Algorithm()
        path_list = algorithm.dijkstra(self.prev_node, self.node_list)
        print("-------------PATH FOUND-----------")
        for path in path_list:
            print(path, end=" ")
        print()
        print("-----------------------------")
        self.movement.path = path_list
        self.movement.state = PATH_TRAVERSE

    def update(self):
        # data_set_saver(self)
        if not self.timer.check_timeout():
            return
        if self.movement.state == FREE_ROAM and self.obstacle(MOV_OBSTACLE_THRESHOLD):
            self.movement.state = INIT

        if self.movement.state == INIT:
            self.initialize()

        elif self.movement.state == ANALYZING:
            self.analyze_surroundings()

        elif self.movement.state == THINKING:
            self.path_finder()
