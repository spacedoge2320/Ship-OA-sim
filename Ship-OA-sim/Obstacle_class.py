import pygame
import math
import numpy as np
import Guidance_algorithms
import Dynamics
import Controllers


class square_obstacle(object):

    def __init__(self, initial_pos_scaled,heading,obstacle_color):

        # base values
        self.output_polygon = None
        self.scale = 40
        self.obstacle_points = [(2, 2), (2, -2), (-2, -2), (-2, 2)]
        self.obstacle_color = obstacle_color

        [lat_pose, ax_pose] = [initial_pos_scaled / self.scale, heading]
        [lat_vel, ax_vel] = [np.array([0.0, 0.0]), 0.0]
        [lat_acc, ax_acc] = [np.array([0.0, 0.0]), 0.0]
        self.phys_status = {'pose': [lat_pose, ax_pose], 'velocity': [lat_vel, ax_vel],
                                 'acceleration': [lat_acc, ax_acc]}
        #40px = 1meter

    def scale_and_rotate_polygons(self):
        self.scaled_pose = [self.phys_status['pose'][0] * self.scale, self.phys_status['pose'][1]]
        rotated_points = []
        for point in self.obstacle_points:
            x, y = point
            x = x * self.scale
            y = y * self.scale

            rotated_x = x * math.cos(self.scaled_pose[1]) - y * math.sin(self.scaled_pose[1])
            rotated_y = x * math.sin(self.scaled_pose[1]) + y * math.cos(self.scaled_pose[1])
            rotated_points.append(
                (rotated_x + self.scaled_pose[0][0], rotated_y + self.scaled_pose[0][1]))
        return rotated_points

    def polygon(self, ):
        # polygon generator
        self.output_polygon = self.scale_and_rotate_polygons()