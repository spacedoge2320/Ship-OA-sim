import pygame
import math
import numpy as np


class VO(object):

    def __init__(self, ship_pos=np.array([100.0, 100.0]), ship_heading=0.0,
                 target_pos=np.array([400.0, 400.0]), ship_color=(50, 50, 50)):
        self.initial_ship_speed = 0.0
        self.ship_max_speed = 1.0
        self.ship_rot_spd = 0.0
        self.ship_rot_lim = 0.015
        self.ship_accel_pos_lim = 0.1
        self.ship_accel_neg_lim = 0.1
        self.ship_rot_accel_lim = 0.00015
        self.ship_points = [(20, 0), (10, -10), (-20, -10), (-20, 10), (10, 10)]
        self.ship_color = ship_color
        self.ship_pos_list = [ship_pos.tolist()]

        self.ship_pos = ship_pos
        self.ship_heading = ship_heading
        self.target_pos = target_pos
        self.ship_speed = self.initial_ship_speed
        self.polygon = []


        # needs a dictionary of ship instances in the environment
        # for each ship's polygons, evaluate if the closest point of each polygon are closer than preconfigured sensing distance.
        # if polygon is close enough, the ship instance information goes to the Velocity obstacle method. (velocity , heading , polygons)

        # for each ships,
        # the VO method first evaluates the relative velocity between Main ship and target ship.
        # it then generates ship radius based on polygon information.
        # it then generates the VO come.
        # it then passes the VO cone information to the VO_cone database

    def ship_polygon(self):
        rotated_points = []
        for point in self.ship_points:
            x, y = point
            rotated_x = x * math.cos(self.ship_heading) - y * math.sin(self.ship_heading)
            rotated_y = x * math.sin(self.ship_heading) + y * math.cos(self.ship_heading)
            rotated_points.append((rotated_x + self.ship_pos[0], rotated_y + self.ship_pos[1]))
        return rotated_points


    def move_ship(self, target_pos=[]):

        if target_pos:
            self.target_pos = target_pos

        [self.ship_speed, self.ship_rot_spd] = self.ship_los_controller(self.ship_pos, self.target_pos, self.ship_speed,
                                                                    self.ship_heading, self.ship_rot_spd)
        self.ship_position_intergrator()
        self.ship_pos_list.append(self.ship_pos.tolist())
        self.polygon = self.ship_polygon()
