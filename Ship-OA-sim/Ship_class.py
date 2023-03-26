import pygame
import math
import numpy as np
import Guidance_algorithms
import Dynamics
import Controllers


class USV(object):

    def __init__(self, initial_ship_pos_scaled=np.array([100.0, 100.0]), ship_heading=0.0,
                 target_pos_scaled=np.array([900.0, 900.0]), ship_color=(50, 50, 50), guidance_type = 1):

        # visual settings
        self.output_polygon = None
        self.scale = 40  # 40px = 1meter
        self.ship_points = [(1, 0), (0.75, 0.25), (-1, 0.25), (-1, -0.25), (0.75, -0.25)]
        self.ship_color = ship_color
        self.ship_pos_list = [initial_ship_pos_scaled.tolist()]
        self.target_pos = target_pos_scaled / self.scale

        # initial state settings
        [ship_lat_pose, ship_ax_pose] = [initial_ship_pos_scaled / self.scale, ship_heading]
        [ship_lat_vel, ship_ax_vel] = [np.array([0.0, 0.0]), 0.0]
        [ship_lat_acc, ship_ax_acc] = [np.array([0.0, 0.0]), 0.0]
        # [ pose[[x,y],rot],velocity[[x,y],rot],acceleration[[x,y],rot] ]
        self.phys_status = {'pose': [ship_lat_pose, ship_ax_pose], 'velocity': [ship_lat_vel, ship_ax_vel],
                            'acceleration': [ship_lat_acc, ship_ax_acc]}
        self.ship_scaled_pose = [ship_lat_pose * self.scale, ship_ax_pose]
        self.thrust = [0, 0]

        self.T_max_thrust = 60
        self.T_min_thrust = -60
        self.T2CL_dist = 0.4

        # controller settings
        controller_params = {
            'Kp_lin': 9.0,
            'Ki_lin': 2.0,
            'Kd_lin': 0.0,
            'Kp_ang': 9.0,
            'Ki_ang': 4.0,
            'Kd_ang': 0.04,
            'MAX_THRUST': self.T_max_thrust,
            'MIN_THRUST': self.T_min_thrust,
            'rot_thrust_weight': 500,
            'thrust_multiplier': 3.0
        }
        self.controller = Controllers.Pid_vel_controller(controller_params)
        self.prev_outputs = []

        # dynamics settings
        # 40px = 1m
        dynamics_params = {
            "mass": 10,
            "Izz": 1.25,
            "inline_drag_coefficient": 5,
            "sideways_drag_coefficient": 1000,
            "rotational_drag_coefficient": 100,
            "T2CL_dist": 0.5
        }
        self.dynamics = Dynamics.water_dynamics(dynamics_params)

        self.guidance_type = guidance_type

        if guidance_type == 1:
            # guidance settings
            guidance_params = {
                "initial_ship_speed": 0.0,
                "ship_max_speed": 1.0,
                "ship_ax_vel_lim": 0.5,
                "ship_lat_acc_pos_lim": 0.6,
                "ship_lat_acc_neg_lim": 0.6,
                "ship_ax_acc_lim": 0.15,
                "T_max_thrust": self.T_max_thrust,
                "T_min_thrust": self.T_min_thrust,
                "dt": 1 / 144
            }
            self.guidance = Guidance_algorithms.LOS_guidance(guidance_params)

        elif guidance_type == 2:
            # guidance settings
            guidance_params = {
                "initial_ship_speed": 0.0,
                "ship_max_speed": 1,
                "ship_ax_vel_lim": 0.5,
                "ship_lat_acc_pos_lim": 0.6,
                "ship_lat_acc_neg_lim": 0.6,
                "ship_ax_acc_lim": 0.15,
                "T_max_thrust": self.T_max_thrust,
                "T_min_thrust": self.T_min_thrust,
                "dt": 1 / 144
            }
            self.guidance = Guidance_algorithms.LOS_VO_guidance(guidance_params)

    def scale_and_rotate_polygons(self):
        self.ship_scaled_pose = [self.phys_status['pose'][0] * self.scale, self.phys_status['pose'][1]]
        rotated_points = []
        for point in self.ship_points:
            x, y = point
            x = x * self.scale
            y = y * self.scale

            rotated_x = x * math.cos(self.ship_scaled_pose[1]) - y * math.sin(self.ship_scaled_pose[1])
            rotated_y = x * math.sin(self.ship_scaled_pose[1]) + y * math.cos(self.ship_scaled_pose[1])
            rotated_points.append(
                (rotated_x + self.ship_scaled_pose[0][0], rotated_y + self.ship_scaled_pose[0][1]))
        return rotated_points

    # Line-of-sight controller (no pid)

    # Dynamics

    # thrust controller
    def simple_thrust_control(self, ship_phys_status, cmd_vel):

        velocity = ship_phys_status['velocity']

        MAX_THRUST = self.T_max_thrust
        MIN_THRUST = self.T_min_thrust
        BOAT_WIDTH = self.T2CL_dist * 2

        Thrust_mul = 2000
        r_m = 0.1

        speed = np.linalg.norm(velocity[0])
        vtheta = velocity[1]

        # Extract the velocity command values
        cmd_speed = np.linalg.norm(cmd_vel[0])
        cmd_vtheta = cmd_vel[1]

        # Calculate the error in velocity and angular velocity
        error_speed = cmd_speed - speed
        error_vtheta = cmd_vtheta - vtheta

        # Calculate the thrust values
        TRight = ((error_speed + (BOAT_WIDTH / 2 * error_vtheta * r_m)) / 2) * Thrust_mul
        TLeft = ((error_speed - (BOAT_WIDTH / 2 * error_vtheta * r_m)) / 2) * Thrust_mul

        # Limit the thrust values to the maximum and minimum values
        TRight = max(min(TRight, MAX_THRUST), MIN_THRUST)
        TLeft = max(min(TLeft, MAX_THRUST), MIN_THRUST)

        return TLeft, TRight

    # ship position integrator
    def ship_position_integrator(self, dt=1 / 144):
        # Adds velocity divided by tick rate to get next frame's pose

        self.phys_status['pose'][0] += dt * self.phys_status['velocity'][0]
        self.phys_status['pose'][1] += dt * self.phys_status['velocity'][1]

    def smooth(self, A, B, window_size):
        """
        Smooth a float input A based on a separate list B using a moving window of size window_size.

        Parameters:
        A (float): The input float to be smoothed.
        B (list): The list of values to use for smoothing.
        window_size (int): The size of the moving window to use for smoothing.

        Returns:
        C (float): The smoothed value of A.
        """
        n = len(B)
        if n < window_size:
            window_size = n
        half_window_size = window_size // 2
        smoothed_values = []
        for i in range(n):
            start_index = max(0, i - half_window_size)
            end_index = min(n, i + half_window_size + 1)
            values_in_window = B[start_index:end_index]
            total = sum(values_in_window)
            avg = total / len(values_in_window)
            smoothed_values.append(avg)
        if n >= window_size:
            return smoothed_values[half_window_size:-half_window_size][B.index(A)]
        else:
            return A

    # main method to generate info for the next frame
    def move_ship(self, sensor_data=None, target_pos_scaled = [900,900], log=False):

        self.ship_scaled_pose = [self.phys_status['pose'][0] * self.scale, self.phys_status['pose'][1]]
        target_pos = np.multiply(target_pos_scaled, 1 / self.scale)

        # updates the vessel's target position to the input target
        self.target_pos = target_pos

        prior_thrust = self.thrust

        # Path_planning


        # Guidance & Obstacle avoidance
        if self.guidance_type ==1:
            cmd_vel = self.guidance.ship_los_guidance(self.phys_status, target_pos=target_pos, dt=1 / 144)
            # cmd_vel = [np.array([-1, 0]), 0]
        elif self.guidance_type ==2:
            cmd_vel = self.guidance.ship_los_vo_guidance(self.phys_status, output_polygon = self.output_polygon, target_pos=target_pos, sensor_data=sensor_data, dt=1 / 144)
            #self.collision_cone = self.guidance.collision_cone
            self.vo_circles = self.guidance.vo_circles
            self.vo_cones = self.guidance.vo_cones
            self.vo_lines = self.guidance.vo_lines


        # thrust controller --validated
        self.thrust = self.controller.pid_tune_velocity(self.phys_status, cmd_vel=cmd_vel, log=log)

        # thrust_jerk_limiter

        thrust_range = self.T_max_thrust - self.T_min_thrust
        max_thrust_jerk = thrust_range/1

        self.thrust[0] = max(min(self.thrust[0], (prior_thrust[0] + max_thrust_jerk)), (prior_thrust[0] - max_thrust_jerk))
        self.thrust[1] = max(min(self.thrust[1], (prior_thrust[1] + max_thrust_jerk)),
                             (prior_thrust[1] - max_thrust_jerk))



        # vehicle dynamics simulator -- validated
        self.phys_status = self.dynamics.dynamics(ship_phys_status=self.phys_status, TR=self.thrust[1],
                                                  TL=self.thrust[0],
                                                  dt=1 / 144)  # this writes velocity into self.ship_phys_status

        # ship position integrator
        self.ship_position_integrator(dt=1 / 144)  # this writes pose into self.ship_phys_status

        # path generator
        self.ship_pos_list.append(self.ship_scaled_pose[0].tolist())

        # polygon generator
        self.output_polygon = self.scale_and_rotate_polygons()



        #self.vo_cone = self.guidance.vo_cone

